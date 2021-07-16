import json
import subprocess
import time
import rospy
from tempfile import NamedTemporaryFile
import ipfshttpclient
from substrateinterface import SubstrateInterface, Keypair
import requests
import threading

from feeders import IFeeder
from stations import StationData, Measurement
from drivers.ping import PING_MODEL

thlock = threading.RLock()

def _sort_payload(data: dict) -> dict:
    ordered = {}
    for k,v in data.items():
        meas = sorted(v["measurements"], key=lambda x: x["timestamp"])
        ordered[k] = {"model":v["model"], "geo":v["geo"], "measurements":meas}

    return ordered

def _get_multihash(buf: set, endpoint: str = "/ip4/127.0.0.1/tcp/5001/http") -> (str, str):
    payload = {}

    for m in buf:
        if m.public in payload:
            payload[m.public]["measurements"].append(m.measurement_check())
        else:
            payload[m.public] = {
                "model": m.model,
                "geo": "{},{}".format(m.geo_lat, m.geo_lon),
                "measurements": [
                    m.measurement_check()
                ]
            }

    rospy.logdebug(f"Payload before sorting: {payload}")
    payload = _sort_payload(payload)
    rospy.logdebug(f"Payload sorted: {payload}")

    temp = NamedTemporaryFile(mode="w", delete=False)
    rospy.logdebug(f"Created temp file: {temp.name}")
    temp.write(json.dumps(payload))
    temp.close()

    #with ipfshttpclient.connect(endpoint, session=True) as client:
    with ipfshttpclient.connect(endpoint) as client:
        response = client.add(temp.name)
        return (response["Hash"], temp.name)


class DatalogFeeder(IFeeder):
    """
    The feeder is responsible for collecting measurements and
    publishing an IPFS hash to Robonomics on Substrate

    It requires the full path to `robonomics` execution binary
    and an account's private key
    """

    def __init__(self, config):
        super().__init__(config)
        self.last_time = time.time()
        self.buffer = set()
        self.interval = self.config["datalog"]["dump_interval"]
        self.ipfs_endpoint = config["robonomics"]["ipfs_provider"] if config["robonomics"]["ipfs_provider"] else "/ip4/127.0.0.1/tcp/5001/http"
        self.keypair = Keypair.create_from_seed(seed_hex=self.config["datalog"]["suri"], ss58_format=32 ) 

    def feed(self, data: [StationData]):
        if self.config["datalog"]["enable"]:
            rospy.loginfo("DatalogFeeder:")
            for d in data:
                if d.measurement.public and d.measurement.model != PING_MODEL:
                    rospy.logdebug(f"Adding data to buffer: {d.measurement}")
                    self.buffer.add(d.measurement)

            if (time.time() - self.last_time) >= self.interval:
                if self.buffer:
                    rospy.logdebug("About to publish collected data...")
                    rospy.logdebug(f"Buffer is {self.buffer}")
                    ipfs_hash, file_path = _get_multihash(self.buffer, self.ipfs_endpoint)
                    self._pin_to_temporal(file_path)
                    self._to_datalog(ipfs_hash)
                else:
                    rospy.loginfo("Nothing to publish")
                self.buffer = set()
                self.last_time = time.time()
            else:
                rospy.loginfo("Still collecting measurements...")

    def _pin_to_temporal(self, file_path: str):
        username = self.config["datalog"]["temporal_username"]
        password = self.config["datalog"]["temporal_password"]
        if username and password:
            auth_url = "https://api.temporal.cloud/v2/auth/login"
            token_resp = requests.post(auth_url, json={"username": username, "password": password})
            token = token_resp.json()

            url_add = "https://api.temporal.cloud/v2/ipfs/public/file/add"
            headers = {"Authorization": f"Bearer {token['token']}"}
            resp = requests.post(url_add, files={"file":open(file_path), "hold_time":(None,1)}, headers=headers)

            if resp.status_code == 200:
                rospy.loginfo("File pinned to Temporal Cloud")

    def _to_datalog(self, ipfs_hash: str):
        rospy.loginfo(ipfs_hash)
        substrate = SubstrateInterface(
            url="wss://ipci.rpc.robonomics.network",
            ss58_format=32,
            type_registry_preset="substrate-node-template",
            type_registry={
                "types": {
                    "Record": "Vec<u8>",
                    "<T as frame_system::Config>::AccountId": "AccountId",
                    "RingBufferItem": {
                        "type": "struct",
                        "type_mapping": [
                            ["timestamp", "Compact<u64>"],
                            ["payload", "Vec<u8>"],
                        ],
                    },
                }
            },
        )
        call = substrate.compose_call(
            call_module = "Datalog",
            call_function = "record",
            call_params = {
                "record": ipfs_hash
            }
        )
        extrinsic = substrate.create_signed_extrinsic(call=call, keypair=self.keypair)
        rospy.loginfo(f"extrincsic is: {extrinsic}")
        try:
            receipt = substrate.submit_extrinsic(extrinsic, wait_for_inclusion=True)
            rospy.loginfo(f'ipfs hash sent and included in block {receipt.block_hash}')
        except Exception as e:
            rospy.loginfo(f'something went wrong during extrinsic submission: {e}')
        
