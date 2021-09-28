import json
from logging import raiseExceptions
import subprocess
import time
import rospy
from tempfile import NamedTemporaryFile
import ipfshttpclient
from substrateinterface import SubstrateInterface, Keypair
from substrateinterface.exceptions import SubstrateRequestException
import requests
import threading
from utils.database import DataBase

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

def _get_multihash(buf: set, db: object, endpoint: str = "/ip4/127.0.0.1/tcp/5001/http") -> (str, str):
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
            db.add_data("not sent", response["Hash"], time.time(), json.dumps(payload))
            return (response["Hash"], temp.name)
    
def _substrate_connect(endpoint: str) -> SubstrateInterface:
    substrate = SubstrateInterface(
        url=endpoint,
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
    return substrate

def _substrate_call(substrate: SubstrateInterface, ipfs_hash: str):
    call = substrate.compose_call(
                    call_module = "Datalog",
                    call_function = "record",
                    call_params = {
                        "record": ipfs_hash
                    }
                )
    return call

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
        self.db = DataBase(self.config)
        self.db.create_table()

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
                    ipfs_hash, file_path = _get_multihash(self.buffer, self.db, self.ipfs_endpoint)
                    self._pin_to_temporal(file_path)
                    self.to_datalog(ipfs_hash)
                else:
                    rospy.loginfo("Nothing to publish")
                # self.buffer = set()
                # self.last_time = time.time()
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


    def to_datalog(self, ipfs_hash: str):
        rospy.loginfo(ipfs_hash)
        self.last_time = time.time()
        self.buffer = set()
        robonomics_keypair = Keypair.create_from_mnemonic(self.config["frontier"]["suri"]) 
        ipci_keypair = Keypair.create_from_seed(self.config["datalog"]["suri"]) 
        robonomics_connect = _substrate_connect(endpoint=self.config["frontier"]["remote"])
        ipci_connect = _substrate_connect(endpoint=self.config["datalog"]["remote"])
        robonomics_call = _substrate_call(substrate=robonomics_connect, ipfs_hash=ipfs_hash)
        ipci_call = _substrate_call(substrate=ipci_connect, ipfs_hash=ipfs_hash)

        robonomics_extrinsic = robonomics_connect.create_signed_extrinsic(call=robonomics_call, keypair=robonomics_keypair)
        ipci_extrinsic = ipci_connect.create_signed_extrinsic(call=ipci_call, keypair=ipci_keypair)

        try:
            ipci_receipt = ipci_connect.submit_extrinsic(ipci_extrinsic, wait_for_inclusion=True)
            rospy.loginfo(f'Ipfs hash sent to DAO IPCI and included in block {ipci_receipt.block_hash}')
        except SubstrateRequestException as e:
            rospy.loginfo(f'Something went wrong during extrinsic submission to DAO IPCI: {e}')
        try:
            robonomics_receipt = robonomics_connect.submit_extrinsic(robonomics_extrinsic, wait_for_inclusion=True)
            rospy.loginfo(f'Ipfs hash sent to Robonomics and included in block {robonomics_receipt.block_hash}')
            self.db.update_status("sent", ipfs_hash)
        except SubstrateRequestException as e:
            rospy.loginfo(f'Something went wrong during extrinsic submission to Robonomics: {e}')


        


