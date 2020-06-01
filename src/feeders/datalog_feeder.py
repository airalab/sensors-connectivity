import json
import subprocess
import time
import rospy
from tempfile import NamedTemporaryFile
import ipfshttpclient

from feeders import IFeeder
from stations import StationData, Measurement


def _create_row(m: Measurement) -> dict:
    return {
        "pm25": m.pm25,
        "pm10": m.pm10,
        "geo": "{},{}".format(m.geo_lat, m.geo_lon),
        "timestamp": m.timestamp
    }

def _sort_payload(data: dict) -> dict:
    ordered = {}
    for k,v in data.items():
        meas = sorted(v["measurements"], key=lambda x: x["timestamp"])
        ordered[k] = {"model":v["model"], "measurements":meas}

    return ordered

def _get_multihash(buffer: set, endpoint: str = "/ip4/127.0.0.1/tcp/5001/http") -> str:
    payload = {}

    for m in buffer:
        if m.public in payload:
            payload[m.public]["measurements"].append(_create_row(m))
        else:
            payload[m.public] = {
                "model": m.model,
                "measurements": [
                    _create_row(m)
                ]
            }

    payload = _sort_payload(payload)

    temp = NamedTemporaryFile(mode="w", delete=False)
    temp.write(json.dumps(payload))
    temp.close()

    with ipfshttpclient.connect(endpoint) as client:
        response = client.add(temp.name)
        return response["Hash"]


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

    def feed(self, data: [StationData]):
        if self.config["datalog"]["enable"]:
            rospy.loginfo("DatalogFeeder:")
            for d in data:
                if d.measurement.public:
                    self.buffer.add(d.measurement)

            if (time.time() - self.last_time) >= self.interval:
                if self.buffer:
                    ipfs_hash = _get_multihash(self.buffer, self.ipfs_endpoint)
                    self._to_datalog(ipfs_hash)
                else:
                    rospy.loginfo("Nothing to publish")
                self.buffer = set()
                self.last_time = time.time()
            else:
                rospy.loginfo("Still collecting measurements...")

    def _to_datalog(self, ipfs_hash: str):
        rospy.loginfo(ipfs_hash)
        prog_path = [self.config["datalog"]["path"], "io", "write", "datalog",
                     "-s", self.config["datalog"]["suri"], "--remote", self.config["datalog"]["remote"]]
        output = subprocess.run(prog_path, stdout=subprocess.PIPE, input=ipfs_hash.encode(),
                           stderr=subprocess.PIPE)
        rospy.loginfo(output.stderr)
        rospy.logdebug(output)
