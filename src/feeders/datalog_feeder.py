import json
import subprocess
import time
import rospy

from std_msgs.msg import String
from feeders import IFeeder
from stations import StationData
from ipfs_common.ipfs_rosbag import IpfsRosBag


def _get_multihash(buffer: set, geo: str = "") -> str:
    data_topic_payload = []

    for m in buffer:
        d = {
            "PM2.5": m.pm25,
            "PM10": m.pm10,
            "timestamp": m.timestamp
        }

        data_topic_payload.append(String(json.dumps(d)))

    topics = {
        "/data": data_topic_payload,
        "/geo": [String(geo)]
    }

    rospy.loginfo(topics)
    bag = IpfsRosBag(messages=topics)
    return bag.multihash.multihash


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
        self.geo = self.config["general"]["geo"]

    def feed(self, data: StationData):
        if self.config["datalog"]["enable"]:
            rospy.loginfo("DatalogFeeder:")
            self.buffer.add(data.measurement)

            if (time.time() - self.last_time) >= self.interval:
                ipfs_hash = _get_multihash(self.buffer, self.geo)
                self._to_datalog(ipfs_hash)
                self.buffer = set()
                self.last_time = time.time()
            else:
                rospy.loginfo("Still collecting measurements...")

    def _to_datalog(self, ipfs_hash: str):
        prog_path = [self.config["datalog"]["path"], "io", "write", "datalog",
                     "-s", self.config["datalog"]["suri"], "--remote", self.config["datalog"]["remote"]]
        output = subprocess.run(prog_path, stdout=subprocess.PIPE, input=ipfs_hash.encode(),
                           stderr=subprocess.PIPE)
        rospy.loginfo(output.stderr)
        rospy.logdebug(output)
