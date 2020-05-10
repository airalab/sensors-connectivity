import subprocess
import time
import rospy

from feeders import IFeeder
from stations import StationData
from .robonomics_feeder import get_multihash


class DatalogFeeder(IFeeder):
    def __init__(self, config):
        super().__init__(config)
        self.last_time = time.time()

    def feed(self, data: StationData):
        if self.config["datalog"]["enable"]:
            rospy.loginfo("DatalogFeeder:")
            ipfs_hash = get_multihash(data).multihash
            self._to_datalog(ipfs_hash)

    def _to_datalog(self, ipfs_hash: str):
        if (time.time() - self.last_time) > self.config["datalog"]["dump_interval"]:
            prog_path = [self.config["datalog"]["path"], "io", "write", "datalog",
                         "-s", self.config["datalog"]["suri"], "--remote", self.config["datalog"]["remote"]]
            output = subprocess.run(prog_path, stdout=subprocess.PIPE, input=ipfs_hash.encode(),
                               stderr=subprocess.PIPE)
            rospy.loginfo(output.stderr)
            rospy.logdebug(output)
            self.last_time = time.time()
