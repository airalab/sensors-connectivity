import subprocess
import time

from sds011.station import StationData
from .robonomics_feeder import get_multihash


class DatalogFeeder:
    def __init__(self, config, geo):
        self.config = config
        self.last_time = time.time()

    def feed(self, data: StationData):
        if self.config["enable"]:
            rospy.loginfo("DatalogFeeder:")
            ipfs_hash = get_multihash(data).multihash
            self._to_datalog(ipfs_hash)

    def _to_datalog(self, ipfs_hash: str):
        if (time.time() - self.last_time) > self.config["dump-interval"]:
            prog_path = [self.config["path"], "io", "write", "datalog",
                         "-s", self.config["suri"], "--remote", self.config["remote"]]
            output = subprocess.run(prog_path, stdout=subprocess.PIPE, input=ipfs_hash.encode(),
                               stderr=subprocess.PIPE)
            rospy.loginfo(output.stderr)
            rospy.logdebug(output)
            self.last_time = time.time()
