import json

import rospy
from std_msgs.msg import String

from robonomics_msgs.msg import Demand, Result
from ethereum_common.msg import Address, UInt256
from ethereum_common.srv import BlockNumber
from ipfs_common.msg import Multihash
from ipfs_common.ipfs_rosbag import IpfsRosBag

from sds011.station import StationData


def get_multihash(data: StationData, geo: str = "") -> Multihash:
    rospy.logdebug(data)
    rospy.logdebug(geo)
    d = {
        "PM2.5": data.meas.pm25,
        "PM10": data.meas.pm10
    }
    topics = {
        "/data": [String(json.dumps(d))],
        "/geo": [String(geo)]
    }
    bag = IpfsRosBag(messages=topics)
    return bag.multihash


class RobonomicsFeeder:
    def __init__(self,
                 result_publisher: rospy.Publisher,
                 demand_publisher: rospy.Publisher,
                 config: dict,
                 geo: str = ""):
        self.config = config

        self.result_publisher = result_publisher
        self.demand_publisher = demand_publisher
        self.geo = geo

    def feed(self, data: StationData):
        if self.config["enable"]:
            rospy.loginfo("RobonomicsFeeder:")
            if self.config["result"]:
                self._result(data)
            if self.config["demand"]:
                self._demand(data)

    def _result(self, data: StationData):
        res = Result()
        res.liability = Address("0x0000000000000000000000000000000000000000")
        res.result = get_multihash(data, self.geo)
        res.success = True

        self.result_publisher.publish(res)
        rospy.loginfo(f"Result published: {res.result.multihash}")

    def _demand(self, data: StationData):
        demand = Demand()

        demand.model = Multihash(self.config["model"])
        demand.objective = get_multihash(data, self.geo)
        demand.token = Address(self.config["token"])
        demand.cost = UInt256("0")
        demand.lighthouse = Address(self.config["lighthouse"])
        demand.validator = Address(self.config["validator"])
        demand.validatorFee = UInt256(str(self.config["validatorFee"]))
        demand.deadline = self._get_deadline()

        self.demand_publisher.publish(demand)
        rospy.loginfo(f"Demand published: {demand.objective.multihash}")
        rospy.logdebug(demand)

    def _get_deadline(self) -> UInt256:
        lifetime = 100      # blocks
        deadline = rospy.ServiceProxy('/eth/current_block', BlockNumber)().number + lifetime
        return UInt256(str(deadline))






