from feeders import IFeeder
from stations import StationData, Measurement

import json
import rospy
from std_msgs.msg import String

from robonomics_msgs.msg import Demand, Result
from ethereum_common.msg import Address, UInt256
from ethereum_common.srv import BlockNumber
from ipfs_common.msg import Multihash
from ipfs_common.ipfs_rosbag import IpfsRosBag

from collections import OrderedDict


class LRU(OrderedDict):
    """Limit size, evicting the least recently looked-up key when full
    """

    def __init__(self, maxsize=128, *args, **kwds):
        self.maxsize = maxsize
        super().__init__(*args, **kwds)

    def __getitem__(self, key):
        value = super().__getitem__(key)
        self.move_to_end(key)
        return value

    def __setitem__(self, key, value):
        super().__setitem__(key, value)
        if len(self) > self.maxsize:
            oldest = next(iter(self))
            del self[oldest]


def _get_multihash(data: StationData, geo: str = "") -> Multihash:
    rospy.logdebug(data)
    rospy.logdebug(geo)
    d = {
        "PM2.5": data.measurement.pm25,
        "PM10": data.measurement.pm10,
        "timestamp": data.measurement.timestamp
    }
    topics = {
        "/data": [String(json.dumps(d))],
        "/geo": [String(geo)]
    }
    bag = IpfsRosBag(messages=topics)
    return bag.multihash


class RobonomicsFeeder(IFeeder):
    """
    Publishes a result or demand message to IPFS pubsub channel
    according to Robonomics communication protocol.

    It keeps track of published messages. In case it's about to publish the same data
    (same value and timestamp) it uses previously calculated IPFS hash
    """

    def __init__(self, config: dict):
        super().__init__(config)

        self.result_publisher = rospy.Publisher("/liability/infochan/eth/signing/result", Result, queue_size=128)
        self.demand_publisher = rospy.Publisher("/liability/infochan/eth/signing/demand", Demand, queue_size=128)
        self.geo = config["general"]["geo"]
        self.published_data = LRU()

    def feed(self, data: StationData):
        if self.config["robonomics"]["enable"]:
            rospy.loginfo("RobonomicsFeeder:")
            if self.config["robonomics"]["result"]:
                self._result(data)
            if self.config["robonomics"]["demand"]:
                self._demand(data)

    def _result(self, data: StationData):
        res = Result()
        res.liability = Address("0x0000000000000000000000000000000000000000")

        if data.measurement in self.published_data:
            res.result = self.published_data[data.measurement]
        else:
            res.result = _get_multihash(data, self.geo)
            self.published_data[data.measurement] = res.result

        res.success = True

        self.result_publisher.publish(res)
        rospy.loginfo(f"Result published: {res.result.multihash}")

    def _demand(self, data: StationData):
        demand = Demand()

        demand.model = Multihash(self.config["robonomics"]["model"])
        demand.objective = _get_multihash(data, self.geo)
        demand.token = Address(self.config["robonomics"]["token"])
        demand.cost = UInt256("0")
        demand.lighthouse = Address(self.config["robonomics"]["lighthouse"])
        demand.validator = Address(self.config["robonomics"]["validator"])
        demand.validatorFee = UInt256(str(self.config["robonomics"]["validatorFee"]))
        demand.deadline = self._get_deadline()

        self.demand_publisher.publish(demand)
        rospy.loginfo(f"Demand published: {demand.objective.multihash}")
        rospy.logdebug(demand)

    def _get_deadline(self) -> UInt256:
        lifetime = 100  # blocks
        deadline = rospy.ServiceProxy("/eth/current_block", BlockNumber)().number + lifetime
        return UInt256(str(deadline))
