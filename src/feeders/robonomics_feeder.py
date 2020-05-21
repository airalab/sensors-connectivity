from feeders import IFeeder
from stations import StationData, Measurement

import json
import rospy
import ipfshttpclient
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

def _to_pubsub_message(data: StationData) -> str:
    meas = data.measurement

    message = {}
    message[meas.public] = {
        "model": meas.model,
        "timestamp": meas.timestamp,
        "measurement": {
            "pm25": meas.pm25,
            "pm10": meas.pm10,
            "geo": "{},{}".format(meas.geo_lat, meas.geo_lon)
        }
    }

    return json.dumps(message)


class RobonomicsFeeder(IFeeder):
    """
    Publishes a result or demand message to IPFS pubsub channel
    according to Robonomics communication protocol.

    It keeps track of published messages. In case it's about to publish the same data
    (same value and timestamp) it uses previously calculated IPFS hash
    """

    def __init__(self, config: dict):
        super().__init__(config)

        # self.result_publisher = rospy.Publisher("/liability/infochan/eth/signing/result", Result, queue_size=128)
        # self.demand_publisher = rospy.Publisher("/liability/infochan/eth/signing/demand", Demand, queue_size=128)
        # self.geo = "" # config["general"]["geo"]
        self.published_data = LRU()
        self.ipfs_client = ipfshttpclient.connect()
        self.topic = config["robonomics"]["ipfs_topic"]

    def feed(self, data: StationData):
        if self.config["robonomics"]["enable"]:
            if data.measurement.public:
                pubsub_payload = _to_pubsub_message(data)
                rospy.loginfo(f"RobonomicsFeeder: {pubsub_payload}")
                self.ipfs_client.pubsub.publish(self.topic, pubsub_payload)

