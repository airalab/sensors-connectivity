import json
import rospy
import ipfshttpclient
from feeders import IFeeder
from drivers.ping import PING_MODEL
from stations import StationData
import threading


thlock = threading.RLock()
def _to_pubsub_message(data: StationData) -> str:
    meas = data.measurement

    message = {}
    message[meas.public] = {
        "model": meas.model,
        "geo": "{},{}".format(meas.geo_lat, meas.geo_lon),
        "measurement": meas.measurement_check()
    }
    return json.dumps(message)

def _to_ping_message(data: StationData) -> str:
    meas = data.measurement

    message = {}
    message[meas.public] = {
        "model": meas.model,
        "timestamp": meas.timestamp,
        "measurement": {
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

        endpoint = config["robonomics"]["ipfs_provider"] if config["robonomics"]["ipfs_provider"] else "/ip4/127.0.0.1/tcp/5001/http"
        self.ipfs_client = ipfshttpclient.connect(endpoint)
        self.topic = config["robonomics"]["ipfs_topic"]

    def feed(self, data: [StationData]):
        if self.config["robonomics"]["enable"]:
            for d in data:
                if d.measurement.public and d.measurement.model != PING_MODEL:
                   pubsub_payload = _to_pubsub_message(d)
                else:
                   pubsub_payload = _to_ping_message(d)

                rospy.loginfo(f"RobonomicsFeeder: {pubsub_payload}")
                self.ipfs_client.pubsub.publish(self.topic, pubsub_payload)

