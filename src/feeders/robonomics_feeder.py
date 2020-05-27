import json
import rospy
import ipfshttpclient
from feeders import IFeeder
from stations import StationData


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

        self.ipfs_client = ipfshttpclient.connect()
        self.topic = config["robonomics"]["ipfs_topic"]

    def feed(self, data: [StationData]):
        if self.config["robonomics"]["enable"]:
            for d in data:
                if d.measurement.public:
                    pubsub_payload = _to_pubsub_message(d)
                    rospy.loginfo(f"RobonomicsFeeder: {pubsub_payload}")
                    self.ipfs_client.pubsub.publish(self.topic, pubsub_payload)

