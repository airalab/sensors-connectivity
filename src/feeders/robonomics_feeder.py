import json
import rospy
import ipfshttpclient
from feeders import IFeeder
from drivers.ping import PING_MODEL
from stations import StationData
import jsonpickle
from json import JSONEncoder
import threading


thlock = threading.RLock()
def _to_pubsub_message(data: StationData) -> str:
    meas = data.measurement

    message = {}
    message[meas.public] = {
        "model": meas.model,
        "geo": "{},{}".format(meas.geo_lat, meas.geo_lon),
        "measurement": {
            meas.to_json()
        }
    }

    return jsonpickle.encode(message)

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


# def _to_pubsub_mGPS_data(data: StationData) -> str:
#     meas = data.measurement

#     message = {}
#     message[meas.public] = {
#         "model": meas.model,
#         "timestamp": meas.timestamp,
#         "measurement": {
#             "geo": "{},{}".format(meas.geo_lat, meas.geo_lon),
#             "temperature": meas.temperature,
#             "pressure": meas.pressure,
#             "humidity": meas.humidity,
#             "ph": meas.ph,
#             "conductivity": meas.conductivity
#         }
#     }

#     return json.dumps(message)

# def _to_pubsub_BME_data(data: StationData) -> str:
#     meas = data.measurement
#     message = {}
#     message[meas.public] = {
#         "model": meas.model,
#         "timestamp": meas.timestamp,
#         "measurement": {
#             "pm25": meas.pm25,
#             "pm10": meas.pm10,
#             "geo": "{},{}".format(meas.geo_lat, meas.geo_lon),
#             "temperature": meas.temperature,
#             "pressure": meas.pressure,
#             "humidity": meas.humidity
#         }
#     }
#     return json.dumps(message)



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
        #publisher_list = [_to_ping_message, _to_pubsub_message, _to_pubsub_mGPS_data]
        if self.config["robonomics"]["enable"]:
            for d in data:
                # if d.measurement.public:
                #     pubsub_payload = publisher_list[d.measurement.model - 1](d)
                
                if d.measurement.public and d.measurement.model != PING_MODEL:
                   pubsub_payload = _to_pubsub_message(d)
                else:
                   pubsub_payload = _to_ping_message(d)


                rospy.loginfo(f"RobonomicsFeeder: {pubsub_payload}")
                self.ipfs_client.pubsub.publish(self.topic, pubsub_payload)

