import json
import typing as tp
import logging
import ipfshttpclient
import threading

from .ifeeder import IFeeder
from ..drivers.ping import PING_MODEL
from ..stations.istation import StationData
import logging.config
from connectivity.config.logging import LOGGING_CONFIG

logging.config.dictConfig(LOGGING_CONFIG)
logger = logging.getLogger("sensors-connectivity")


thlock = threading.RLock()


def _to_pubsub_message(data: StationData) -> str:
    meas = data.measurement
    message = {}
    message[meas.public] = {
        "model": meas.model,
        "geo": "{},{}".format(meas.geo_lat, meas.geo_lon),
        "measurement": meas.measurement_check(),
    }
    return json.dumps(message)


def _to_ping_message(data: StationData) -> str:
    meas = data.measurement
    message = {}
    message[meas.public] = {
        "model": meas.model,
        "timestamp": meas.timestamp,
        "measurement": {"geo": "{},{}".format(meas.geo_lat, meas.geo_lon)},
    }

    return json.dumps(message)


class RobonomicsFeeder(IFeeder):
    """
    Publishes a result or demand message to IPFS pubsub channel
    according to Robonomics communication protocol.

    It keeps track of published messages. In case it's about to publish the same data
    (same value and timestamp) it uses previously calculated IPFS hash
    """

    def __init__(self, config: dict) -> None:
        super().__init__(config)

        endpoint: str = (
            config["robonomics"]["ipfs_provider"]
            if config["robonomics"]["ipfs_provider"]
            else "/ip4/127.0.0.1/tcp/5001/http"
        )
        self.ipfs_client = ipfshttpclient.connect(endpoint, session=True)
        self.topic: str = config["robonomics"]["ipfs_topic"]

    def feed(self, data: tp.List[StationData]) -> None:
        if self.config["robonomics"]["enable"]:
            for d in data:
                if d.measurement.public and d.measurement.model != PING_MODEL:
                    pubsub_payload = _to_pubsub_message(d)
                else:
                    pubsub_payload = _to_ping_message(d)
                logger.info(f"RobonomicsFeeder: {pubsub_payload}")
                self.ipfs_client.pubsub.publish(self.topic, pubsub_payload)
