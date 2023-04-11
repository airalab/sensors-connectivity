import json
import logging
import logging.config
import os
import tempfile
import threading
import typing as tp

import ipfshttpclient2

from connectivity.config.logging import LOGGING_CONFIG

from ...constants import PING_MODEL
from .ifeeder import IFeeder

logging.config.dictConfig(LOGGING_CONFIG)
logger = logging.getLogger("sensors-connectivity")


thlock = threading.RLock()


def _to_pubsub_message(data: dict) -> str:
    """Prepare JSON formatted string with measurements.

    :param data: Dict with the last measurement from one sensor.
    :return: JSON formatted string for pubsub.
    """

    message = {}
    message[data.public] = {
        "model": data.model,
        "geo": "{},{}".format(data.geo_lat, data.geo_lon),
        "measurement": data.measurement,
    }
    return json.dumps(message)


def _to_ping_message(data: dict) -> str:
    """Prepare JSON formatted string with base info about sensor.
    No measurements.

    :param data: Dict with the base info from one sensor.
    :return: JSON formatted string for pubsub.
    """

    message = {}
    message[data.public] = {
        "model": data.model,
        "timestamp": data.measurement.timestamp,
        "measurement": {"geo": "{},{}".format(data.geo_lat, data.geo_lon)},
    }

    return json.dumps(message)


class RobonomicsFeeder(IFeeder):
    """
    Publishes a result or demand message to IPFS pubsub channel
    according to Robonomics communication protocol.
    """

    def __init__(self, config: dict) -> None:
        """Initialize IPFS client.

        :param config: Dict with configuration.
        """

        super().__init__(config)

        endpoint: str = (
            config["robonomics"]["ipfs_provider"]
            if config["robonomics"]["ipfs_provider"]
            else "/ip4/127.0.0.1/tcp/5001/http"
        )
        self.ipfs_client = ipfshttpclient2.connect(endpoint, session=True)
        self.topic: str = config["robonomics"]["ipfs_topic"]

    def _publish_to_topic(self, payload):

        if int(self.ipfs_client.version()["Version"].split(".")[1]) < 11:
            return self.ipfs_client.pubsub.publish_old(self.topic, payload)

        if isinstance(payload, str) and not os.path.exists(payload):
            payload = payload.encode()
        if isinstance(payload, bytes) or isinstance(payload, bytearray):
            with tempfile.NamedTemporaryFile() as tp:
                tp.write(payload)
                tp.flush()
                self.ipfs_client.pubsub.publish(self.topic, tp.name)
        else:
            self.ipfs_client.pubsub.publish(self.topic, payload)

    def feed(self, data: tp.List[dict]) -> None:
        """Send data to IPFS pubsub in the topic from config.

        :param data: Data from the stations.
        """
        if self.config["robonomics"]["enable"]:
            for d in data:
                if d.public and d.model != PING_MODEL:
                    pubsub_payload = _to_pubsub_message(d)
                else:
                    pubsub_payload = _to_ping_message(d)
                logger.info(f"RobonomicsFeeder: {pubsub_payload}")
                self._publish_to_topic(pubsub_payload)
