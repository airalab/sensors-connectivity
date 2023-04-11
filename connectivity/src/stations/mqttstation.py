# -*- coding: utf-8 -*-
"""Station to input data via MQTT."""
import json
import logging.config
import threading
import typing as tp

import paho.mqtt.client as mqtt

from connectivity.config.logging import LOGGING_CONFIG

from ...constants import STATION_VERSION
from ..sensors import EnvironmentalBox, LoraSensor, MobileLab
from .istation import IStation

logging.config.dictConfig(LOGGING_CONFIG)
logger = logging.getLogger("sensors-connectivity")

thlock = threading.RLock()
sessions = dict()


class MQTTHandler(mqtt.Client):
    """Service class to handle MQTT messages."""

    def __init__(self, host: str, port: int, topic: str, username: str, password: str) -> None:
        """Initialize MQTT client.

        :param host: MQTT broker host.
        :param port: MQTT broker port.
        :param topic: MQTT topic where messages are published.
        :param username: Username for MQTT broker authorization.
        :param password: Password for MQTT broker authorization.
        """

        mqtt.Client.__init__(self)
        self.host: str = host
        self.port: int = port
        self.topic: str = topic
        self.username: str = username
        self.password: str = password
        if self.username and self.password:
            self.username_pw_set(username=self.username, password=self.password)

    def on_connect(self, client, obj, flags, rc) -> None:
        """Connect callback. If connects successfully, it subscribes on the `topic`.

        :param rc: Result connection code.
        """

        if rc == 0:
            logger.info(f"MQTT Station: Connected OK with result code {str(rc)}")
            self.subscribe(self.topic, 0)
        elif rc == 5:
            logger.error(f"MQTT Station: Connection Refused: Authorization error")
        else:
            logger.error(f"MQTT Station: Connection refused with result code {str(rc)}")

    def on_message(self, client, userdata, msg: dict) -> None:
        """Callback for message receiving. Parse data from a sensor and store it into `sessions`.

        :param msg: MQTT message.
        """

        global thlock
        global sessions
        data = json.loads(msg.payload.decode())
        if "esp8266id" in data.keys():
            meas = EnvironmentalBox(data)
        elif "ID" in data.keys():
            meas = MobileLab(data)
        elif "decoded_payload" in data["uplink_message"]:
            id = data["end_device_ids"]["device_id"]
            meas = LoraSensor(id=id, data=data["uplink_message"]["decoded_payload"])
        else:
            return
        with thlock:
            if meas:
                sessions[meas.id] = meas

    def on_subscribe(self, client, userdata, mid, granted_qos) -> None:
        """Subscription callback."""
        logger.info(f"Subscribed {str(mid)} to topic {self.topic}")

    def run(self) -> None:
        """Service function for MQTT handler."""

        self.connect_async(self.host, self.port, 60)
        self.loop_start()


class MQTTStation(IStation):
    """Station to input data via MQTT."""

    def __init__(self, config: dict) -> None:
        """Initialize MQTT client based on credentials from config.

        :param config: Dict with configuration file.
        """

        self.DEAD_SENSOR_TIME: int = 60 * 60  # 1 hour
        self.version = STATION_VERSION
        host: str = config["mqttstation"]["host"]
        port: int = int(config["mqttstation"]["port"])
        topic: str = config["mqttstation"]["topic"]
        username: str = config["mqttstation"]["username"]
        password: str = config["mqttstation"]["password"]
        client = MQTTHandler(host, port, topic, username, password)
        rc = client.run()

    def get_data(self) -> tp.List[dict]:
        """Main function of the class.

        :return: Formatetd data.
        """

        global sessions
        global thlock
        result = []
        with thlock:
            stripped = self.drop_dead_sensors(sessions)
        for k, v in stripped.items():
            result.append(v)
        return result
