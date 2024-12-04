# -*- coding: utf-8 -*-
"""Station to input data via MQTT."""
import json
import logging.config
import threading
import typing as tp

import paho.mqtt.client as mqtt

from connectivity.config.logging import LOGGING_CONFIG

from connectivity.constants import STATION_VERSION
from connectivity.src.sensors import SensorsFabcric
from .istation import IStation

logging.config.dictConfig(LOGGING_CONFIG)
logger = logging.getLogger("sensors-connectivity")

thlock = threading.RLock()
sessions = dict()
sensors_data = dict()


class LoraHandler(mqtt.Client):
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
            logger.info(f"Lora Station: Connected OK with result code {str(rc)}")
            self.subscribe(self.topic, 0)
        elif rc == 5:
            logger.error(f"Lora Station: Connection Refused: Authorization error")
        else:
            logger.error(f"Lora Station: Connection refused with result code {str(rc)}")

    def on_message(self, client, userdata, msg: dict) -> None:
        """Callback for message receiving. Parse data from a sensor and store it into `sessions`.

        :param msg: MQTT message.
        """

        global thlock
        global sessions
        data = json.loads(msg.payload.decode())
        meas = SensorsFabcric.get_sensor(data)
        if meas is None:
            return
        
        with thlock:
            if meas:
                sessions[meas.id] = meas

    def on_subscribe(self, client, userdata, mid, granted_qos) -> None:
        """Subscription callback."""
        logger.info(f"Lora Station: Subscribed {str(mid)} to topic {self.topic}")

    def run(self) -> None:
        """Service function for MQTT handler."""

        self.connect_async(self.host, self.port, 60)
        self.loop_start()


class LoraStation(IStation):
    """Station to input data via MQTT."""

    def __init__(self, config: dict) -> None:
        """Initialize MQTT client based on credentials from config.

        :param config: Dict with configuration file.
        """

        self.DEAD_SENSOR_TIME: int = 60 * 60  # 1 hour
        self.version = STATION_VERSION
        host: str = config["lorastation"]["host"]
        port: int = int(config["lorastation"]["port"])
        topic: str = config["lorastation"]["topic"]
        username: str = config["lorastation"]["username"]
        password: str = config["lorastation"]["password"]
        client = LoraHandler(host, port, topic, username, password)
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
