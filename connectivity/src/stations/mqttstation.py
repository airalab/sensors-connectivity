# -*- coding: utf-8 -*-
import paho.mqtt.client as mqtt
import typing as tp
import threading
import json
import logging.config

from ..sensors import EnvironmentalBox, MobileLab
from .istation import IStation
from ...constants import STATION_VERSION
from connectivity.config.logging import LOGGING_CONFIG

logging.config.dictConfig(LOGGING_CONFIG)
logger = logging.getLogger("sensors-connectivity")

thlock = threading.RLock()
sessions = dict()

class MQTTHandler(mqtt.Client):
    def __init__(
        self, host: str, port: int, topic: str, username: str, password: str
    ) -> None:
        mqtt.Client.__init__(self)
        self.host: str = host
        self.port: int = port
        self.topic: str = topic
        self.username: str = username
        self.password: str = password
        if self.username and self.password:
            self.username_pw_set(username=self.username, password=self.password)

    def on_connect(self, client, obj, flags, rc) -> None:
        if rc == 0:
            logger.info(f"MQTT Station: Connected OK with result code {str(rc)}")
            self.subscribe(self.topic, 0)
        elif rc == 5:
            logger.error(f"MQTT Station: Connection Refused: Authorization error")
        else:
            logger.error(f"MQTT Station: Connection refused with result code {str(rc)}")

    def on_message(self, client, userdata, msg: dict) -> None:
        global thlock
        global sessions
        data = json.loads(msg.payload.decode())
        if "esp8266id" in data.keys():
            meas = EnvironmentalBox(data)
        elif "ID" in data.keys():
            meas = MobileLab(data)
        with thlock:
            if meas:
                sessions[meas.id] = meas

    def on_subscribe(self, client, userdata, mid, granted_qos) -> None:
        logger.info(f"Subscribed {str(mid)} to topic {self.topic}")

    def run(self) -> None:
        self.connect_async(self.host, self.port, 60)
        self.loop_start()


class MQTTStation(IStation):
    def __init__(self, config: dict) -> None:
        # super().__init__(config)
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
        global sessions
        global thlock
        result = []
        with thlock:
            stripped = self.drop_dead_sensors(sessions)
        for k, v in stripped.items():
            result.append(v)
        return result

