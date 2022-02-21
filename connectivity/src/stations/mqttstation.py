# -*- coding: utf-8 -*-
import paho.mqtt.client as mqtt
import typing as tp
import threading
import time
import json
import copy
import hashlib
import logging.config

from ..drivers.sds011 import SDS011_MODEL, MOBILE_GPS
from .istation import IStation, StationData, Measurement, STATION_VERSION
from connectivity.config.logging import LOGGING_CONFIG

logging.config.dictConfig(LOGGING_CONFIG)
logger = logging.getLogger("sensors-connectivity")

thlock = threading.RLock()
sessions = dict()


def _generate_pubkey(id: str) -> str:
    verify_key = hashlib.sha256(id.encode("utf-8"))
    verify_key_hex = verify_key.hexdigest()
    return str(verify_key_hex)


class MQTTHandler(mqtt.Client):
    def __init__(self, host: str, port: int) -> None:
        mqtt.Client.__init__(self)
        self.host: str = host
        self.port: int = port

    def on_connect(self, client, obj, flags, rc) -> None:
        logger.info(f"MQTT Station: Connected to mqtt with result code {str(rc)}")
        self.subscribe("/freertos_mqtt_robonomics_example/#", 0)

    def _parser(self, data: dict) -> Measurement:
        global sessions
        global thlock
        paskal = 133.32

        try:
            if "esp32mac" in data.keys():
                self.client_id = data["esp32mac"]
                temperature = None
                pressure = None
                humidity = None
                for d in data["sensordatavalues"]:
                    if d["value_type"] == "SDS_P1":
                        pm10 = float(d["value"])
                    if d["value_type"] == "SDS_P2":
                        pm25 = float(d["value"])
                    if d["value_type"] == "GPS_lat":
                        geo_lat = d["value"]
                    if d["value_type"] == "GPS_lon":
                        geo_lon = d["value"]

                    if "temperature" in d["value_type"]:
                        temperature = float(d["value"])
                    if "pressure" in d["value_type"]:
                        pressure = float(d["value"]) / paskal
                    if "humidity" in d["value_type"]:
                        humidity = float(d["value"])

                meas = {}
                model = SDS011_MODEL
                meas.update(
                    {
                        "pm10": pm10,
                        "pm25": pm25,
                        "temperature": temperature,
                        "pressure": pressure,
                        "humidity": humidity,
                    }
                )

            elif "ID" in data.keys():
                self.client_id = data["ID"]
                temperature = None
                pressure = None
                humidity = None
                CO = None
                NH3 = None
                NO2 = None
                speed = None
                vane = None
                pm1 = None
                pm10 = None
                pm25 = None

                if "temperature" in data.keys():
                    temperature = float(data["temperature"])
                if "humidity" in data.keys():
                    humidity = float(data["humidity"])
                if "pressure" in data.keys():
                    pressure = float(data["pressure"]) / paskal
                if "CO" in data.keys():
                    CO = float(data["CO"])
                if "NH3" in data.keys():
                    NH3 = float(data["NH3"])
                if "NO2" in data.keys():
                    NO2 = float(data["NO2"])
                if "speed" in data.keys():
                    speed = float(data["speed"])
                if "vane" in data.keys():
                    vane = data["vane"]
                if "PM1" in data.keys():
                    pm1 = data["PM1"]
                if "PM10" in data.keys():
                    pm10 = data["PM10"]
                if "PM25" in data.keys():
                    pm25 = data["PM25"]

                geo_lat = float(data["GPS_lat"])
                geo_lon = float(data["GPS_lon"])

                meas = {}
                model = MOBILE_GPS
                meas.update(
                    {
                        "temperature": temperature,
                        "humidity": humidity,
                        "pressure": pressure,
                        "CO": CO,
                        "NH3": NH3,
                        "NO2": NO2,
                        "speed": speed,
                        "vane": vane,
                        "pm1": pm1,
                        "pm10": pm10,
                        "pm25": pm25,
                    }
                )

            timestamp = int(time.time())
            meas.update({"timestamp": timestamp})
            with thlock:
                if self.client_id not in sessions:
                    public = _generate_pubkey()
                else:
                    public = sessions[self.client_id].public

            measurement = Measurement(public, model, geo_lat, geo_lon, meas)

        except Exception as e:
            logging.warning(f"MQTT Station: Error in parser {e}")
            return

        return measurement

    def on_message(self, client, userdata, msg: dict) -> None:
        global thlock
        global sessions
        data = json.loads(msg.payload.decode())
        parse_data = self._parser(data)

        with thlock:
            if parse_data:
                sessions[self.client_id] = parse_data

    def on_subscribe(self, client, userdata, mid, granted_qos) -> None:
        logger.info(f"Subscribed {str(mid)}, client {client}")

    def run(self) -> None:
        self.connect_async(self.host, self.port, 60)
        self.loop_start()


class MQTTStation(IStation):
    def __init__(self, config: dict) -> None:
        super().__init__(config)
        self.version: str = f"airalab-http-{STATION_VERSION}"
        self.DEAD_SENSOR_TIME: int = 60 * 60  # 1 hour
        host: str = config["mqttstation"]["host"]
        port: int = int(config["mqttstation"]["port"])
        client = MQTTHandler(host, port)
        rc = client.run()

    def get_data(self) -> tp.List[StationData]:
        global sessions
        result = []
        for k, v in self._drop_dead_sensors().items():
            result.append(
                StationData(
                    self.version, self.mac_address, time.time() - self.start_time, v
                )
            )
        return result

    def _drop_dead_sensors(self) -> dict:
        global thlock
        global sessions
        stripped = dict()
        current_time = int(time.time())
        with thlock:
            sessions_copy = copy.deepcopy(sessions)
            for k, v in sessions_copy.items():
                if (current_time - v.measurement["timestamp"]) < self.DEAD_SENSOR_TIME:
                    stripped[k] = v
                else:
                    del sessions[k]

        return stripped
