# -*- coding: utf-8 -*-
import threading
import time
import json
import cgi
import copy
import hashlib
import typing as tp
import logging.config

from ..drivers.sds011 import SDS011_MODEL, MOBILE_GPS
from .istation import IStation, StationData, Measurement, STATION_VERSION
from http.server import BaseHTTPRequestHandler, HTTPServer
from connectivity.config.logging import LOGGING_CONFIG

logging.config.dictConfig(LOGGING_CONFIG)
logger = logging.getLogger("sensors-connectivity")
thlock = threading.RLock()
sessions = dict()

def _generate_pubkey(id: str) -> str:
    verify_key = hashlib.sha256(id.encode("utf-8"))
    verify_key_hex = verify_key.hexdigest()
    return str(verify_key_hex)


class RequestHandler(BaseHTTPRequestHandler):
    def _set_headers(self, id: str = None) -> None:
        self.send_response(200)
        self.send_header("Content-type", "application/json")
        self.send_header("sensors-count", f"{len(sessions)}")
        if id is not None:
            self.send_header("on-server", f"{int(id) in sessions}")
        self.end_headers()

    def do_HEAD(self) -> None:
        self._set_headers()

    def do_GET(self) -> None:
        logger.info(
            "GET request,\nPath: %s\nHeaders:\n%s\n", str(self.path), str(self.headers)
        )
        id = self.headers["Sensor-id"]
        self._set_headers(id)
        logger.info(f"HTTP Station session length: {len(sessions)}")

    def _parser(self, data: dict) -> Measurement:
        global sessions
        global thlock
        paskal = 133.32
        try:
            if "esp8266id" in data.keys():
                self.client_id = int(data["esp8266id"])
                temperature = None
                pressure = None
                humidity = None
                CCS_CO2 = None
                CCS_TVOC = None

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
                    if "CCS_CO2" in d["value_type"]:
                        CCS_CO2 = float(d["value"])
                    if "CCS_TVOC" in d["value_type"]:
                        CCS_TVOC = float(d["value"])

                meas = {}
                model = SDS011_MODEL
                meas.update(
                    {
                        "pm10": pm10,
                        "pm25": pm25,
                        "temperature": temperature,
                        "pressure": pressure,
                        "humidity": humidity,
                        "CCS_CO2": CCS_CO2,
                        "CCS_TVOC": CCS_TVOC,
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

            with thlock:
                if self.client_id not in sessions:
                    public = _generate_pubkey(str(self.client_id))
                else:
                    public = sessions[self.client_id].public

            timestamp = int(time.time())
            meas.update({"timestamp": timestamp})
            measurement = Measurement(public, model, geo_lat, geo_lon, meas)

        except Exception as e:
            logger.warning(f"HTTP Station: Error in parser {e}")
            return
        return measurement

    def do_POST(self) -> None:
        global sessions
        ctype, pdict = cgi.parse_header(self.headers["content-type"])
        length = int(self.headers.get("content-length"))
        self.data = json.loads(self.rfile.read(length))
        meas = self._parser(self.data)
        with thlock:
            if meas:
                sessions[self.client_id] = meas
        self._set_headers()


class HTTP_server(threading.Thread):
    def __init__(self, port: int):
        threading.Thread.__init__(self)
        self.port: int = port

    def run(self) -> None:
        self.server_address = ("", self.port)
        self.httpd = HTTPServer(self.server_address, RequestHandler)
        self.httpd.serve_forever()


class HTTPStation(IStation):
    def __init__(self, config: dict) -> None:
        super().__init__(config)
        port: int = int(config["httpstation"]["port"])
        HTTP_server(port).start()
        self.version: str = f"airalab-http-{STATION_VERSION}"
        self.DEAD_SENSOR_TIME: int = 60 * 60  # 1 hour

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
        global sessions
        global thlock
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
