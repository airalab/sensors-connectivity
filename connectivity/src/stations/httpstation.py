# -*- coding: utf-8 -*-
import threading
import time
import json
import cgi
import copy
import hashlib
import typing as tp
import logging.config
from http.server import BaseHTTPRequestHandler, HTTPServer
from prometheus_client import Gauge
from functools import reduce

from ..drivers.sds011 import SDS011_MODEL, MOBILE_GPS
from .istation import IStation, StationData, Measurement, STATION_VERSION
from connectivity.config.logging import LOGGING_CONFIG

logging.config.dictConfig(LOGGING_CONFIG)
logger = logging.getLogger("sensors-connectivity")
thlock = threading.RLock()
sessions = dict()
last_sensors_update = time.time()

ALIVE_SENSORS_METRIC = Gauge(
    "connectivity_sensors_alive_total", "return number of active sessions"
)


def _generate_pubkey(id: str) -> str:
    verify_key = hashlib.sha256(id.encode("utf-8"))
    verify_key_hex = verify_key.hexdigest()
    return str(verify_key_hex)


class RequestHandler(BaseHTTPRequestHandler):
    def _set_headers(self, id: str = None) -> None:
        global last_sensors_update
        self.send_response(200)
        self.send_header("Content-type", "application/json")
        updating_sensors_interval = 60 * 60  # 1hr, how often sensors will switch server
        if (time.time() - last_sensors_update) > updating_sensors_interval:
            self.send_header("sensors-count", "0")
            last_sensors_update = time.time()
        else:
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

    def _SDS011_values_saver(self, meas: dict, value: dict) -> dict:
        "Reducer callback for SDS011 sensors"
        extra_data = [
            "GPS",
            "micro",
            "signal",
            "samples",
            "interval",
        ]  # values which shouldn't be stored in meas dict
        paskal = 133.32
        if any(x in value["value_type"] for x in extra_data):
            return meas
        if "_" in value["value_type"] and not "CCS" in value["value_type"]:
            if "pressure" in value["value_type"]:
                meas[value["value_type"].split("_")[1]] = float(value["value"]) / paskal
            else:
                meas[value["value_type"].split("_")[1]] = value["value"]
        else:
            meas[value["value_type"]] = value["value"]
        return meas

    def _mobile_sensor_data_saver(self, meas: dict, value: str) -> dict:
        "Reducer callback for mobile GPS sensor's data"
        key, item = value
        paskal = 133.32
        if "GPS" in key or "ID" in key:
            return meas
        if "pressure" in key:
            meas[key] = float(item) / paskal
        meas[key] = item
        return meas

    def _parser(self, data: dict) -> Measurement:
        global sessions
        global thlock
        paskal = 133.32
        try:
            if "esp8266id" in data.keys():
                self.client_id = int(data["esp8266id"])
                for d in data["sensordatavalues"]:
                    if d["value_type"] == "GPS_lat":
                        geo_lat = d["value"]
                    if d["value_type"] == "GPS_lon":
                        geo_lon = d["value"]

                model = SDS011_MODEL
                meas = reduce(self._SDS011_values_saver, data["sensordatavalues"], {})

            elif "ID" in data.keys():
                self.client_id = data["ID"]
                geo_lat = float(data.get("GPS_lat"))
                geo_lon = float(data.get("GPS_lon"))
                meas = reduce(self._mobile_sensor_data_saver, data.items(), {})
                model = MOBILE_GPS

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
        d = self.rfile.read(length).decode().replace("SDS_P1", "SDS_pm10").replace("SDS_P2", "SDS_pm25")
        self.data = json.loads(d)
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
        ALIVE_SENSORS_METRIC.set(len(stripped))
        return stripped
