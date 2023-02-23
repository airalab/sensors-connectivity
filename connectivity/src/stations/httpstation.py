# -*- coding: utf-8 -*-
"""Station to input data via HTTP."""
import cgi
import json
import logging.config
import threading
import time
import typing as tp
from http.server import BaseHTTPRequestHandler, HTTPServer

from prometheus_client import Gauge

from connectivity.config.logging import LOGGING_CONFIG

from ...constants import STATION_VERSION
from ..sensors import EnvironmentalBox, MobileLab
from .istation import IStation

logging.config.dictConfig(LOGGING_CONFIG)
logger = logging.getLogger("sensors-connectivity")
thlock = threading.RLock()
sessions = dict()
last_sensors_update = time.time()

ALIVE_SENSORS_METRIC = Gauge("connectivity_sensors_alive_total", "return number of active sessions")


class RequestHandler(BaseHTTPRequestHandler):
    """Service class to handle HTTP requests."""

    def _set_headers(self, id: str = None) -> None:
        """Service functions to set headers for requests.
        It checks if the sensor has sent data to this station already and
        sends the count of the active sensors back to the sensor. It allows to
        distribute sensors evenly across multiple servers.

        :param id: Unique id of the sensor. Used to check wether the sensor is on the station or no.
        """

        global last_sensors_update
        global sessions
        self.send_response(200)
        self.send_header("Content-type", "application/json")
        updating_sensors_interval = 60 * 60  # 1hr, how often sensors will switch server
        if (time.time() - last_sensors_update) > updating_sensors_interval:
            self.send_header("sensors-count", "0")
            last_sensors_update = time.time()
            logger.debug(f"sessions before rotation: {sessions}")
            with thlock:
                sessions.clear()
            logger.debug(f"sessions after rotation: {sessions}")
        else:
            self.send_header("sensors-count", f"{len(sessions)}")
        if id is not None:
            self.send_header("on-server", f"{str(id) in sessions}")
        self.end_headers()

    def do_HEAD(self) -> None:
        """Service function to set headers."""
        self._set_headers()

    def do_GET(self) -> None:
        """Callback for get requests."""

        logger.info("GET request,\nPath: %s\nHeaders:\n%s\n", str(self.path), str(self.headers))
        id = self.headers["Sensor-id"]
        self._set_headers(id)
        logger.info(f"HTTP Station session length: {len(sessions)}")

    def do_POST(self) -> None:
        """Callback for post requests. Save unparsed data into `sessions`."""

        global sessions
        ctype, pdict = cgi.parse_header(self.headers["content-type"])
        length = int(self.headers.get("content-length"))
        d = self.rfile.read(length).decode().replace("SDS_P1", "SDS_pm10").replace("SDS_P2", "SDS_pm25")
        data = json.loads(d)
        if "esp8266id" in data.keys():
            meas = EnvironmentalBox(data)
        elif "ID" in data.keys():
            meas = MobileLab(data)
        with thlock:
            if meas:
                sessions[meas.id] = meas
        self._set_headers()


class HTTP_server(threading.Thread):
    """Servcie class to run HTTP server."""

    def __init__(self, port: int):
        threading.Thread.__init__(self)
        self.port: int = port

    def run(self) -> None:
        self.server_address = ("", self.port)
        logger.debug(f"HTTP Station on port: {self.port}")
        self.httpd = HTTPServer(self.server_address, RequestHandler)
        self.httpd.serve_forever()


class HTTPStation(IStation):
    """Station to input data via HTTP."""

    def __init__(self, config: dict) -> None:
        """Start HTTP server on the port from config.

        :param config: Dict with configuration file.
        """

        port: int = int(config["httpstation"]["port"])
        self.version = STATION_VERSION
        HTTP_server(port).start()
        self.DEAD_SENSOR_TIME: int = 60 * 60  # 1 hour

    def get_data(self) -> tp.List[dict]:
        """Main function of the class.

        :return: Formatetd data.
        """

        global sessions
        global thlock
        result = []
        with thlock:
            stripped = self.drop_dead_sensors(sessions)
            ALIVE_SENSORS_METRIC.set(len(stripped))
        for k, v in stripped.items():
            result.append(v)
        return result
