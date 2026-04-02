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

from connectivity.constants import STATION_VERSION
from connectivity.src.sensors import SensorsFabcric
from .istation import IStation


logging.config.dictConfig(LOGGING_CONFIG)
logger = logging.getLogger("sensors-connectivity")
thlock = threading.RLock()
sessions = dict()
last_sensors_update = time.time()

ALIVE_SENSORS_METRIC = Gauge("connectivity_sensors_alive_total", "return number of active sessions")


class RequestHandler(BaseHTTPRequestHandler):
    """Service class to handle HTTP requests."""

    known_peers = []
    server_port = 8001

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

        id = self.headers.get("Sensor-id")
        self._set_headers(id)
        logger.info(f"HTTP Station session length: {len(sessions)}")

        own_count = len(sessions)
        servers = []
        for peer in self.known_peers:
            is_self = peer.get("port") == self.server_port
            servers.append({
                "host": peer["host"],
                "port": peer["port"],
                "sensors_count": own_count if is_self else 0,
                "region": peer.get("region", "global"),
            })

        body = json.dumps({"servers": servers, "ttl": 3600, "version": 1})
        self.wfile.write(body.encode())

    def do_POST(self) -> None:
        """Callback for post requests. Save unparsed data into `sessions`."""

        global sessions
        ctype, pdict = cgi.parse_header(self.headers["content-type"])
        length = int(self.headers.get("content-length"))
        d = self.rfile.read(length).decode().replace("SDS_P1", "SDS_pm10").replace("SDS_P2", "SDS_pm25")
        data = json.loads(d)
        meas = SensorsFabcric.get_sensor(data)
        if meas is None: 
            self.send_response(500)
            self.end_headers()
            return
        with thlock:
            if meas:
                sessions[meas.id] = meas
        self._set_headers()


class HTTP_server(threading.Thread):
    """Servcie class to run HTTP server."""

    def __init__(self, port: int, known_peers: list = None):
        threading.Thread.__init__(self)
        self.port: int = port
        self.known_peers: list = known_peers or []

    def run(self) -> None:
        self.server_address = ("", self.port)
        logger.debug(f"HTTP Station on port: {self.port}")
        RequestHandler.known_peers = self.known_peers
        RequestHandler.server_port = self.port
        self.httpd = HTTPServer(self.server_address, RequestHandler)
        self.httpd.serve_forever()


class HTTPStation(IStation):
    """Station to input data via HTTP."""

    def __init__(self, config: dict) -> None:
        """Start HTTP server on the port from config.

        :param config: Dict with configuration file.
        """

        port: int = int(config["httpstation"]["port"])
        known_peers: list = config.get("known_peers", [])
        self.version = STATION_VERSION
        HTTP_server(port, known_peers).start()
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
