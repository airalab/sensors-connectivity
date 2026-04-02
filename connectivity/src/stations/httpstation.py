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
    """Service class to handle HTTP requests.

    Handles GET, POST, and HEAD requests from sensors and discovery clients.
    GET returns a peer discovery response (list of known servers with sensor counts).
    POST accepts sensor measurement data.
    HEAD returns headers with sensor count info for legacy load-balancing.
    """

    # Class-level attributes shared across all request handler instances.
    # These are set by HTTP_server.run() before the server starts accepting requests.
    # We use class variables (not instance variables) because HTTPServer creates
    # a new RequestHandler instance for each incoming request — there is no way
    # to pass constructor arguments through HTTPServer.
    known_peers = []      # List of peer dicts from config["known_peers"]
    server_port = 8001    # Port this server is listening on, used to identify "self" in the peer list

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
        """Handle GET requests — serve the peer discovery response.

        This is the core of the peer discovery feature. When a sensor (or any
        client) sends a GET request, the server responds with a JSON document
        listing all known connectivity servers, how many sensors each one has,
        and a TTL for caching.

        The sensor firmware uses this response to pick the least-loaded server,
        distributing devices across the fleet without a central coordinator.

        Optional header:
            Sensor-id — if present, the response headers will include whether
            this sensor is already registered on this server (via _set_headers).

        Response body (JSON):
            {
                "servers": [{"host", "port", "sensors_count", "region"}, ...],
                "ttl": <cache lifetime in seconds>,
                "version": <protocol version>
            }
        """

        # Use .get() instead of [] so that GET requests without the header
        # (e.g. from browsers or monitoring) don't crash with KeyError.
        id = self.headers.get("Sensor-id")
        self._set_headers(id)
        logger.info(f"HTTP Station session length: {len(sessions)}")

        # Build the server list for the discovery response.
        # We only know our own sensor count; for remote peers we report 0.
        # In a future iteration, peers could exchange counts, but for now
        # the sensor firmware will query each peer directly when it needs
        # an accurate count — this list just tells it WHERE to look.
        own_count = len(sessions)
        servers = []
        for peer in self.known_peers:
            # Identify whether this peer entry refers to ourselves by comparing
            # the configured port. This is a simple heuristic: in production
            # each node runs on a unique port, so port equality means "this is me".
            is_self = peer.get("port") == self.server_port
            servers.append({
                "host": peer["host"],
                "port": peer["port"],
                # Only populate sensors_count for our own entry — we don't
                # know how many sensors remote peers have without asking them.
                "sensors_count": own_count if is_self else 0,
                "region": peer.get("region", "global"),
            })

        # Wrap the server list in a versioned envelope.
        # ttl=3600 tells clients to cache this response for 1 hour before
        # re-fetching, reducing unnecessary discovery traffic.
        # version=1 allows future protocol changes without breaking old clients.
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
    """Threaded HTTP server wrapper for the connectivity station.

    Runs the HTTP server in a daemon-like background thread so that the
    main application thread remains free for other station types (COM, MQTT)
    and the data publishing loop.

    :param port: TCP port to listen on.
    :param known_peers: List of peer server dicts from the config. Each dict
        must contain "host" and "port", and optionally "region". Passed through
        to RequestHandler for the discovery endpoint.
    """

    def __init__(self, port: int, known_peers: list = None):
        threading.Thread.__init__(self)
        self.port: int = port
        # Default to empty list if no peers configured — the discovery
        # endpoint will simply return an empty server list.
        self.known_peers: list = known_peers or []

    def run(self) -> None:
        """Start the HTTP server and block forever (runs in its own thread)."""
        self.server_address = ("", self.port)
        logger.debug(f"HTTP Station on port: {self.port}")
        # Inject configuration into the RequestHandler class before the server
        # starts. HTTPServer instantiates RequestHandler per-request and only
        # accepts the class (not an instance), so we must use class-level
        # attributes to pass configuration to the handler.
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
        # Load the peer discovery list from config. This is a top-level key
        # (not nested under "httpstation") because peer discovery is a
        # cross-cutting concern that may be used by other station types later.
        known_peers: list = config.get("known_peers", [])
        self.version = STATION_VERSION
        # Start the HTTP server thread with the peer list so that GET /
        # can serve the discovery response immediately.
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
