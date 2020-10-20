# -*- coding: utf-8 -*-
import threading
import time
import rospy
from drivers.sds011 import SDS011_MODEL
import json
import cgi
import nacl.signing

from stations import IStation, StationData, Measurement, STATION_VERSION
#from collections import deque
from http.server import BaseHTTPRequestHandler, HTTPServer

#q = deque()

thlock = threading.RLock()
sessions = dict()

class RequestHandler(BaseHTTPRequestHandler):

    def _set_headers(self):
        self.send_response(200)
        self.send_header("Content-type", "application/json")
        self.end_headers()

    def do_HEAD(self):
        self._set_headers()

    def do_GET(self):
        self._set_headers()

    def _generate_pubkey(self) -> str:
        signing_key = nacl.signing.SigningKey.generate()
        verify_key = signing_key.verify_key
        verify_key_hex = bytes(verify_key).hex()
        return str(verify_key_hex)

    def parser(self, data) -> Measurement:
        global sessions
        global thlock
        #rospy.loginfo(f"parser data: {data}")
        try:
            if data["esp8266id"]:
                self.client_id = data["esp8266id"]
            for d in data["sensordatavalues"]:
                if d["value_type"] == "SDS_P1":
                    pm10 = float(d["value"])
                if d["value_type"] == "SDS_P2":
                    pm25 = float(d["value"])
                if d["value_type"] == "GPS_lat":
                    geo_lat = d["value"]
                if d["value_type"] == "GPS_lon":
                    geo_lon = d["value"]

            #rospy.loginfo("After 'for'")

            with thlock:
                if self.client_id not in sessions:
                    #rospy.loginfo(f"There is no such address: {self.client_address}")
                    public = self._generate_pubkey()
                else:
                    #rospy.loginfo(f"Found such address: {self.client_address}")
                    public = sessions[self.client_id].public

            #rospy.loginfo(f"Pubkey: {public}")
            timestamp = int(time.time())
            measurement = Measurement(public,
                                    SDS011_MODEL,
                                    pm25,
                                    pm10,
                                    geo_lat,
                                    geo_lon,
                                    timestamp)
        except Exception as e:
            rospy.logerr(e)
            return

        return measurement

    def do_POST(self):
        global sessions
        rospy.loginfo("data is coming!")
        ctype, pdict = cgi.parse_header(self.headers["content-type"])
        length = int(self.headers.get("content-length"))
        self.data = json.loads(self.rfile.read(length))
        rospy.loginfo(self.data)
        meas = self.parser(self.data)
        rospy.loginfo(meas)
        with thlock:
            if meas:
                if self.client_id in sessions:
                    del sessions[self.client_id]
                sessions[self.client_id] = meas
        self._set_headers()


class HTTP_server(threading.Thread):
    def __init__(self, port: int):
        threading.Thread.__init__(self)
        self.port = port

    def run(self):
        rospy.loginfo("run func")
        self.server_address = ("", self.port)
        self.httpd = HTTPServer(self.server_address, RequestHandler)
        rospy.loginfo("Starting httpd")
        self.httpd.serve_forever()


class HTTPStation(IStation):

    def __init__(self, config: dict):
        super().__init__(config)
        port = int(config["httpstation"]["port"])
        HTTP_server(port).start()
        self.version = f"airalab-http-{STATION_VERSION}"
        self.DEAD_SENSOR_TIME = 60*60 # 1 hour

    def get_data(self):
        global sessions
        rospy.loginfo(sessions)

        result = []
        for k, v in self._drop_dead_sensors().items():
            result.append(StationData(
                self.version,
                self.mac_address,
                time.time() - self.start_time,
                v
            ))

        return result

    def _drop_dead_sensors(self) -> dict:
        global sessions
        global thlock
        #rospy.loginfo(f"_drop_dead_sensors: {sessions}")
        stripped = dict()
        current_time = int(time.time())
        with thlock:
            for k, v in sessions.items():
                if (current_time - v.timestamp) < self.DEAD_SENSOR_TIME:
                    stripped[k] = v

        return stripped

