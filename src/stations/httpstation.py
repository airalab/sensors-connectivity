# -*- coding: utf-8 -*-
import threading
import time
import rospy
from drivers.sds011 import SDS011_MODEL, MOBILE_GPS
import json
import cgi
#import nacl.signing
import copy
import hashlib


from stations import IStation, StationData, Measurement, STATION_VERSION
from http.server import BaseHTTPRequestHandler, HTTPServer


thlock = threading.RLock()
sessions = dict()


def _generate_pubkey(id) -> str:
    verify_key = hashlib.sha256(id.encode('utf-8'))
    verify_key_hex = verify_key.hexdigest()
    return str(verify_key_hex)



class RequestHandler(BaseHTTPRequestHandler):

    def _set_headers(self):
        self.send_response(200)
        self.send_header("Content-type", "application/json")
        self.end_headers()

    def do_HEAD(self):
        self._set_headers()

    def do_GET(self):
        self._set_headers()


    def _parser(self, data: dict) -> Measurement:
        global sessions
        global thlock

        #rospy.loginfo(f"parser data: {data}")
        try:
            if 'esp8266id' in data.keys():
                self.client_id = int(data["esp8266id"])
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
                        pressure = float(d["value"])
                    if "humidity" in d["value_type"]:
                        humidity = float(d["value"])

                meas = {}
                meas.update({'pm10': pm10, 'pm25': pm25, 'temperature': temperature, 'pressure': pressure, 'humidity': humidity})
            elif 'ID' in data.keys():
                self.client_id = data["ID"]
                temperature = float(data["temperature"])
                humidity = float(data["humidity"])
                pressure = float(data["pressure"])
                CO = float(data["CO"])
                NH3 = float(data["NH3"])
                NO2 = float(data["NO2"])
                geo_lat = float(data["GPS_lat"])
                geo_lon = float(data["GPS_lon"])

                meas = {}
                meas.update({"temperature": temperature, "humidity": humidity, "pressure": pressure, "CO": CO, "NH3": NH3, "NO2": NO2})

            with thlock:
                if self.client_id not in sessions:
                    #rospy.loginfo(f"There is no such address: {self.client_address}")
                    public = _generate_pubkey(str(self.client_id))
                else:
                    #rospy.loginfo(f"Found such address: {self.client_address}")
                    public = sessions[self.client_id].public

            #rospy.loginfo(f"Pubkey: {public}")
            timestamp = int(time.time())
            #rospy.loginfo(f"time: {timestamp}")
            meas.update({'timestamp': timestamp})
            measurement = Measurement(public,
                                     SDS011_MODEL,
                                     geo_lat,
                                     geo_lon,
                                     meas)

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
        meas = self._parser(self.data)
        with thlock:
            if meas:
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

    def get_data(self) -> StationData:
        global sessions
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
            sessions_copy = copy.deepcopy(sessions)
            for k, v in sessions_copy.items():
                if (current_time - v.measurement["timestamp"]) < self.DEAD_SENSOR_TIME:
                    stripped[k] = v
                else:
                    del sessions[k]

        return stripped

