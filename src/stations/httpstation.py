# -*- coding: utf-8 -*-
import asyncio
from asyncio import StreamReader, StreamWriter
import threading
import time
import rospy
import time
from drivers.sds011 import SDS011_MODEL
import json
import cgi

from stations import IStation, StationData, Measurement, STATION_VERSION
from collections import deque
from http.server import BaseHTTPRequestHandler, HTTPServer

q = deque(maxlen=1)

class RequestHandler(BaseHTTPRequestHandler):

    def _set_headers(self):
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.end_headers()

    def do_HEAD(self):
        self._set_headers()

    def do_GET(self):
        self._set_headers()

    def parser(self, data):
        try:
            for dict in data['sensordatavalues']:
                if dict['value_type'] == 'SDS_P1':
                    pm10 = dict['value']
                if dict['value_type'] == 'SDS_P2':
                    pm25 = dict['value']
                if dict['value_type'] == 'GPS_lat':
                    geo_lat = dict['value']
                if dict['value_type'] == 'GPS_lon':
                    geo_lon = dict['value']
            public = '655a40d4b951b4fbc0c9a7658e66377f1a5ff92111f10145256e0026ab07a669'
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
        global q
        rospy.loginfo("data is coming!")
        ctype, pdict = cgi.parse_header(self.headers['content-type'])
        length = int(self.headers.get('content-length'))
        self.data = json.loads(self.rfile.read(length))
        rospy.logdebug(self.data)
        meas = self.parser(self.data)
        if meas:
            q.append(meas)
        rospy.loginfo(q[0])
        self._set_headers()




class HTTP_server(threading.Thread):
    def __init__(self, port: int):
        threading.Thread.__init__(self)
        self.port = port


    def run(self):
        rospy.loginfo('run func')
        self.server_address = ('', self.port)
        self.httpd = HTTPServer(self.server_address, RequestHandler)
        rospy.loginfo('Starting httpd')
        self.httpd.serve_forever()


class HTTPStation(IStation):

    def __init__(self, config: dict):
        super().__init__(config)
        port = int(config["httpstation"]["port"])
        HTTP_server(port).start()
        self.version = f"airalab-com-{STATION_VERSION}"

    def get_data(self):
        global q
        rospy.loginfo(q)
        if q:
            value = q[0]
        else:
            value = Measurement()
        return [StationData(
            self.version,
            self.mac_address,
            time.time() - self.start_time,
            value
        )]
