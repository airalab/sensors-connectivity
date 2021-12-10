#!/usr/bin/env python3
import urllib.request as ur
from urllib import parse
import ssl
import json
import time
import hashlib
import threading
import rospy
import copy

from stations import IStation, StationData, Measurement, STATION_VERSION
from drivers.sds011 import SDS011_MODEL


def _generate_pubkey(id) -> str:
    verify_key = hashlib.sha256(id.encode("utf-8"))
    verify_key_hex = verify_key.hexdigest()
    return str(verify_key_hex)


thlock = threading.RLock()


class TrackAgroStation(IStation):
    def __init__(self, config: dict) -> None:
        super().__init__(config)
        self.headers = {"X-Token-Auth": config["trackagro"]["token"]}
        ssl._create_default_https_context = ssl._create_unverified_context
        self.sessions = dict()
        self.version = f"airalab-http-{STATION_VERSION}"
        self.DEAD_SENSOR_TIME = 60 * 60  # 1 hour
        self.client_id = None
        self._collecting_data()

    def url_updater(self, till_time: float, from_time: float) -> str:
        url = "https://api.ttrackagro.ru/telemetry"
        url_parse = parse.urlparse(url)
        query = url_parse.query
        param = {"from": from_time, "to": till_time}
        dic = dict(parse.parse_qsl(query))
        dic.update(param)
        upd_query = parse.urlencode(dic)
        url_parse = url_parse._replace(query=upd_query)
        upd_url = parse.urlunparse(url_parse)
        return upd_url

    def request_sendler(self) -> object:
        url = self.url_updater("1637746203000", "1637744400000")
        request = ur.Request(url, headers=self.headers)
        response_body = ur.urlopen(request).read()
        data = json.loads(response_body)
        return data

    def _parser(self, data) -> Measurement:
        meas = {}
        try:
            for d in data:
                self.client_id = d["id"]
                if "position" in d["key"]:
                    if d["key"] == "position.longitude":
                        geo_lon = d["value"]
                    else:
                        geo_lat = d["value"]
                else:
                    if any(d["key"] in keys for keys in meas.keys()):
                        if d["ts"] > meas[d["key"]]["timestamp"]:
                            meas[d["key"]].update(
                                {"value": d["value"], "timestamp": d["ts"]}
                            )
                    else:
                        meas.update(
                            {d["key"]: {"value": d["value"], "timestamp": d["ts"]}}
                        )
            timestamp = int(time.time())
            model = SDS011_MODEL
            meas.update({"timestamp": timestamp})
            with thlock:
                if self.client_id not in self.sessions:
                    public = _generate_pubkey(str(self.client_id))
                else:
                    public = self.sessions[self.client_id].public
            measurement = Measurement(public, model, geo_lat, geo_lon, meas)

        except Exception as e:
            rospy.logerr(f"TracArgo: error in parser {e}")
            return
        return measurement

    def _collecting_data(self):
        threading.Timer(300, self.request_sendler).start()
        data = self.request_sendler()
        meas = self._parser(data)
        print(f"parsed data: {meas}")
        with thlock:
            if meas:
                self.sessions[self.client_id] = meas
        print(f"sessions: {self.sessions}")

    def get_data(self) -> StationData:
        result = []
        for k, v in self._drop_dead_sensors().items():
            result.append(
                StationData(
                    self.version, self.mac_address, time.time() - self.start_time, v
                )
            )
        print(f"result: {result}")
        return result

    def _drop_dead_sensors(self) -> dict:
        global thlock
        stripped = dict()
        current_time = int(time.time())
        with thlock:
            sessions_copy = copy.deepcopy(self.sessions)
            for k, v in sessions_copy.items():
                if (current_time - v.measurement["timestamp"]) < self.DEAD_SENSOR_TIME:
                    stripped[k] = v
                else:
                    del self.sessions[k]

        return stripped
