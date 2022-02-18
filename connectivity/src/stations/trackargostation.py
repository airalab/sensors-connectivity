#!/usr/bin/env python3
from os import times
import urllib.request as ur
from urllib import parse, error
import ssl
import json
import time
import hashlib
import threading
import logging.config
import copy
import typing as tp

from .istation import IStation, StationData, Measurement, STATION_VERSION
from ..drivers.sds011 import SDS011_MODEL
from connectivity.config.logging import LOGGING_CONFIG

logging.config.dictConfig(LOGGING_CONFIG)
logger = logging.getLogger("sensors-connectivity")


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
        self.sessions: tp.Dict[str, Measurement] = dict()
        self.version: str = f"airalab-http-{STATION_VERSION}"
        self.DEAD_SENSOR_TIME: int = 60 * 60  # 1 hour
        self.client_id: str = None
        self.time_from: int = "1645169200000"
        self._collecting_data()

    def url_updater(self, till_time: str, from_time: str) -> str:
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

    def _request_sendler(self) -> tp.List[tp.Dict[str, tp.Union[str, int, float]]]:
        url = self.url_updater(
            till_time=f"{int(time.time()) * 1000}", from_time=f"{self.time_from}"
        )
        try:
            request = ur.Request(url, headers=self.headers)
            response_body = ur.urlopen(request).read()
        except (error.URLError, error.HTTPError) as e:
            logger.warning(f"TrackAgro: error while sending request {e}")
            return
        data = json.loads(response_body)
        return data

    def _parser(
        self, data: tp.List[tp.Dict[str, tp.Union[str, int, float]]]
    ) -> Measurement:
        meas = {}
        timestamp = 0
        try:
            for d in data:
                self.client_id = d["id"]
                if "position" in d["key"]:
                    if d["key"] == "position.longitude":
                        geo_lon = float(d["value"])
                    else:
                        geo_lat = float(d["value"])
                else:
                    if d["ts"] > timestamp:
                        timestamp = d["ts"]
                    if any(d["key"] == key for key in meas.keys()):
                        if d["ts"] > meas[d["key"]]["timestamp"]:
                            meas[d["key"]].update(
                                {"value": d["value"], "timestamp": d["ts"]}
                            )
                    else:
                        meas.update(
                            {d["key"]: {"value": d["value"], "timestamp": d["ts"]}}
                        )
            parsed_meas = {}
            for k, v in meas.items():
                parsed_meas.update({k: v["value"]})
            parsed_meas.update({"pm10": "", "pm25": ""})
            model = SDS011_MODEL
            with thlock:
                if self.client_id not in self.sessions:
                    public = _generate_pubkey(str(self.client_id))
                else:
                    public = self.sessions[self.client_id].public
            parsed_meas.update({"timestamp": timestamp / 1000})
            self.time_from = timestamp
            measurement = Measurement(public, model, geo_lat, geo_lon, parsed_meas)
        except (UnboundLocalError, TypeError):
            pass
            return
        except Exception as e:
            logger.warning(f"TracAgro: error in parser {e}")
            return
        return measurement

    def _collecting_data(self) -> None:
        threading.Timer(3600, self._collecting_data).start()
        data = self._request_sendler()
        meas = self._parser(data)
        with thlock:
            if meas:
                self.sessions[self.client_id] = meas

    def get_data(self) -> tp.List[StationData]:
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
