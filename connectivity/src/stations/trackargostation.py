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

from .istation import IStation, STATION_VERSION
from ..sensors import TrackAgro
from connectivity.config.logging import LOGGING_CONFIG

logging.config.dictConfig(LOGGING_CONFIG)
logger = logging.getLogger("sensors-connectivity")


thlock = threading.RLock()


class TrackAgroStation(IStation):
    def __init__(self, config: dict) -> None:
        # super().__init__(config)
        self.headers = {"X-Token-Auth": config["trackagro"]["token"]}
        ssl._create_default_https_context = ssl._create_unverified_context
        self.sessions: dict = dict()
        self.version: str = f"airalab-http-{STATION_VERSION}"
        self.DEAD_SENSOR_TIME: int = 60 * 60  # 1 hour
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

    def _collecting_data(self) -> None:
        threading.Timer(3600, self._collecting_data).start()
        data = self._request_sendler()
        meas = TrackAgro(data, self.time_from)
        with thlock:
            if meas:
                self.sessions[meas.id] = meas
                self.time_from = meas.time_from

    def get_data(self) -> tp.List[dict]:
        global sessions
        result = []
        for k, v in self._drop_dead_sensors().items():
            result.append(v)
        return result

    def _drop_dead_sensors(self) -> dict:
        global thlock
        stripped = dict()
        current_time = int(time.time())
        with thlock:
            sessions_copy = copy.deepcopy(self.sessions)
            for k, v in sessions_copy.items():
                if (current_time - v.timestamp) < self.DEAD_SENSOR_TIME:
                    stripped[k] = v
                else:
                    del self.sessions[k]
        return stripped
