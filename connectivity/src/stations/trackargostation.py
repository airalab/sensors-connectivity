#!/usr/bin/env python3
"""Station to get data from TrackAgro sensor using its API."""
import json
import logging.config
import ssl
import threading
import time
import typing as tp
import urllib.request as ur
# from os import times
from urllib import error, parse

from connectivity.config.logging import LOGGING_CONFIG

from ...constants import STATION_VERSION
from ..sensors import TrackAgro
from .istation import IStation

logging.config.dictConfig(LOGGING_CONFIG)
logger = logging.getLogger("sensors-connectivity")


thlock = threading.RLock()


class TrackAgroStation(IStation):
    def __init__(self, config: dict) -> None:
        """Prepare headers. COllects initial data."""

        self.headers = {"X-Token-Auth": config["trackagro"]["token"]}
        ssl._create_default_https_context = ssl._create_unverified_context
        self.sessions: dict = dict()
        self.version: str = f"airalab-http-{STATION_VERSION}"
        self.DEAD_SENSOR_TIME: int = 60 * 60  # 1 hour
        self.time_from: int = "1645169200000"
        self._collecting_data()

    def url_updater(self, till_time: str, from_time: str) -> str:
        """Update url to get data for a period of time.

        :param till_time: End of the period.
        :param from_time: Start of the period.
        :return: Updated url for request.
        """

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
        """Send request.

        :return: Dict with response.
        """

        url = self.url_updater(till_time=f"{int(time.time()) * 1000}", from_time=f"{self.time_from}")
        try:
            request = ur.Request(url, headers=self.headers)
            response_body = ur.urlopen(request).read()
        except (error.URLError, error.HTTPError) as e:
            logger.warning(f"TrackAgro: error while sending request {e}")
            return
        data = json.loads(response_body)
        return data

    def _collecting_data(self) -> None:
        """Parse data and store it into `sessions`."""

        threading.Timer(3600, self._collecting_data).start()
        data = self._request_sendler()
        meas = TrackAgro(data, self.time_from)
        with thlock:
            if meas:
                self.sessions[meas.id] = meas
                self.time_from = meas.time_from

    def get_data(self) -> tp.List[dict]:
        """Main function of the class.

        :return: Formatetd data.
        """

        global sessions
        global thlock
        with thlock:
            stripped = self.drop_dead_sensors(self.sessions)
        result = []
        for k, v in stripped.items():
            result.append(v)
        return result
