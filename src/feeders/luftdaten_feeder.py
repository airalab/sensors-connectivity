import requests
import typing as tp
import logging

from feeders import IFeeder
from stations import StationData, Measurement


class LuftdatenFeeder(IFeeder):
    """
    Publishes a measurement to luftdaten.info site

    It's required to register your sensor on http://meine.luftdaten.info/sensors
    """

    def __init__(self, config: dict) -> None:
        super().__init__(config)
        self.apiServerUrl: str = "https://api.luftdaten.info/v1/push-sensor-data/"
        self.enable: bool = config["luftdaten"]["enable"]

    def feed(self, data: tp.List[StationData]) -> None:
        if self.enable:
            for d in data:
                sensor_id = f"raspi-{d.mac}"
                sensor_data = self._payload(d.version, d.measurement)
                self._post_data(sensor_id, 1, sensor_data)

    def _payload(self, version: str, meas: Measurement) -> dict:
        ret = {
            "software_version": version,
            "sensordatavalues": [
                {"value_type": "P1", "value": meas.measurement["pm10"]},
                {"value_type": "P2", "value": meas.measurement["pm25"]},
            ],
        }

        logging.debug(ret)
        return ret

    def _post_data(self, sensor_id: str, pin: int, data: dict) -> None:
        headers = {
            "Content-Type": "application/json",
            "X-Pin": str(pin),
            "X-Sensor": sensor_id,
        }

        try:
            r = requests.post(self.apiServerUrl, json=data, headers=headers, timeout=30)
            if r.status_code == 201:
                logging.info(f"LuftdatenFeeder: sent successfuly")
            else:
                logging.info(f"LuftdatenFeeder: unknown error")
        except Exception as e:
            logging.warning(f"LuftdatenFeeder: {e}")
