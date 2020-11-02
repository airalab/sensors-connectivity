import requests
import rospy
from feeders import IFeeder
from stations import StationData, Measurement


class LuftdatenFeeder(IFeeder):
    """
    Publishes a measurement to luftdaten.info site

    It's required to register your sensor on http://meine.luftdaten.info/sensors
    """

    def __init__(self, config):
        super().__init__(config)
        self.apiServerUrl = "https://api.luftdaten.info/v1/push-sensor-data/"
        self.enable = config["luftdaten"]["enable"]

    def feed(self, data: [StationData]):
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
                {"value_type": "P2", "value": meas.measurement["pm25"]}
            ]
        }

        rospy.logdebug(ret)
        return ret

    def _post_data(self, sensor_id: str, pin: int, data: dict):
        headers = {
                "Content-Type": "application/json",
                "X-Pin": str(pin),
                "X-Sensor": sensor_id
                }

        try:
            r = requests.post(self.apiServerUrl, json=data, headers=headers, timeout=30)
            if r.status_code == 201:
                rospy.loginfo(f"LuftdatenFeeder: sent successfuly")
            else:
                rospy.loginfo(f"LuftdatenFeeder: unknown error")
        except Exception as e:
            rospy.logerr(f"LuftdatenFeeder: {e}")
