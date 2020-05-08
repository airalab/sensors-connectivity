import requests
import rospy
from feeders import IFeeder
from stations import StationData, Measurement


class LuftdatenFeeder(IFeeder):
    def __init__(self, config):
        super().__init__(config)
        self.apiServerUrl = "https://api.luftdaten.info/v1/push-sensor-data/"
        self.enable = config["luftdaten"]["enable"]

    def feed(self, data: StationData):
        if self.enable:
            sensor_id = f"raspi-{data.mac}"
            sensor_data = self._payload(data.measurement)

            self._post_data(sensor_id, 1, sensor_data)

    def _payload(self, meas: Measurement) -> dict:
        ret = {
            "software_version": self.version,
            "sensordatavalues": [
                {"value_type": "P1", "value": meas.pm10},
                {"value_type": "P2", "value": meas.pm25}
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

        rospy.loginfo("Sending data...")
        try:
            r = requests.post(self.apiServerUrl, json=data, headers=headers, timeout=30)
            rospy.loginfo(f"Response {r.status_code}")
        except Exception as e:
            rospy.logerr(e)
