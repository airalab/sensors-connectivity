import requests
import rospy
from sds011.station import StationData


class LuftdatenFeeder:
    def __init__(self, config):
        self.apiServerUrl = "https://api.luftdaten.info/v1/push-sensor-data/"
        self.enable = config["enable"]

    def feed(self, data: StationData):
        if self.enable:
            sensor_id = f"raspi-{data.mac}"
            sensor_data = data.to_json()

            self._post_data(sensor_id, 1, sensor_data)

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
