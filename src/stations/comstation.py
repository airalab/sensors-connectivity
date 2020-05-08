from stations import *
from drivers import SDS011

BROADCASTER_VERSION = "v0.1.0"


class COMStation(IStation):
    def __init__(self, config: dict):
        # sds_sensor_port: str = "/dev/ttyUSB0", work_time: int = 300):
        super().__init__(config)
        self.version = f"airalab-rpi-broadcaster-{BROADCASTER_VERSION}"

        self.sensor = SDS011(self.config["comstation"]["port"])
        self.sensor.set_work_period(work_time=int(self.config["comstation"]["work_period"]))

    def __str__(self):
        return f"{{Version: {self.version}, Start: {self.start_time}, MAC: {self.mac_address}}}"

    def get_data(self) -> StationData:
        meas = self.sensor.query()

        return StationData(
            self.version,
            self.mac_address,
            time.time() - self.start_time,
            Measurement(meas[0], meas[1])
        )