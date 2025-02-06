import time
from dataclasses import dataclass, field
from functools import reduce
from substrateinterface import Keypair, KeypairType
from robonomicsinterface import RWS, Account

from connectivity.constants import PASKAL2MMHG, SDS011_MODEL
from .base import Device


@dataclass(repr=False, eq=False)
class Altruist(Device):
    """Represents a Robonomics Altruist Sensor.

    :param data: Unparsed data from the sensor.
    """

    data: dict = field(repr=False)

    def __post_init__(self) -> None:
        """Parse data from sensor and store into the corresponding variables."""

        super().__post_init__()
        self.id = str(self.data["robonomics_address"])
        self.model = int(self.data.get("model", SDS011_MODEL))
        self.public = self.id
        self.donated_by = str(self.data.get("donated_by", ""))
        sensors_data = self.data["sensordatavalues"]
        if not self._check_signature(sensors_data):
            return
        elif not self._is_address_in_subscription():
            return
        self.geo_lat = sensors_data["lat"]
        self.geo_lon = sensors_data["lon"]

        self.measurements_formatter(sensors_data)
        self.timestamp = int(time.time())
        self.measurement.update({"timestamp": self.timestamp})

    def _check_signature(self, sensor_data: dict) -> bool:
        """Verifies data with specified signature.
        :param sensor_data: Unparsed data from the sensor.
        :return: True if data is signed with this Keypair, otherwise False
        """

        sender_keypair = Keypair(
            ss58_address=self.public, crypto_type=KeypairType.ED25519
        )
        timestamp = str(int(time.time()))[:-2]
        sensor_data["time"] = timestamp
        return sender_keypair.verify(str(sensor_data), self.data["signature"])

    def _is_address_in_subscription(self) -> bool:
        account = Account()
        rws = RWS(account)
        return rws.is_in_sub(sub_owner_addr=self.data["owner"], addr=self.public)

    def measurements_formatter(self, sensor_data: dict) -> dict:
        mapping = {
            "temperature": "t",
            "pressure": "p",
            "humidity": "h",
            "pm10": "p1",
            "pm25": "p2",
            "noiseMax": "nm",
            "noiseAvg": "na",
        }

        self.measurement.update({key: sensor_data[value] for key, value in mapping.items() if value in sensor_data})


    def __str__(self) -> str:
        if self.model == SDS011_MODEL:
            return f"{{Public: {self.public}, geo: ({self.geo_lat},{self.geo_lon}), model: {self.model}, donated_by: {self.donated_by}, measurements: {self.measurement}}}"
        self.measurement.update({"geo": f"{self.geo_lat},{self.geo_lon}"})
        return f"{{Public: {self.public}, model: {self.model}, donated_by: {self.donated_by}, measurements: {self.measurement}}}"
