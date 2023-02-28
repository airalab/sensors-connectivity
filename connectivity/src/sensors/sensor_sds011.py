import time
import typing as tp
from dataclasses import dataclass, field

from ...constants import PING_MODEL, SDS011_MODEL
from .base import Device


@dataclass(repr=False, eq=False, kw_only=True)
class SensorSDS011(Device):
    """Represents a SDS011 sensor connected vua USB.
    :param public_key: Public key of the sensor or COMStation.
    :param data: Unparsed data from the sensor.
    :param model: Model of the sensor.
    :param geo: GPS coordinates of the sensor.
    """

    public_key: str
    data: tp.Union[tp.Deque, tuple] = None
    model: tp.Optional[str] = None
    geo: tp.Optional[list] = None

    def __post_init__(self) -> None:
        """Parse data from sensor and store into the corresponding variables."""

        self.timestamp = int(time.time())
        if not self.model:
            self.model = SDS011_MODEL
        if self.geo:
            # from COMStation
            self.geo_lat = self.geo[0]
            self.geo_lon = self.geo[1]
        else:
            # from Driver
            self.geo_lat = round(self.data[2], 6)
            self.geo_lon = round(self.data[3], 6)
        self.public = self.public_key
        if self.model is not PING_MODEL:
            self.measurement = {
                "pm10": round(self.data[1], 2),
                "pm25": round(self.data[0], 2),
                "timestamp": self.timestamp
            }
        else:
            self.measurement = {}
