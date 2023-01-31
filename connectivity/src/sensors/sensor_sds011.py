import time
import typing as tp
from dataclasses import dataclass, field

from ...constants import PING_MODEL, SDS011_MODEL
from .base import Device

@dataclass(repr=False, eq=False, kw_only=True)
class SensorSDS011(Device):
    public_key: str
    data: tp.Union[tp.Deque, tuple] = None
    model: tp.Optional[str] = None
    geo: tp.Optional[list] = None

    def __post_init__(self) -> None:
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
            self.measurement = {"pm10": round(self.data[1], 2), "pm25": round(self.data[0],2), "timestamp": self.timestamp, "model": self.model}
        else:
            self.measurement = {}
        




