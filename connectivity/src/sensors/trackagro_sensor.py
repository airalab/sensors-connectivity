import logging.config
import time
import typing
import typing as tp
from dataclasses import dataclass

from connectivity.config.logging import LOGGING_CONFIG

from ...constants import SDS011_MODEL
from .base import Device

logging.config.dictConfig(LOGGING_CONFIG)
logger = logging.getLogger("sensors-connectivity")


@dataclass(repr=False, eq=False)
class TrackAgro(Device):
    """Represents TrackAgro device.

    :param data: Unparsed data from the sensors.
    :param _time_from: The starting time from which to request data.
    """

    data: tp.List[tp.Dict[str, tp.Union[str, int, float]]]
    _time_from: int

    def __post_init__(self) -> None:
        """Parse data from sensor and store into the corresponding variables."""

        self.model = SDS011_MODEL
        meas = self._parse_data()
        if meas:
            self.measurement = meas
            self.public = self.generate_pubkey(self.id)

    @property
    def time_from(self):
        return self._time_from

    def _parse_data(self) -> dict | None:
        """Parse data from the sensor.

        :return: Parsed dictionary with measurements if success, None otherwise.
        """

        self.timestamp = 0
        meas = {}
        try:
            for d in self.data:
                self.id = d["id"]
                self.public = self.generate_pubkey(str(self.id))
                if "position" in d["key"]:
                    if d["key"] == "position.longitude":
                        self.geo_lon = float(d["value"])
                    else:
                        self.geo_lat = float(d["value"])
                else:
                    if d["ts"] > self.timestamp:
                        self.timestamp = d["ts"]
                    if any(d["key"] == key for key in meas.keys()):
                        if d["ts"] > meas[d["key"]]["timestamp"]:
                            meas[d["key"]].update({"value": d["value"], "timestamp": d["ts"]})
                    else:
                        meas.update({d["key"]: {"value": d["value"], "timestamp": d["ts"]}})
            parsed_meas = {}
            for k, v in meas.items():
                parsed_meas.update({k: v["value"]})
            parsed_meas.update({"timestamp": self.timestamp / 1000})
            self._time_from = self.timestamp
            parsed_meas.update({"pm10": "", "pm25": ""})
        except (UnboundLocalError, TypeError):
            pass
            return
        except Exception as e:
            logger.warning(f"TracAgro: error in parser {e}")
            return
        return parsed_meas
