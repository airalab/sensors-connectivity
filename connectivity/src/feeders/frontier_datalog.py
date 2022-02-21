import robonomicsinterface as RI
import typing as tp
import logging

from .ifeeder import IFeeder
from ..stations.istation import StationData, Measurement
from ..drivers.ping import PING_MODEL
import logging.config
from connectivity.config.logging import LOGGING_CONFIG

logging.config.dictConfig(LOGGING_CONFIG)
logger = logging.getLogger("sensors-connectivity")


class FrontierFeeder(IFeeder):
    def __init__(self, config: dict) -> None:
        super().__init__(config)

    def feed(self, data: tp.List[StationData]) -> None:
        if self.config["frontier"]["enable"]:
            interface = RI.RobonomicsInterface(seed=self.config["datalog"]["suri"])
            for d in data:
                try:
                    robonomics_receipt = interface.record_datalog(f"{d.measurement}")
                    logger.info(
                        f"Frontier Datalog: Data sent to Robonomics datalog and included in block {robonomics_receipt}"
                    )
                except Exception as e:
                    logger.warning(
                        f"Frontier Datalog: Something went wrong during extrinsic submission to Robonomics: {e}"
                    )
