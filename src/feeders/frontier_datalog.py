import robonomicsinterface as RI
import typing as tp
import logging

from feeders import IFeeder
from stations import StationData, Measurement
from drivers.ping import PING_MODEL


class FrontierFeeder(IFeeder):
    def __init__(self, config: dict) -> None:
        super().__init__(config)

    def feed(self, data: tp.List[StationData]) -> None:
        if self.config["frontier"]["enable"]:
            interface = RI.RobonomicsInterface(seed=self.config["datalog"]["suri"])
            try:
                robonomics_receipt = interface.record_datalog(data)
                logging.info(
                    f"Data sent to Robonomics datalog and included in block {robonomics_receipt}"
                )
            except Exception as e:
                logging.info(
                    f"Something went wrong during extrinsic submission to Robonomics: {e}"
                )
