import typing as tp
import logging.config

from .gateways import (
    CrustGateway,
    LocalGateway,
    PinataGateway,
    TemporalGateway,
    PinArgs,
)
from connectivity.config.logging import LOGGING_CONFIG

logging.config.dictConfig(LOGGING_CONFIG)
logger = logging.getLogger("sensors-connectivity")

class PinningManager:
    def __init__(self, config: dict) -> None:
        self.config = config
        self.gateways = {}
        self._set_gateways()

    def _set_gateways(self) -> None:
        ipfs_endpoint: str = (
            self.config["robonomics"]["ipfs_provider"]
            if self.config["robonomics"]["ipfs_provider"]
            else "/ip4/127.0.0.1/tcp/5001/http"
        )
        self.gateways["local"] = LocalGateway(ipfs_endpoint)
        self.gateways["crust"] = CrustGateway(self.config["datalog"]["suri"])
        if self.config["datalog"]["pinata_secret"]:
            self.gateways["pinata"] = PinataGateway(
                self.config["datalog"]["pinata_api"],
                self.config["datalog"]["pinata_secret"],
            )
        if self.config["datalog"]["temporal_password"]:
            self.gateways["temporal"] = TemporalGateway(
                self.config["datalog"]["temporal_username"],
                self.config["datalog"]["temporal_password"],
            )

    def pin_to_gateways(self, file_path: str) -> tp.Optional[str]:
        pinArgs_for_local_gateway = PinArgs(file_path)
        try:
            file_hash, file_size = self.gateways["local"].pin(pinArgs_for_local_gateway)
        except Exception as e:
            logger.error(f"PinningManager: couldn't pin file to local gateway: {e}")
        pin_args = PinArgs(file_path=file_path, hash=file_hash, file_size=file_size)
        for gateway_name, gateway in self.gateways.items():
            if gateway_name != "local":
                try:
                    gateway.pin(pin_args)
                except Exception as e:
                    logger.error(f"PinningManager: couldn't pin {file_hash} to {gateway_name}: {e}")
        return file_hash
