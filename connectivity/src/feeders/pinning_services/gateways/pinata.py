import typing as tp
import logging.config
from pinatapy import PinataPy
from connectivity.config.logging import LOGGING_CONFIG
from .pinning_gateway import (
    PinningGateway,
    PinArgs,
)


logging.config.dictConfig(LOGGING_CONFIG)
logger = logging.getLogger("sensors-connectivity")

class PinataGateway(PinningGateway):

    def __init__(self, api_key: str, secret_key: str) -> None:
        self.api_key = api_key
        self.secret_key = secret_key


    def pin(self, args: PinArgs) -> None:
        """Pin file to Pinata for for better accessibility.
        Need to provide pinata credentials in the config file."""
        file_path: str = args.file_path
        try:
            logger.info("PinataGateway: Pinning file to Pinata")
            pinata = PinataPy(self.api_key, self.secret_key)
            result = pinata.pin_file_to_ipfs(path_to_file=file_path, save_absolute_paths=False)
            hash = result["IpfsHash"]
            logger.info(f"PinataGateway: File sent to pinata. Hash is {hash}")
        except Exception as e:
            logger.warning(f"PinataGateway: Failed while pining file to Pinata. Error: {e}")

