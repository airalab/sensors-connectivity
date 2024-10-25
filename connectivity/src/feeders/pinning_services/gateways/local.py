import typing as tp
import logging.config
import ipfshttpclient2

from connectivity.config.logging import LOGGING_CONFIG
from .pinning_gateway import (
    PinningGateway,
    PinArgs,
)

logging.config.dictConfig(LOGGING_CONFIG)
logger = logging.getLogger("sensors-connectivity")


class LocalGateway(PinningGateway):
    def __init__(self, endpoint: str) -> None:
        self.ipfs_endpoint = endpoint

    def pin(self, args: PinArgs) -> tp.Optional[tp.Tuple[str, int]]:
        file_path: str = args.file_path
        try:
            with ipfshttpclient2.connect(self.ipfs_endpoint) as client:
                response = client.add(file_path)
                file_hash = response["Hash"]
                file_size = response["Size"]
                logger.debug(f"LocalGateway: Hash, size: {file_hash}, {file_size}")
                return (file_hash, file_size)
        except Exception as e:
            logger.warning(
                f"LocalGateway: cou;dn't add file or connect to local gateway: {e}"
            )
