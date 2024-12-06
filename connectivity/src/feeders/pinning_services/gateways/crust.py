import typing as tp
import logging.config
from crustinterface import Mainnet

from connectivity.config.logging import LOGGING_CONFIG
from .pinning_gateway import (
    PinningGateway,
    PinArgs,
)

logging.config.dictConfig(LOGGING_CONFIG)
logger = logging.getLogger("sensors-connectivity")


class CrustGateway(PinningGateway):
    def __init__(self, seed: str) -> None:
        self.mainnet = Mainnet(seed=seed)

    
    def pin(self, args: PinArgs) -> None:
        file_hash: str = args.hash
        file_size: int = args.file_size
        if self._can_upload(file_size):
            try:
                logger.info(
                    f"CrustGateway: Start adding {file_hash} to crust with size :{file_size}"
                )
                file_stored = self.mainnet.store_file(file_hash, file_size)
                logger.info(
                    f"CrustGateway: File stored in Crust. Extrinsic data is:  {file_stored}"
                )
            except Exception as e:
                logger.warning(
                    f"CrustGateway: error while uploading file to crust: {e}"
                )
                return None
        else:
            logger.warning(
                f"CrustGateway: Not enough account balance to store the file in Crust Network"
            )

    def _can_upload(self, file_size: int) -> bool:
        """Check whether there is enough tokens on balance"""
        balance = self._get_balance()
        if not balance:
            return None
        approximately_price = self._get_store_price(file_size)
        return balance >= approximately_price

    def _get_balance(self) -> tp.Optional[int]:
        try:
            balance = self.mainnet.get_balance()
            logger.debug(f"CrustGateway: Actual balance in crust network: {balance}")
            return balance
        except Exception as e:
            logger.warning(f"CrustGateway: Error while getting account balance: {e}")
            return None

    def _get_store_price(self, file_size: int) -> tp.Optional[int]:
        """Check price in Main net. Price in pCRUs"""
        price = self.mainnet.get_appx_store_price(int(file_size))
        logger.debug(f"CrustGateway: Approximate cost to store the file: {price}")
        return price
