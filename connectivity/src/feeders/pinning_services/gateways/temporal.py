import typing as tp
import logging.config
import requests

from connectivity.config.logging import LOGGING_CONFIG
from .pinning_gateway import (
    PinningGateway,
    PinArgs,
)

logging.config.dictConfig(LOGGING_CONFIG)
logger = logging.getLogger("sensors-connectivity")


class TemporalGateway(PinningGateway):
    def __init__(self, username: str, password: str) -> None:
        self.username = username
        self.password = password

    @staticmethod
    def pin(self, args: PinArgs) -> None:
        """Pin file to Temporal Cloud for for better accessibility.
        Need to provide corresponding credentials in the config file."""
        file_path: str = args.file_path
        auth_url = "https://api.temporal.cloud/v2/auth/login"
        token_resp = requests.post(
            auth_url, json={"username": self.username, "password": self.password}
        )
        token = token_resp.json()

        url_add = "https://api.temporal.cloud/v2/ipfs/public/file/add"
        headers = {"Authorization": f"Bearer {token['token']}"}
        resp = requests.post(
            url_add,
            files={"file": open(file_path), "hold_time": (None, 1)},
            headers=headers,
        )
        if resp.status_code == 200:
            logger.info("TemporalGateway: File pinned to Temporal Cloud")
        else:
            logger.warning(f"emporal Cloud: {resp}")
