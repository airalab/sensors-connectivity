import json
import time
import typing as tp
from tempfile import NamedTemporaryFile
import ipfshttpclient
import robonomicsinterface as RI
import requests
import threading
from pinatapy import PinataPy

from connectivity.utils.database import DataBase
from .ifeeder import IFeeder
from ..stations.istation import StationData, Measurement
from ..drivers.ping import PING_MODEL
import logging.config
from connectivity.config.logging import LOGGING_CONFIG

logging.config.dictConfig(LOGGING_CONFIG)
logger = logging.getLogger("sensors-connectivity")

thlock = threading.RLock()


def _sort_payload(data: dict) -> dict:
    ordered = {}
    for k, v in data.items():
        meas = sorted(v["measurements"], key=lambda x: x["timestamp"])
        ordered[k] = {"model": v["model"], "geo": v["geo"], "measurements": meas}
    return ordered


def _get_multihash(
    buf: set, db: object, endpoint: str = "/ip4/127.0.0.1/tcp/5001/http"
) -> tp.Dict[str, str]:
    payload = {}
    for m in buf:
        if m.public in payload:
            payload[m.public]["measurements"].append(m.measurement_check())
        else:
            payload[m.public] = {
                "model": m.model,
                "geo": "{},{}".format(m.geo_lat, m.geo_lon),
                "measurements": [m.measurement_check()],
            }

    logger.debug(f"Payload before sorting: {payload}")
    payload = _sort_payload(payload)
    logger.debug(f"Payload sorted: {payload}")

    temp = NamedTemporaryFile(mode="w", delete=False)
    logger.debug(f"Created temp file: {temp.name}")
    temp.write(json.dumps(payload))
    temp.close()

    with ipfshttpclient.connect(endpoint) as client:
        response = client.add(temp.name)
        db.add_data("not sent", response["Hash"], time.time(), json.dumps(payload))
        return (response["Hash"], temp.name)


def _pin_to_pinata(file_path: str, config: dict) -> None:
    pinata_api = config["datalog"]["pinata_api"]
    pinata_secret = config["datalog"]["pinata_secret"]
    if pinata_secret:
        try:
            logger.info("DatalogFeeder: Pinning file to Pinata")
            pinata = PinataPy(pinata_api, pinata_secret)
            pinata.pin_file_to_ipfs(file_path)
            hash = pinata.pin_list()["rows"][0]["ipfs_pin_hash"]
            logger.info(f"DatalogFeeder: File sent to pinata. Hash is {hash}")
        except Exception as e:
            logger.warning(f"DatalogFeeder: Failed while pining file to Pinata. Error: {e}")


class DatalogFeeder(IFeeder):
    """
    The feeder is responsible for collecting measurements and
    publishing an IPFS hash to Robonomics on Substrate

    It requires the full path to `robonomics` execution binary
    and an account's private key
    """

    def __init__(self, config) -> None:
        super().__init__(config)
        self.last_time: float = time.time()
        self.buffer: set = set()
        self.interval: int = self.config["datalog"]["dump_interval"]
        self.ipfs_endpoint: str = (
            config["robonomics"]["ipfs_provider"]
            if config["robonomics"]["ipfs_provider"]
            else "/ip4/127.0.0.1/tcp/5001/http"
        )
        self.db: DataBase = DataBase(self.config)
        self.db.create_table()

    def feed(self, data: tp.List[StationData]) -> None:
        if self.config["datalog"]["enable"]:
            if data:
                for d in data:
                    if d.measurement.public and d.measurement.model != PING_MODEL:
                        logger.debug(f"DatalogFeeder: Adding data to buffer: {d.measurement}")
                        self.buffer.add(d.measurement)

                if (time.time() - self.last_time) >= self.interval:
                    if self.buffer:
                        logger.debug("Datalog Feeder: About to publish collected data...")
                        logger.debug(f"Datalog Feeder: Buffer is {self.buffer}")
                        ipfs_hash, file_path = _get_multihash(
                            self.buffer, self.db, self.ipfs_endpoint
                        )
                        self._pin_to_temporal(file_path)
                        _pin_to_pinata(file_path, self.config)
                        self.to_datalog(ipfs_hash)
                    else:
                        logger.info("Datalog Feeder:Nothing to publish")
                    # self.buffer = set()
                    # self.last_time = time.time()
                else:
                    logger.info("Datalog Feeder: Still collecting measurements...")

    def _pin_to_temporal(self, file_path: str) -> None:
        username = self.config["datalog"]["temporal_username"]
        password = self.config["datalog"]["temporal_password"]
        if username and password:
            auth_url = "https://api.temporal.cloud/v2/auth/login"
            token_resp = requests.post(
                auth_url, json={"username": username, "password": password}
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
                logger.info("Datalog Feeder: File pinned to Temporal Cloud")

    def to_datalog(self, ipfs_hash: str) -> None:
        logger.info(ipfs_hash)
        self.last_time = time.time()
        self.buffer = set()
        interface = RI.RobonomicsInterface(seed=self.config["datalog"]["suri"])
        try:
            robonomics_receipt = interface.record_datalog(ipfs_hash)
            logger.info(
                f"Datalog Feeder: Ipfs hash sent to Robonomics Parachain and included in block {robonomics_receipt}"
            )
            self.db.update_status("sent", ipfs_hash)
        except Exception as e:
            logger.warning(
                f"Datalog Feeder: Something went wrong during extrinsic submission to Robonomics: {e}"
            )
