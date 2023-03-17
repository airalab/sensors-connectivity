"""
Datalog Feeder. This feeder collects data from the stations and adds it to the buffer. 
Every `dump_interval` (from the config) the buffer writes to the file which pins to IPFS.
IPFS hash of the file sends to Robonomics Datalog.
"""
import json
import logging.config
import os
import threading
import time
import typing as tp
from tempfile import NamedTemporaryFile

import ipfshttpclient2
import requests
from pinatapy import PinataPy
from prometheus_client import Enum
from robonomicsinterface import RWS, Account, Datalog

from connectivity.config.logging import LOGGING_CONFIG
from connectivity.utils.database import DataBase

from ...constants import PING_MODEL
from .ifeeder import IFeeder

logging.config.dictConfig(LOGGING_CONFIG)
logger = logging.getLogger("sensors-connectivity")

thlock = threading.RLock()

DATALOG_STATUS_METRIC = Enum(
    "connectivity_sensors_datalog_feeder",
    "This will give last status of datalog feeder",
    states=["starting", "success", "error"],
)
DATALOG_STATUS_METRIC.state("starting")

DATALOG_MEMORY_METRIC = Enum(
    "connectivity_sensors_memory_datalog_feeder",
    "This will give last status of free memoory in datalog feeder",
    states=["success", "error"],
)


def _sort_payload(data: dict) -> dict:
    """Sort measurements dict with timestamp.

    :param data: Measurements dict.
    :return: Sorted measurements dict.
    """

    ordered = {}
    for k, v in data.items():
        meas = sorted(v["measurements"], key=lambda x: x["timestamp"])
        ordered[k] = {"model": v["model"], "geo": v["geo"], "measurements": meas}
    return ordered


def _get_multihash(buf: set, db: object, endpoint: str = "/ip4/127.0.0.1/tcp/5001/http") -> tp.Dict[str, str]:
    """Write sorted measurements to the temp file, add file to IPFS and add 
    measurements and hash in the database with 'not sent' status.

    :param buf: Set of measurements from all sensors.
    :param db: Database class object.
    :param endpoint: Endpoint for IPFS node. Default is local.
    :return: IPFS hash of the file and path to the temp file.
    """

    payload = {}
    for m in buf:
        if m.public in payload:
            payload[m.public]["measurements"].append(m.measurement)
        else:
            payload[m.public] = {
                "model": m.model,
                "geo": "{},{}".format(m.geo_lat, m.geo_lon),
                "measurements": [m.measurement],
            }

    logger.debug(f"Payload before sorting: {payload}")
    payload = _sort_payload(payload)
    logger.debug(f"Payload sorted: {payload}")
    try:
        temp = NamedTemporaryFile(mode="w", delete=False)
        logger.debug(f"Created temp file: {temp.name}")
        temp.write(json.dumps(payload))
        temp.close()
        DATALOG_MEMORY_METRIC.state("success")
    except Exception as e:
        DATALOG_MEMORY_METRIC.state("error")

    with ipfshttpclient2.connect(endpoint) as client:
        response = client.add(temp.name)
        db.add_data("not sent", response["Hash"], time.time(), json.dumps(payload))
        return (response["Hash"], temp.name)


def _pin_to_pinata(file_path: str, config: dict) -> None:
    """Pin file to Pinata for for better accessibility.
    Need to provide pinata credentials in the config file.

    :param file_path: Path to the temp file.
    :param config: Configuration dictionary.
    """

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
    publishing an IPFS hash to Robonomics on Substrate.
    It requires an account's seed pharse. Optional, hash can be pinned in
    Pinata and/or temporal if corresponding credentials provided.
    """

    def __init__(self, config) -> None:
        """Create table for Database in the file from config.

        :param config: Configuration dictionary
        """

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

    def feed(self, data: tp.List[dict]) -> None:
        """Main function of the feeder and it is called in `main.py`. It collects 
        data into buffer and, every `interval` from config, adds it to IPFS and sends the hash 
        to Robonomics Datalog.

        :param data: Data from the stations.
        """
        if self.config["datalog"]["enable"]:
            if data:
                for d in data:
                    if d.public and d.model != PING_MODEL:
                        logger.debug(f"DatalogFeeder: Adding data to buffer: {d}")
                        self.buffer.add(d)

                if (time.time() - self.last_time) >= self.interval:
                    if self.buffer:
                        logger.debug("Datalog Feeder: About to publish collected data...")
                        logger.debug(f"Datalog Feeder: Buffer is {self.buffer}")
                        ipfs_hash, file_path = _get_multihash(self.buffer, self.db, self.ipfs_endpoint)
                        self._pin_to_temporal(file_path)
                        _pin_to_pinata(file_path, self.config)
                        os.unlink(file_path)
                        self.to_datalog(ipfs_hash)
                    else:
                        logger.info("Datalog Feeder:Nothing to publish")
                else:
                    logger.info("Datalog Feeder: Still collecting measurements...")

    def _pin_to_temporal(self, file_path: str) -> None:
        """Pin file to Temporal Cloud for for better accessibility.
        Need to provide corresponding credentials in the config file.

        :param file_path: Path to the temp file.
        """

        username = self.config["datalog"]["temporal_username"]
        password = self.config["datalog"]["temporal_password"]
        if username and password:
            auth_url = "https://api.temporal.cloud/v2/auth/login"
            token_resp = requests.post(auth_url, json={"username": username, "password": password})
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
        """Send IPFS hash to Robonomics Datalog. It uses seed pharse from the config file.
        It can be sent either with RWS or general Datalog. To use RWS the account of the provided seed
        must have an active subscription.
        If the hash is sended successfully, the status in Database for this hash
        changes to `sent`.

        :param ipfs_hash: Ipfs hash of the file.
        """

        logger.info(ipfs_hash)
        self.last_time = time.time()
        self.buffer = set()
        account = Account(seed=self.config["datalog"]["suri"])
        rws = RWS(account)
        try:
            if rws.get_days_left():
                rws_sub_owner = account.get_address()
                if not rws.is_in_sub(sub_owner_addr=rws_sub_owner, addr=rws_sub_owner):
                    rws.set_devices([rws_sub_owner])
                rws_datalog = Datalog(account, rws_sub_owner=rws_sub_owner)
                robonomics_receipt = rws_datalog.record(ipfs_hash)
            else:
                datalog = Datalog(account)
                robonomics_receipt = datalog.record(ipfs_hash)
            logger.info(
                f"Datalog Feeder: Ipfs hash sent to Robonomics Parachain and included in block {robonomics_receipt}"
            )
            DATALOG_STATUS_METRIC.state("success")
            self.db.update_status("sent", ipfs_hash)
        except Exception as e:
            logger.warning(f"Datalog Feeder: Something went wrong during extrinsic submission to Robonomics: {e}")
            DATALOG_STATUS_METRIC.state("error")
