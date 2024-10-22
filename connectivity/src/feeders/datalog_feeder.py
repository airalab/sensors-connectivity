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

from prometheus_client import Enum
from robonomicsinterface import RWS, Account, Datalog

from connectivity.config.logging import LOGGING_CONFIG
from connectivity.utils.datalog_db import DatalogDB
from connectivity.utils.ipfs_db import IPFSDB
from ...constants import MOBILE_GPS

from ...constants import PING_MODEL
from .ifeeder import IFeeder
from .pinning_services import PinningManager

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
        if v.get("geo"):
            ordered[k] = {"model": v["model"], "geo": v["geo"], "donated_by": v["donated_by"], "measurements": meas}
        else:
            ordered[k] = {"model": v["model"], "donated_by": v["donated_by"], "measurements": meas}
    return ordered


def _get_multihash(buf: set) -> tuple:
    """Write sorted measurements to the temp file, add file to IPFS and add
    measurements and hash in the database with 'not sent' status.

    :param buf: Set of measurements from all sensors.
    :param db: Database class object.
    :param endpoint: Endpoint for IPFS node. Default is local.
    :return: IPFS hash of the file and path to the temp file.
    """

    payload = {}
    for m in buf:
        try:
            if m.public in payload:
                payload[m.public]["measurements"].append(m.measurement)
            else:
                if m.model == MOBILE_GPS:
                    payload[m.public] = {
                        "model": m.model,
                        "donated_by": m.donated_by,
                        "measurements": [m.measurement],
                    }
                else:
                    payload[m.public] = {
                        "model": m.model,
                        "geo": "{},{}".format(m.geo_lat, m.geo_lon),
                        "donated_by": m.donated_by,
                        "measurements": [m.measurement],
                    }
        except Exception as e:
            logger.warning(f"Datalog Feeder: Couldn't store data: {e}")


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

    return temp.name, payload


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
        self.datalog_db: DatalogDB = DatalogDB(self.config["general"]["datalog_db_path"])
        self.ipfs_db: IPFSDB = IPFSDB(self.config["general"]["ipfs_db_path"])
        self.datalog_db.create_table()
        self.ipfs_db.create_table()
        self.pinning_manager = PinningManager(self.config)

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
                        self.last_time = time.time()
                        logger.debug("Datalog Feeder: About to publish collected data...")
                        logger.debug(f"Datalog Feeder: Buffer is {self.buffer}")
                        file_path, payload = _get_multihash(self.buffer)
                        ipfs_hash = self.pinning_manager.pin_to_gateways(file_path)
                        self.datalog_db.add_data("not sent", ipfs_hash, time.time(), json.dumps(payload))
                        self.buffer = set()
                        os.unlink(file_path)
                        self.to_datalog(ipfs_hash)
                    else:
                        logger.info("Datalog Feeder:Nothing to publish")
                else:
                    logger.info("Datalog Feeder: Still collecting measurements...")


    def to_datalog(self, ipfs_hash: str) -> None:
        """Send IPFS hash to Robonomics Datalog. It uses seed pharse from the config file.
        It can be sent either with RWS or general Datalog. To use RWS the account of the provided seed
        must have an active subscription.
        If the hash is sended successfully, the status in Database for this hash
        changes to `sent`.

        :param ipfs_hash: Ipfs hash of the file.
        """

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
            self.datalog_db.update_status("sent", ipfs_hash)
        except Exception as e:
            logger.warning(f"Datalog Feeder: Something went wrong during extrinsic submission to Robonomics: {e}")
            DATALOG_STATUS_METRIC.state("error")
