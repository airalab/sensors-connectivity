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

from prometheus_client import Enum
from robonomicsinterface import RWS, Account, Datalog

from connectivity.config.logging import LOGGING_CONFIG
from connectivity.utils.datalog_db import DatalogDB
from connectivity.utils.ipfs_db import IPFSDB
from connectivity.utils.datalog_payload import (
    create_payload,
    sort_payload,
    create_tmp_file,
)
from connectivity.constants import PING_MODEL, POLKA_REMOTE_WS, KSM_REMOTE_WS

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
        self.datalog_db: DatalogDB = DatalogDB(
            self.config["general"]["datalog_db_path"]
        )
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
                        logger.debug(
                            "Datalog Feeder: About to publish collected data..."
                        )
                        logger.debug(f"Datalog Feeder: Buffer is {self.buffer}")
                        payload = create_payload(self.buffer)
                        sorted_payload = sort_payload(payload)
                        try:
                            tmp_file_path = create_tmp_file(sorted_payload)
                            DATALOG_MEMORY_METRIC.state("success")
                        except Exception as e:
                            DATALOG_MEMORY_METRIC.state("error")
                            logger.warning(
                                f"Datalog Feeder: couldn't create tmp file: {e}"
                            )

                        ipfs_hash = self.pinning_manager.pin_to_gateways(tmp_file_path)
                        self.datalog_db.add_data(
                            "not sent", ipfs_hash, time.time(), json.dumps(payload)
                        )
                        self.buffer = set()
                        os.unlink(tmp_file_path)
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

        ksm_account = Account(seed=self.config["datalog"]["suri"], remote_ws=KSM_REMOTE_WS)
        polkadot_account = Account(seed=self.config["datalog"]["suri"], remote_ws=POLKA_REMOTE_WS)
        rws = RWS(ksm_account)
        try:
            if rws.get_days_left():
                rws_sub_owner = ksm_account.get_address()
                if not rws.is_in_sub(sub_owner_addr=rws_sub_owner, addr=rws_sub_owner):
                    rws.set_devices([rws_sub_owner])
                rws_datalog = Datalog(ksm_account, rws_sub_owner=rws_sub_owner)
                robonomics_receipt = rws_datalog.record(ipfs_hash)
            else:
                datalog = Datalog(ksm_account)
                robonomics_receipt = datalog.record(ipfs_hash)
            logger.info(
                f"Datalog Feeder: Ipfs hash sent to Robonomics KSM Parachain and included in block {robonomics_receipt}"
            )
            DATALOG_STATUS_METRIC.state("success")
            self.datalog_db.update_status("sent", ipfs_hash)
        except Exception as e:
            logger.warning(
                f"Datalog Feeder: Something went wrong during extrinsic submission to KSM Robonomics: {e}"
            )
            DATALOG_STATUS_METRIC.state("error")
        try:
            datalog = Datalog(polkadot_account)
            robonomics_receipt = datalog.record(ipfs_hash)
            logger.info(
                f"Datalog Feeder: Ipfs hash sent to Robonomics Polkadot Parachain and included in block {robonomics_receipt}"
            )
        except Exception as e:
            logger.warning(
                f"Datalog Feeder: Something went wrong during extrinsic submission to Polkadot Robonomics: {e}"
            )
