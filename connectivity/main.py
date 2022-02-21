#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import json
import time
import logging.config

# Standart, System and Third party
import threading
from threading import Timer
import os
import sys
import sentry_sdk
import argparse

from connectivity.config.logging import LOGGING_CONFIG
from .src.stations import COMStation, HTTPStation, MQTTStation
from .src.feeders import (
    LuftdatenFeeder,
    RobonomicsFeeder,
    DatalogFeeder,
    FrontierFeeder,
)
from .src.stations.trackargostation import TrackAgroStation
from .utils.database import DataBase


logging.config.dictConfig(LOGGING_CONFIG)
logger = logging.getLogger("sensors-connectivity")


class WorkerNode:
    """The main class that initialize stations and feeders and launches the loop."""

    def __init__(self, path: str) -> None:
        self.config: dict = self._read_configuration(path)
        logging.debug(self.config)
        self.interval: int = self.config["general"]["publish_interval"]
        sentry_sdk.init(self.config["dev"]["sentry"])

        self.stations: list = self._populate_stations()
        self.feeders: list = self._populate_feeders()
        self.station_data: list = []
        self.db: DataBase = DataBase(self.config)

    def _read_configuration(self, config_path: str) -> dict:
        """Internal method

        Loads configuration.
        """

        try:
            with open(config_path) as f:
                content = f.read()
                config = json.loads(content)
                logging.debug(f"Configuration dict: {content}")
                return config
        except Exception as e:
            while True:
                logging.error("Configuration file is broken or not readable!")
                logging.error(e)
                time.sleep(5)

    def _populate_stations(self) -> list:
        """Initialize stations

        If you make a new one, add initialization here.
        Don't forget to append an instance to `s`
        """
        s = []
        if self.config["comstation"]["enable"]:
            s.append(COMStation(self.config))

        if self.config["httpstation"]["enable"]:
            s.append(HTTPStation(self.config))

        if self.config["mqttstation"]["enable"]:
            s.append(MQTTStation(self.config))

        if self.config["trackagro"]["enable"]:
            s.append(TrackAgroStation(self.config))

        return s

    def _populate_feeders(self) -> list:
        """Initialize feeders

        If you make a new one, add initialization here.
        """
        f = [
            LuftdatenFeeder(self.config),
            FrontierFeeder(self.config),
            RobonomicsFeeder(self.config),
            DatalogFeeder(self.config),
        ]

        return f

    def spin(self) -> None:
        def get_result() -> None:
            self.station_data.clear()
            logger.info("Getting data from the stations...")
            Timer(self.interval, get_result).start()
            for s in self.stations:
                self.station_data.append(s.get_data())
            logger.info(f"{s.version}: {self.station_data}")

        def send_result() -> None:
            logger.info("Sending result to the feeders...")
            Timer(self.interval, send_result).start()
            for f in self.feeders:
                class_name = f.get_classname()
                for d in self.station_data:
                    t = threading.Thread(target=f.feed, args=(d,))
                    t.name = class_name
                    t.start()
                    t.join(timeout=20)

        def db_watcher() -> None:
            logger.info("Checking data base...")
            Timer(3600, db_watcher).start()
            for data in self.db.checker(time.time()):
                for hash in data:
                    self.feeders[3].to_datalog(hash)

        get_result()
        send_result()
        db_watcher()


def run() -> None:
    parser = argparse.ArgumentParser(description="Add config path.")
    parser.add_argument("config_path", type=str, help="config path")
    args = parser.parse_args()
    WorkerNode(args.config_path).spin()


if __name__ == "__main__":
    WorkerNode().spin()
