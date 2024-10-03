"""
Main script of connectivity module. If you add a new station and/or a new feeder do not forget to add them to 
`_populate_stations` and `_populate_feeders` in WorkerNode class respectively. 
"""
import argparse
import json
import logging.config
import threading
import time
from threading import Timer

import sentry_sdk
from prometheus_client import start_http_server

from connectivity.config.logging import LOGGING_CONFIG

from .src.feeders import DatalogFeeder, FrontierFeeder, RobonomicsFeeder
from .src.stations import COMStation, HTTPStation, MQTTStation
from .src.stations.trackargostation import TrackAgroStation
from .utils.datalog_db import DatalogDB

logging.config.dictConfig(LOGGING_CONFIG)
logger = logging.getLogger("sensors-connectivity")


class WorkerNode:
    """The main class that initialize stations and feeders and launches the loop.
    `spin()` is the main function of the class. This class read configuration file,
    add all enabled stations into list in `self.stations` and all feeders in `self.feeders`
    variables. 
    """

    def __init__(self, path: str) -> None:
        """Save enabled stations and all feeders. Load configuration file
        from `path`. Initialised DataBase class.
        
        :param path: Path to configuration file. Taken from command line argument.
        """

        self.config: dict = self._read_configuration(path)
        logging.debug(self.config)
        self.interval: int = self.config["general"]["publish_interval"]
        sentry_sdk.init(self.config["dev"]["sentry"])

        self.stations: list = self._populate_stations()
        self.feeders: list = self._populate_feeders()
        self.station_data: list = []
        self.datalog_db: DatalogDB = DatalogDB(self.config["general"]["datalog_db_path"])

    def _read_configuration(self, config_path: str) -> dict:
        """Internal method. Loads configuration.
        :param config_path: Path to configuration file.
        :return: Python dict with config.
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
        """Initialize stations.
        If you make a new one, add initialization here.
        Don't forget to append an instance to `s`.
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
        """Initialize feeders.
        If you make a new one, add initialization here.
        """

        f = [
            FrontierFeeder(self.config),
            RobonomicsFeeder(self.config),
            DatalogFeeder(self.config),
        ]

        return f

    def spin(self) -> None:
        """Function to run the main threads. Initially, each
        function is called from this function, then they are called 
        by Timer.
        """

        def get_result() -> None:
            """Get data from the stations stored in `self.stations`, using 
            `get_data()` function, which is initialised in every station. 
            The function is called using Timer with the interval setted in config file.
            Data stores in `self.station_data` which is cleared every time the
            function is called.
            """

            self.station_data.clear()
            logger.info("Getting data from the stations...")
            Timer(self.interval, get_result).start()
            for s in self.stations:
                self.station_data.append(s.get_data())
            logger.info(f"{s.version}: {self.station_data}")

        def send_result() -> None:
            """Send data to the feeders from `self.feeders`, using
            `feed()` function, which is initialised in every feeder.
            Each feeder has its own thread to make it alive
            even when one of the feeder is down. The function 
            is called using Timer with the interval setted in config file.
            """

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
            """Check database for unsended ipfs hashes and 
            send them to Datalog again using DatalogFeeder.
            """

            logger.info("Checking data base...")
            Timer(3600, db_watcher).start()
            for data in self.datalog_db.checker(time.time()):
                for hash in data:
                    self.feeders[2].to_datalog(hash)

        get_result()
        send_result()
        db_watcher()


def run() -> None:
    """Main function of the script. Read config path as argument from command line,
    initialize the `WorkerNode` with the config path and run its `spin()` function.
    Initialize Prometheus server on port 8000.
    """
    
    parser = argparse.ArgumentParser(description="Add config path.")
    parser.add_argument("config_path", type=str, help="config path")
    args = parser.parse_args()
    start_http_server(8000)
    WorkerNode(args.config_path).spin()
