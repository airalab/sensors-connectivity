# This is interface for any kind of feeders
from stations import StationData


class IFeeder:
    def __init__(self, config: dict):
        """
        The initialization of an object.
        It's required to extract necessary configuration parameters by itself

        :param config: configuration dictionary
        """
        self.config = config

    def feed(self, data: StationData):
        """
        It's responsible for publishing data to an implemented channel

        :param data: StationData object that holds necessary data including measurements
        :return: nothing
        """
        raise NotImplementedError("Subclass must implement this!")
