# This is an interface for any kind of feeders
import typing as tp


class IFeeder:
    """
    Feeder is an output in terms of this package.

    Basically this is where a feeder stands

    input1 \                        / feeder1
    input2 -  sensors-connectivity  - feeder2
    input3 /                        \ feeder3

    Every feeder must implement `feed()` method that requires a dict with measurements argument
    """

    def __init__(self, config: dict):
        """
        The initialization of an object.
        It's required to extract necessary configuration parameters by itself

        :param config: configuration dictionary
        """
        self.config = config

    @classmethod
    def get_classname(cls):
        return cls.__name__

    def feed(self, data: tp.List[dict]):
        """
        It's responsible for publishing data to an implemented channel

        :param data: dict with measurements that holds necessary data including measurements
        :return: nothing
        """
        raise NotImplementedError("Subclass must implement this!")
