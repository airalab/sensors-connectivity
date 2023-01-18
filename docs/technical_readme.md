## Sensors-Connectivtiy Sheme
This sheme introduces intection between classes in the package. It does not cover all entities but the most popular. For more information have a look at the code.

![connectivity sheme][sheme]

It is possible to implement 3 new entities:
- station (an input/source of data) -- check `connectivity/srs/stations/istation.py`
- feeder (an output of data) -- check `connectivity/srs/feeders/ifeeder.py`
- sensor (an interface for a particular sensor) -- check `connectivity/sensors/sensor_template.py`

If you implemented a new station or feeder do not forget to add it to `_populate_stations` or `_populate_feeders` in `main.py` respectively. If you add a new sensor do not forget to add it to the corresponding station (according to the protocol your sensor uses to send data).

[sheme]: <https://github.com/airalab/sensors-connectivity/blob/refactoring/docs/connectivity.jpg>

