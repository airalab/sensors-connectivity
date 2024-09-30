# Sensors Connectivity Module for Decentralized Sensors Network

The module for launching your own server instance for receiving data from Altruist Civil Station and further processing.

## Available Guides

We have created a comprehensive guide that explains the concept of the Decentralized Sensors Network and the Sensors Connectivity module. The guide is available at Robonomics Academy:

https://robonomics.academy/en/learn/sensors-connectivity-course/overview/


## Pre-requirements

The IPFS daemon should be installed to build this package. Assuming you are working on Linux:

```
wget https://dist.ipfs.io/go-ipfs/v0.8.0/go-ipfs_v0.8.0_linux-amd64.tar.gz
tar -xzf go-ipfs_v0.8.0_linux-amd64.tar.gz
cd go-ipfs
sudo bash install.sh
ipfs init
```

You also need at least Python 3.10.0 to run this project.

## Sensor Preparation

To prepare a sensor to work with the package, follow the instructions on [Robonomics Academy](https://robonomics.academy/en/learn/sensors-connectivity-course/setting-up-and-connecting-sensors/).


## Installation as PyPi package

Run the following commands:

```
pip3 install py-sr25519-bindings
pip3 install sensors-connectivity
```

### Configuration

At Robonomics Academy, you can find an article on setting up the proper configuration for your instance:

https://robonomics.academy/en/learn/sensors-connectivity-course/sensors-connectivity-config-options/


### Launch

First, launch IPFS daemon:

```
ipfs daemon --enable-pubsub-experiment
```

After the configuration and log files are set, you can run the service in another terminal:

```
sensors_connectivity "path/to/your/config/file"
```

You will see logs in your terminal and in `~/.logs`.

## Build from Source

To build a Python package from source, [Poetry](https://python-poetry.org/docs/#osx--linux--bashonwindows-install-instructions) should also be installed. Assuming you are working on Linux:

```
curl -sSL https://raw.githubusercontent.com/python-poetry/poetry/master/get-poetry.py | python -
```

Get the package and install dependencies:

```
git clone https://github.com/airalab/sensors-connectivity
cd sensors-connectivity
poetry install
```

### Configuration

At Robonomics Academy, you can find an article on setting up the proper configuration for your instance:

https://robonomics.academy/en/learn/sensors-connectivity-course/sensors-connectivity-config-options/

Make a copy of `default.json` and fill it using the description from the article.

You can also set a logging file. The default file for logs is `logging.py`, which uses the `console` and `file` handlers by default. Pay attention to the `file` handler.

The template is stored in `connectivity/config/logging_template.py`. You can specify the path (`filename`) where your logs will be stored (do not forget to create this directory if it doesn't exist). The default path for logs is `~/.logs`. You can find other handlers in the [Logging facility module](https://docs.python.org/3.8/library/logging.html) for Python.


### Launch

First, launch the IPFS daemon:

```
ipfs daemon --enable-pubsub-experiment
```
After the configuration and log files are set, you can run the service in another terminal:

```
poetry run sensors_connectivity "path/to/your/config/file"  
```

If your log file is set with the `console` handler, you will be able to see logs in your terminal.

### Development

To test the module with an HTTP station, use:

```
poetry run test_mobile_lab
test_environmental_box
```

For more information about development, check the `/docs` directory:

https://github.com/airalab/sensors-connectivity/tree/master/docs

### Troubleshooting

**Python.h: No such file or directory:**:

If, during the execution of the `poetry install` command, you encounter this error, you need to install the header files and static libraries for `python-dev`. Use your package manager for installation. For example, with `apt`, you need to run:

```
sudo apt install python3-dev
```
> **Note**: `python3-dev` does not cover all versions for Python 3. The service needs at least Python 3.10, for that you may need to specify the version: `sudo apt install python3.10-dev`.

[Here](https://stackoverflow.com/a/21530768) you can find examples for other package managers.

**Python versions mismatch:**

If, during the execution of the `poetry install` command, you encounter a `SolverProblemError`, which states `The current project's Python requirement (3.6.9) is not compatible with some of the required packages' Python requirement:..`, even though you have a newer version of Python (e.g., Python 3.10.9), you may need to specify the Python version for Poetry:

```
poetry env use python3.10.9
```
