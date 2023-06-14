# Sensors Connectivity Module for Decentralized Sensors Network

The module for launching your own server instance for receiving data from sensors (like SDS011) and their further processing.

## Available Guides

We have created a nice complete guide that reveals the idea of the Decentralized Sensors Network and the Sensors Connectivity module. The guide is available at Robonomics Academy:

https://robonomics.academy/en/learn/sensors-connectivity-course/overview/

## Pre-requirements

IPFS daemon should be installed to build this package. Assuming you work on Linux:

```
wget https://dist.ipfs.io/go-ipfs/v0.8.0/go-ipfs_v0.8.0_linux-amd64.tar.gz
tar -xzf go-ipfs_v0.8.0_linux-amd64.tar.gz
cd go-ipfs
sudo bash install.sh
ipfs init
```

You also need at least `Python3.10.0` to run this project.

## Sensor Preparation

To prepare a sensor for the work with the package follow instructions on [Robonomics Academy](https://robonomics.academy/en/learn/sensors-connectivity-course/setting-up-and-connecting-sensors/).

## Installation as PyPi package

Run the following commands:

```
pip3 install py-sr25519-bindings
pip3 install sensors-connectivity
```

### Configuration

At Robonomics Academy you can find an article to set a proper configuration for your instance:

https://robonomics.academy/en/learn/sensors-connectivity-course/sensors-connectivity-config-options/

### Launch

First, launch IPFS daemon:

```
ipfs daemon --enable-pubsub-experiment
```

After configuration and log files are set, you can run the service in another terminal:

```
sensors_connectivity "path/to/your/config/file"
```

You will see logs in your terminal and in `~/.logs`.

## Build from Source

To build a python package from source [Poetry](https://python-poetry.org/docs/#osx--linux--bashonwindows-install-instructions) should be also installed. Assuming you work on Linux:

```
curl -sSL https://raw.githubusercontent.com/python-poetry/poetry/master/get-poetry.py | python -
```

Get a package and install dependencies:

```
git clone https://github.com/airalab/sensors-connectivity
cd sensors-connectivity
poetry install
```

### Configuration

At Robonomics Academy you can find an article to set a proper configuration for your instance:

https://robonomics.academy/en/learn/sensors-connectivity-course/sensors-connectivity-config-options/

Make a copy of `default.json` and fill it using description from the article.

You also can set a logging file. The default file for logs is `logging.py`, which uses `console` and `file` handler by default. Pay attention to the `file` handler.

The template is stored in `connectivity/config/logging_template.py`. You can specify the path (`filename`), where your logs will be stored in (do not forget to create this directory if it doesn't exist). Default path for logs is `~/.logs`. You can find any other handlers in the [Logging facility module](https://docs.python.org/3.8/library/logging.html) for Python.

### Launch

First, launch IPFS daemon:

```
ipfs daemon --enable-pubsub-experiment
```
After configuration and log files are set, you can run the service in another terminal:

```
poetry run sensors_connectivity "path/to/your/config/file"  
```

If your log file is set with `console` handler, you will be able to see logs in your terminal.

### Development

To test the module with HTTP station use:
```
poetry run test_mobile_lab
test_environmental_box
```

For more information about development check `/docs` directory:

https://github.com/airalab/sensors-connectivity/tree/master/docs

### Troubleshooting

**Python.h: No such file or directory:**:

If during running `poetry install` command you get such error, you need to install the header files and static libraries for `python-dev`. Use your package manager for installation. For example, for `apt` you need to run:
```
sudo apt install python3-dev
```
> Note:
`python3-dev` does not cover all versions for Python 3. The service needs at least Python 3.10, for that you may need to specify the version: `sudo apt install python3.10-dev`.

[Here](https://stackoverflow.com/a/21530768) you can find examples for other package managers.

**Python versions mismatch:**

If during running `poetry install` command you get `SolverProblemError`, which says `The current project's Python requirement (3.6.9) is not compatible with some of the required packages Python requirement:..`, even though you have older version of Python (e.g. Python 3.10.9), you may need to specify the Python version for Poetry:

```
poetry env use python3.10.9
```
