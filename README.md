# Sensors Connectivity

A Python service that acts as the data aggregation layer in the Robonomics decentralized sensors network. It receives signed sensor measurements, validates them, and distributes data through two channels: real-time IPFS pubsub for the Robonomics dApp and batched datalogs to the Robonomics parachain.

## Architecture Overview

Sensors Connectivity is the provider node in the decentralized sensors network:

- Receives signed measurements from Altruist devices via HTTP POST (port 8001 by default)
- Also accepts data from other sensor types: EnvironmentalBox, MobileLab, LoRa sensors
- Publishes real-time data via IPFS pubsub for the Robonomics dApp
- Batches sensor data, pins to IPFS, and writes the hash as a datalog to the Robonomics parachain

```
Altruist / Other sensors
  |
  v HTTP POST (port 8001)
Sensors Connectivity Provider
  |- Validate: ED25519 signature + RWS subscription (Kusama/Polkadot)
  |- Real-time: IPFS pubsub -> Robonomics dApp
  |- Batch: collect -> pin to IPFS (local/Pinata/Crust) -> datalog to chain
  '- Network priority: Kusama (with RWS) -> Kusama -> Polkadot
```

## How It Works

**Validation.** Each incoming message is verified: ED25519 signature check, then RWS (Robonomics Web Services) subscription lookup on both Kusama and Polkadot parachains. Sensor types are routed via `sensors_fabric.py` based on payload fields.

**IPFS publishing.** Validated data is published in real-time to IPFS pubsub topic `airalab.lighthouse.5.robonomics.eth`. Batched data is pinned through up to 4 gateways (local IPFS, Pinata, Crust, Temporal) managed by `pinning_manager.py`.

**Datalog to chain.** Pinned IPFS hashes are written as datalogs to the Robonomics parachain with a network cascade: Kusama with active subscription first, then Kusama without, then Polkadot as fallback.

## Pre-requirements

IPFS daemon with pubsub enabled and Python 3.10+.

Install IPFS (Linux):

```
wget https://dist.ipfs.io/go-ipfs/v0.8.0/go-ipfs_v0.8.0_linux-amd64.tar.gz
tar -xzf go-ipfs_v0.8.0_linux-amd64.tar.gz
cd go-ipfs
sudo bash install.sh
ipfs init
```

## Installation

### From PyPi

```
pip3 install py-sr25519-bindings
pip3 install sensors-connectivity
```

### From Source

Requires [Poetry](https://python-poetry.org/docs/#osx--linux--bashonwindows-install-instructions):

```
curl -sSL https://raw.githubusercontent.com/python-poetry/poetry/master/get-poetry.py | python -
```

Clone and install:

```
git clone https://github.com/airalab/sensors-connectivity
cd sensors-connectivity
poetry install
```

## Configuration

Configuration guide: https://robonomics.academy/en/learn/sensors-connectivity-course/sensors-connectivity-config-options/

When building from source, make a copy of `default.json` and fill it using the guide above.

Logging uses `console` and `file` handlers by default. The logging template is in `connectivity/config/logging_template.py`. Default log path: `~/.logs`. See the [Python logging docs](https://docs.python.org/3.8/library/logging.html) for other handler options.

## Launch

Start the IPFS daemon first:

```
ipfs daemon --enable-pubsub-experiment
```

Then run the service:

```
# PyPi install
sensors_connectivity "path/to/your/config/file"

# From source
poetry run sensors_connectivity "path/to/your/config/file"
```

Logs appear in your terminal (if `console` handler is set) and in `~/.logs`.

## Development

Test the module with an HTTP station:

```
poetry run test_mobile_lab
poetry run test_environmental_box
```

More details in the [`/docs`](https://github.com/airalab/sensors-connectivity/tree/master/docs) directory.

Comprehensive guide on the Decentralized Sensors Network concept: https://robonomics.academy/en/learn/sensors-connectivity-course/overview/

Sensor preparation instructions: https://robonomics.academy/en/learn/sensors-connectivity-course/setting-up-and-connecting-sensors/

## Troubleshooting

**Python.h: No such file or directory**

Install header files and static libraries for python-dev:

```
sudo apt install python3-dev
```

> Note: `python3-dev` does not cover all versions. The service needs at least Python 3.10, so you may need: `sudo apt install python3.10-dev`. See [examples for other package managers](https://stackoverflow.com/a/21530768).

**Python versions mismatch**

If `poetry install` fails with a `SolverProblemError` about Python version incompatibility, specify the version for Poetry:

```
poetry env use python3.10.9
```
