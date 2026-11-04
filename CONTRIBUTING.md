# Contributing to sensors-connectivity

Thanks for helping the sensors.social network. Bug reports, fixes and new sensor
integrations are welcome.

## Before you start

- Search [existing issues](https://github.com/airalab/sensors-connectivity/issues)
  first. For a larger change, open an issue and describe the idea before writing code.
- Security problems: do not open a public issue, see [SECURITY.md](SECURITY.md).

## Development setup

Requirements: Python 3.10+, [Poetry](https://python-poetry.org/), and an IPFS
daemon with pubsub enabled.

```bash
git clone --recurse-submodules https://github.com/airalab/sensors-connectivity.git
cd sensors-connectivity
poetry install
```

Run the emulated stations against a local instance:

```bash
poetry run test_environmental_box
poetry run test_mobile_lab
```

Configuration and architecture are described in [README.md](README.md) and
[`docs/`](docs/).

## Pull requests

- Branch from `master`, keep one topic per pull request.
- Describe what changed and how you tested it; fill in the PR template.
- Merging to `master` builds the package and publishes it to PyPI, so a change
  that should be released also bumps `version` in `pyproject.toml`.
- Do not commit local databases, logs or configs with keys (`connectivity/config/`
  is ignored except for the default template).

## Licence

sensors-connectivity is licensed under the GNU Affero General Public License
v3.0 only (see [LICENSE](LICENSE) and [NOTICE](NOTICE)). By submitting a pull
request you agree that your contribution is licensed under the same terms.
