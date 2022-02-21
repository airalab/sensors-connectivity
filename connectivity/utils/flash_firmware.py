#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse
import shutil
import tempfile
import nacl.signing
import os
import sys
import yaml
import logging.config
from config.logging import LOGGING_CONFIG

logging.config.dictConfig(LOGGING_CONFIG)
logger = logging.getLogger("utils")


def write_array(arr: list) -> str:
    buff = ""
    i = 0
    while i < 32:
        buff += arr[i]
        if i != 31:
            buff += ", "

        i += 1
        if i % 8 == 0:
            buff += "\n  "
    return buff


def generate_keys() -> tuple:
    signing_key = nacl.signing.SigningKey.generate()
    signing_array = [int(x) for x in bytes(signing_key)]
    signing_letters = ["0x{0:02X}".format(x) for x in signing_array]

    verify_key = signing_key.verify_key
    verify_array = [int(x) for x in bytes(verify_key)]
    verify_letters = ["0x{0:02X}".format(x) for x in verify_array]

    return signing_letters, verify_letters


def main() -> None:
    parser = argparse.ArgumentParser(description="Prepare and flush ESP firmware")
    parser.add_argument(
        "-s",
        metavar="source",
        type=str,
        default=".",
        help="firmware folder (default to current dir)",
    )
    parser.add_argument(
        "-c",
        metavar="config",
        type=str,
        default="config.yaml",
        help="Path to configuration file",
    )
    parser.add_argument(
        "-p", "--port", metavar="port", type=str, help="Port the board is connected to"
    )

    args = parser.parse_args()

    if args.port:
        port = args.port
    else:
        if sys.platform.startswith("win32"):
            port = "COM1"
        else:
            port = "/dev/ttyUSB0"

    logger.debug(f"Port is {port}")

    with open(args.c) as f:
        settings = yaml.load(f.read(), Loader=yaml.FullLoader)

    ino = os.path.abspath(args.s)
    source_file = os.listdir(os.path.join(ino, "src"))[0]
    with open(os.path.join(ino, "src", source_file), "r") as f:
        firmware = f.read()

    for k, v in settings.items():
        firmware = firmware.replace(k, str(v))

    tempenv = tempfile.TemporaryDirectory()
    logger.debug(f"Temporal directory is created: {tempenv}")

    os.chdir(tempenv.name)
    os.mkdir("src")
    with open(os.path.join("src", source_file), "w") as f:
        f.write(firmware)
        logger.debug(firmware)
        logger.debug("File {} is written".format(os.path.join("src", source_file)))

    os.mkdir("include")

    sk, vk = generate_keys()
    with open(os.path.join("include", "secrets.h"), "w") as f:
        f.write("uint8_t signing_key[32] = {\n  ")
        f.write(write_array(sk))
        f.write(f"}};\n\nuint8_t verifying_key[32] = {{\n  ")
        f.write(write_array(vk))
        f.write("};")

    shutil.copyfile(os.path.join(ino, "platformio.ini"), "platformio.ini")

    os.environ["PLATFORMIO_UPLOAD_PORT"] = port
    if sys.platform.startswith("win32"):
        os.system("python -m platformio run")
        os.system("python -m platformio run -t upload")
    else:
        os.system("python3 -m platformio run")
        os.system("python3 -m platformio run -t upload")

    os.chdir(ino)


if __name__ == "__main__":
    main()
