#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse
import shutil
import logging
import tempfile
import nacl.signing
import os
import yaml


def write_array(arr: []) -> str:
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
    signing_letters = [ "0x{0:02X}".format(x) for x in signing_array]

    verify_key = signing_key.verify_key
    verify_array = [int(x) for x in bytes(verify_key)]
    verify_letters = [ "0x{0:02X}".format(x) for x in verify_array]

    return signing_letters, verify_letters

def main():
    logging.basicConfig(level=logging.DEBUG)
    parser = argparse.ArgumentParser(description="Prepare and flush ESP firmware")
    parser.add_argument("-s", metavar="source", type=str, default=".", help="firmware folder (default to current dir)")
    parser.add_argument("-c", metavar="config", type=str, help="Path to configuration file")

    args = parser.parse_args()

    config_file = args.c if args.c else "config.yaml"
    with open(args.c) as f:
        settings = yaml.load(f.read(), Loader=yaml.FullLoader)

    geos = settings["geo"].split(",")

    ino = os.path.abspath(args.s)
    with open(ino + "/src/tcp.ino", "r") as f:
        firmware = f.read()

    firmware = firmware.replace("$$STASSID$$", "\"{}\"".format(settings["stassid"])) \
                .replace("$$STAPSK$$", "\"{}\"".format(settings["stapsk"])) \
                .replace("$$HOST$$", "\"{}\"".format(settings["host"])) \
                .replace("$$PORT$$", str(settings["port"])) \
                .replace("$$RXPIN$$", str(settings["rxpin"])) \
                .replace("$$TXPIN$$", str(settings["txpin"])) \
                .replace("$$GEOLAT$$", geos[0]) \
                .replace("$$GEOLON$$", geos[1]) \
                .replace("$$WORKPERIOD$$", str(settings["work_period"])) \

    tempenv = tempfile.TemporaryDirectory()
    logging.debug(f"Temporal directory is created: {tempenv}")

    os.chdir(tempenv.name)
    os.mkdir("src")
    with open("src/firmware.ino", "w") as f:
        f.write(firmware)
        logging.debug(firmware)
        logging.debug("File src/firmware.ino is written")

    os.mkdir("include")

    sk, vk = generate_keys()
    with open("include/secrets.h", "w") as f:
        f.write("uint8_t signing_key[32] = {\n  ")
        f.write(write_array(sk))
        f.write(f"}};\n\nuint8_t verifying_key[32] = {{\n  ")
        f.write(write_array(vk))
        f.write("};")


    shutil.copyfile(ino + "/platformio.ini", "platformio.ini")
    print(os.getcwd())
    os.system("pio run")
    os.system("pio run -t upload")


if __name__ == "__main__":
    main()

