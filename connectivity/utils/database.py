#!/usr/bin/env python3
import sqlite3
from sqlite3.dbapi2 import connect
import contextlib
import typing as tp


class DataBase:
    def __init__(self, config: dict) -> None:
        self.config: dict = config

    def connection(self) -> tp.Any:
        db_file = self.config["general"]["db_path"]
        try:
            connection = sqlite3.connect(db_file)

        except sqlite3.Error as e:
            print(f"Could not connect to data base {e}")

        return connection

    def create_table(self) -> None:
        connection = self.connection()
        cursor = connection.cursor()
        cursor.execute(
            """
                        CREATE TABLE IF NOT EXISTS datalog (
                                                    id integer PRIMARY KEY,
                                                    status text,
                                                    hash text,
                                                    time real,
                                                    payload blob

                            ); """
        )

    def add_data(self, status, hash, time, payload) -> None:
        connection = self.connection()
        cursor = connection.cursor()
        with contextlib.closing(connection) as conn:  # auto-closes
            with conn:  # auto-commits
                with contextlib.closing(cursor) as cursor:  # auto-closes
                    cursor.execute(
                        "INSERT INTO datalog (status, hash, time, payload) VALUES (?, ?, ?, ?)",
                        (status, hash, time, payload),
                    )

    def update_status(self, status, hash) -> None:
        connection = self.connection()
        cursor = connection.cursor()
        with contextlib.closing(connection) as conn:  # auto-closes
            with conn:  # auto-commits
                with contextlib.closing(cursor) as cursor:  # auto-closes
                    cursor.execute(
                        "UPDATE datalog SET status = ? WHERE hash = ?", (status, hash)
                    )

    def checker(self, current_time) -> list:
        connection = self.connection()
        cursor = connection.cursor()
        time = current_time - 86400  # 24hrs
        with contextlib.closing(connection) as conn:  # auto-closes
            with conn:  # auto-commits
                with contextlib.closing(cursor) as cursor:  # auto-closes
                    hashes = cursor.execute(
                        "SELECT hash FROM datalog WHERE time < ? AND status='not sent'",
                        (time,),
                    ).fetchall()
                    cursor.execute("DELETE FROM datalog WHERE status='sent'")
        return hashes
