#!/usr/bin/env python3
import sqlite3
from sqlite3.dbapi2 import connect
import contextlib

class DataBase():
    def __init__(self, config: dict):
        self.config = config
 
    
    def connection(self):
        db_file = self.config["general"]["db_path"]
        try:
            connection = sqlite3.connect(db_file)
        
        except sqlite3.Error as e:
            print(f"Could not connect to data base {e}")

        return connection

    def create_table(self):
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

    def add_data(self, status, hash, time, payload):
        connection = self.connection()
        cursor = connection.cursor()
        with contextlib.closing(connection) as conn: # auto-closes
            with conn: # auto-commits
                with contextlib.closing(cursor) as cursor: # auto-closes
                    cursor.execute("INSERT INTO datalog (status, hash, time, payload) VALUES (?, ?, ?, ?)", (status, hash, time, payload))


    def update_status(self, status, hash):
        connection = self.connection()
        cursor = connection.cursor()
        with contextlib.closing(connection) as conn: # auto-closes
            with conn: # auto-commits
                with contextlib.closing(cursor) as cursor: # auto-closes
                    cursor.execute("UPDATE datalog SET status = ? WHERE hash = ?", (status, hash))


