#!/usr/bin/env python3
import contextlib
import sqlite3
import typing as tp


class DataBase:
    def __init__(self, path_to_db_file: str, table_name) -> None:
        self.path_to_db_file: str = path_to_db_file
        self.table_name: str = table_name

    def connection(self) -> tp.Any:
        db_file = self.path_to_db_file
        try:
            connection = sqlite3.connect(db_file)

        except sqlite3.Error as e:
            print(f"Could not connect to data base {e}")

        return connection
    
    def create_table(self, rows_with_types: str) -> None:
        connection = self.connection()
        cursor = connection.cursor()
        cursor.execute(
            f"CREATE TABLE IF NOT EXISTS {self.table_name} ({rows_with_types});"
        )
    
    def insert_data(self, value_names: str, values: tuple):
        connection = self.connection()
        cursor = connection.cursor()
        values_str = ', '.join(["?"] * len(values))
        with contextlib.closing(connection) as conn:  # auto-closes
            with conn:  # auto-commits
                with contextlib.closing(cursor) as cursor:  # auto-closes
                    cursor.execute(
                        f"INSERT INTO {self.table_name} ({value_names}) VALUES ({values_str})",
                        values,
                    )
    

    def update(self, condition: str, values: tuple) -> None:
        connection = self.connection()
        cursor = connection.cursor()
        with contextlib.closing(connection) as conn:  # auto-closes
            with conn:  # auto-commits
                with contextlib.closing(cursor) as cursor:  # auto-closes
                    cursor.execute(
                        f"UPDATE {self.table_name} {condition}", values
                    )
    
    def get_data(self, values_to_select: str, condifitons: str, condidition_values: tp.Optional[tuple]=None) -> list:
        connection = self.connection()
        cursor = connection.cursor()
        with contextlib.closing(connection) as conn:  # auto-closes
            with conn:  # auto-commits
                with contextlib.closing(cursor) as cursor:  # auto-closes
                    if condidition_values:
                        data_from_table = cursor.execute(
                            f"SELECT {values_to_select} FROM {self.table_name} {condifitons}",
                            condidition_values,
                        ).fetchall()
                    else:
                        data_from_table = cursor.execute(
                            f"SELECT {values_to_select} FROM {self.table_name} {condifitons}",
                        ).fetchall()
        return data_from_table

    def delete_data(self, conditions: str) -> None:
        connection = self.connection()
        cursor = connection.cursor()
        with contextlib.closing(connection) as conn:  # auto-closes
            with conn:  # auto-commits
                with contextlib.closing(cursor) as cursor:  # auto-closes
                    cursor.execute(f"DELETE FROM {self.table_name} {conditions}")
