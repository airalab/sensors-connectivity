import time

from .database import DataBase


class IPFSDB:
    def __init__(self, path_to_db_file: str) -> None:
        self.db_class = DataBase(path_to_db_file, "ipfs")
    
    def create_table(self) -> None:
        self.db_class.create_table("id integer PRIMARY KEY, status text, hash text, timestamp real")

    def add_hash(self, hash: str) -> None:
        status = "pinned"
        timestamp = time.time()
        self.db_class.insert_data("status, hash, timestamp", (status, hash, timestamp))