
from .database import DataBase

class DatalogDB:
    def __init__(self, path_to_db_file: str) -> None:
        self.db_class = DataBase(path_to_db_file, "datalog")
    
    def create_table(self) -> None:
        self.db_class.create_table("id integer PRIMARY KEY, status text, hash text, time real, payload blob")
    
    def add_data(self, status, hash, time, payload) -> None:
        self.db_class.insert_data("status, hash, time, payload", (status, hash, time, payload))
    
    def update_status(self, status, hash) -> None:
        self.db_class.update("SET status = ? WHERE hash = ?", (status, hash))

    def checker(self, current_time) -> list:
        time = current_time - 86400  # 24hrs
        hashes = self.db_class.get_data("hash", "WHERE time < ? AND status='not sent'", (time,))
        self.db_class.delete_data("WHERE status='sent'")
        return hashes



