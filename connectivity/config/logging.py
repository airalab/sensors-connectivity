import sys
import os

try:
    os.mkdir(f"{os.path.expanduser('~')}/.logs/connectivity")
except FileExistsError:
    pass

LOGGING_CONFIG = {
    "version": 1,
    "formatters": {
        "standart": {"format": "%(asctime)s - %(levelname)s - %(message)s"},
    },
    "handlers": {
        "console": {
            "level": "INFO",
            "formatter": "standart",
            "class": "logging.StreamHandler",
            "stream": sys.stdout,
        },
        "file": {
            "level": "INFO",
            "formatter": "standart",
            "class": "logging.handlers.RotatingFileHandler",
            "filename": f"{os.path.expanduser('~')}/.logs/connectivity/sensors-connectivity.log",
            "maxBytes": 102400000,
            "backupCount": 10,
        },
    },
    "loggers": {
        "sensors-connectivity": {
            "handlers": ["console", "file"],
            "level": "INFO",
            "propagate": False,
        },
    },
}
