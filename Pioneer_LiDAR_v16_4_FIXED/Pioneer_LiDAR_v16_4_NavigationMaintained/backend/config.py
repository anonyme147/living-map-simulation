from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent.parent
STATIC_DIR = BASE_DIR / "static"
MAPS_DIR = BASE_DIR / "saved_maps"

HOST = "0.0.0.0"
PORT = 5000
DEBUG = False

LMAP_MAGIC = b"LMAP"
