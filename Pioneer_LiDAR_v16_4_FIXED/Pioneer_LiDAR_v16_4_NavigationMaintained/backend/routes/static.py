from flask import Blueprint, send_from_directory, make_response

from backend.config import STATIC_DIR

static_bp = Blueprint("static", __name__)

def _no_cache(response):
    response.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
    response.headers["Pragma"] = "no-cache"
    response.headers["Expires"] = "0"
    return response

@static_bp.get("/")
def index():
    return _no_cache(make_response(send_from_directory(STATIC_DIR, "index.html")))

@static_bp.get("/static/<path:filename>")
def static_files(filename):
    return _no_cache(make_response(send_from_directory(STATIC_DIR, filename)))
