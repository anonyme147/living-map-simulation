from flask import Blueprint, jsonify, current_app

config_bp = Blueprint("config", __name__, url_prefix="/api/config")


@config_bp.get("")
def get_config():
    return jsonify(current_app.config["STARTUP_CONFIG"])
