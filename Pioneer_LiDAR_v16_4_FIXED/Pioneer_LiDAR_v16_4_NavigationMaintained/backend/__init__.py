from flask import Flask
from flask_cors import CORS

from backend.config import MAPS_DIR
from backend.routes.maps import maps_bp
from backend.routes.detections import detections_bp
from backend.routes.static import static_bp
from backend.routes.config import config_bp
from backend.routes.navigation import navigation_bp
from backend.routes.runtime_tests import runtime_tests_bp


def create_app(startup_config=None):
    app = Flask(__name__, static_folder=None)
    CORS(app)

    MAPS_DIR.mkdir(parents=True, exist_ok=True)

    app.register_blueprint(static_bp)
    app.register_blueprint(maps_bp)
    app.register_blueprint(config_bp)
    app.register_blueprint(navigation_bp)
    app.register_blueprint(runtime_tests_bp)
    app.register_blueprint(detections_bp)

    app.config["STARTUP_CONFIG"] = startup_config or {}

    return app
