import threading
import time
from flask import Blueprint, jsonify, request

runtime_tests_bp = Blueprint("runtime_tests", __name__, url_prefix="/api/runtime-tests")
_LOCK = threading.RLock()
_LATEST = None
_HISTORY = []
_MAX_HISTORY = 120


def _copy(obj):
    if isinstance(obj, dict):
        return {k: _copy(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_copy(v) for v in obj]
    return obj


@runtime_tests_bp.post("/telemetry")
def push_telemetry():
    global _LATEST, _HISTORY
    data = request.get_json(silent=True)
    if not isinstance(data, dict):
        return jsonify({"ok": False, "error": "JSON object required"}), 400

    sample = _copy(data)
    sample["server_received_ms"] = int(time.time() * 1000)
    with _LOCK:
        _LATEST = sample
        _HISTORY.append(sample)
        if len(_HISTORY) > _MAX_HISTORY:
            del _HISTORY[:-_MAX_HISTORY]
    return jsonify({"ok": True}), 200


@runtime_tests_bp.get("/status")
def runtime_status():
    with _LOCK:
        latest = _copy(_LATEST)
        history = _copy(_HISTORY[-30:])
    return jsonify({
        "ok": True,
        "latest": latest,
        "history": history,
    }), 200
