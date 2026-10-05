from datetime import datetime, timezone
import threading

from flask import Blueprint, jsonify, request


navigation_bp = Blueprint("navigation", __name__, url_prefix="/api/navigation")

_LOCK = threading.RLock()
_TARGET = None
_NEXT_ID = 1


def _target_view():
    with _LOCK:
        return dict(_TARGET) if _TARGET else {"active": False, "target": None, "status": "idle"}


@navigation_bp.get("/target")
def get_target():
    return jsonify(_target_view())


@navigation_bp.post("/target")
def set_target():
    global _TARGET, _NEXT_ID
    data = request.get_json(silent=True) or {}
    try:
        x = float(data["x"])
        y = float(data["y"])
    except (KeyError, TypeError, ValueError):
        return jsonify({"error": "Target requires numeric x and y world coordinates"}), 400

    # Reject NaN/inf without requiring extra dependencies.
    if not (x == x and y == y) or abs(x) == float("inf") or abs(y) == float("inf"):
        return jsonify({"error": "Target x/y must be finite numbers"}), 400

    with _LOCK:
        target_id = _NEXT_ID
        _NEXT_ID += 1
        _TARGET = {
            "active": True,
            "id": target_id,
            "target": {"x": x, "y": y},
            "status": "pending",
            "updated_at": datetime.now(timezone.utc).isoformat(),
            "source": "terminal",
        }
        result = dict(_TARGET)
    return jsonify(result), 200


@navigation_bp.post("/target/status")
def update_target_status():
    global _TARGET
    data = request.get_json(silent=True) or {}
    status = str(data.get("status", "")).strip().lower()
    allowed = {"pending", "running", "blocked", "reached"}
    if status not in allowed:
        return jsonify({"error": f"status must be one of {sorted(allowed)}"}), 400

    with _LOCK:
        if not _TARGET:
            return jsonify({"active": False, "target": None, "status": "idle"}), 200
        requested_id = data.get("id")
        if requested_id is not None and int(requested_id) != int(_TARGET["id"]):
            return jsonify({"error": "target id does not match current target"}), 409
        _TARGET["status"] = status
        _TARGET["updated_at"] = datetime.now(timezone.utc).isoformat()
        if status in {"reached", "blocked"}:
            _TARGET["active"] = False
        result = dict(_TARGET)
    return jsonify(result), 200


@navigation_bp.delete("/target")
def clear_target():
    global _TARGET
    with _LOCK:
        _TARGET = None
    return jsonify({"active": False, "target": None, "status": "idle"}), 200
