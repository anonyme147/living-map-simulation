"""
command_post/dashboard.py
Flask web dashboard for the Command Post live tactical map.
Uses 100% Free & Open-Source Leaflet.js with standard OpenStreetMap tiles (Zero API keys required).

Endpoints:
  GET /                  → Live map HTML (Leaflet.js + OpenStreetMap)
  GET /api/config        → Tactical base & disaster zone coordinate configuration
  GET /api/events        → JSON list of all current events
  GET /api/events/geojson → GeoJSON FeatureCollection
  GET /api/summary       → Stats summary
  POST /api/generate_mission → Trigger mission generation
  GET /api/mission/latest → Most recent mission briefing

Auto-refreshes every 3 seconds via JS with no-cache headers.
"""

from __future__ import annotations

import json
import logging
import math
from pathlib import Path
import subprocess
import threading
import time
from typing import Optional, Tuple

from flask import Flask, jsonify, request, make_response, send_file
from beacon.job_compatibility import mission_compatibility_score
from simulation.config import COMMAND_POST_LAT, COMMAND_POST_LON, BUILDING_SITE_LAT, BUILDING_SITE_LON, WRITER_START_LAT, WRITER_START_LON, EXECUTOR_START_LAT, EXECUTOR_START_LON

logger = logging.getLogger(__name__)

# Public, non-project-specific demonstration coordinates.
DEFAULT_CP_LAT = COMMAND_POST_LAT
DEFAULT_CP_LON = COMMAND_POST_LON
DEFAULT_BUILDING_LAT = BUILDING_SITE_LAT
DEFAULT_BUILDING_LON = BUILDING_SITE_LON
DEFAULT_WRITER_LAT = WRITER_START_LAT
DEFAULT_WRITER_LON = WRITER_START_LON
DEFAULT_EXECUTOR_LAT = EXECUTOR_START_LAT
DEFAULT_EXECUTOR_LON = EXECUTOR_START_LON

# Global state set by init_dashboard()
_live_map = None
_mission_generator = None
_last_briefing: Optional[dict] = None
_last_briefing_lock = threading.Lock()
_cp_lat = DEFAULT_CP_LAT
_cp_lon = DEFAULT_CP_LON
_building_lat = DEFAULT_BUILDING_LAT
_building_lon = DEFAULT_BUILDING_LON
_writer_lat = DEFAULT_WRITER_LAT
_writer_lon = DEFAULT_WRITER_LON
_executor_lat = DEFAULT_EXECUTOR_LAT
_executor_lon = DEFAULT_EXECUTOR_LON
_writer_battery = 100.0
_executor_battery = 100.0
_carry_activity_source = None
_executor_dispatch_activity_source = None
_heartbeat_status_source = None
_baseline_alert_source = None
_integrity_ledger_source = None
_webots_visualization_enabled = False
_webots_beacon_source = None
_webots_mission_source = None
_webots_completion_source = None
_webots_world = None
_webots_launch_label = "Pioneer LiDAR visualization"
_webots_process: Optional[subprocess.Popen] = None
_executor_units: dict[str, dict] = {}
_executor_units_lock = threading.RLock()
_selected_executor_unit_id: Optional[str] = None
_primary_executor_unit_id: Optional[str] = None

# ── Authorization Gate ────────────────────────────────────────────────────────
# EXECUTOR-2 must NOT move until the operator clicks "Send Mission".
# _auth_event is set() only when the button is clicked; simulation blocks on it.
_auth_event = threading.Event()          # cleared = waiting; set = authorized
_executor_status = "IDLE"                # IDLE | AWAITING_AUTHORIZATION | EN_ROUTE | COMPLETE
_executor_status_lock = threading.Lock()

# ── Failure Log (populated by /api/failure/* endpoints, cleared per cycle) ────
_failure_log: list = []
_failure_log_lock = threading.Lock()

# ── Cycle Summary (written by push_cycle_summary(), read by /api/cycle_summary) ─
_cycle_summary: dict = {}
_cycle_summary_lock = threading.Lock()

# ── Live Communication Log (read-only observability) ─────────────────────────
_COMMUNICATION_LOG_CAPACITY = 1200
_communication_log: list[dict] = []
_communication_log_lock = threading.RLock()
_communication_log_sequence = 0
_communication_log_handler: Optional[logging.Handler] = None

# The visual companion can optionally submit explicitly-labelled demonstration
# beacons. The active simulation still owns the BeaconStore → ONA pipeline.
_WEBOTS_EXECUTABLE = Path(r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe")
_WEBOTS_WORLD = Path(
    Path(__file__).resolve().parent.parent
    / "Pioneer_LiDAR_v16_4_FIXED"
    / "Pioneer_LiDAR_v16_4_NavigationMaintained"
    / "webots"
    / "worlds"
    / "mavic_2_pro_with_house_scene1_robot1_camera.wbt"
)


def _is_communication_record(record: logging.LogRecord) -> bool:
    """Keep the dashboard stream focused on real pipeline/mission traffic."""
    name = record.name
    message = record.getMessage()
    if name.startswith("outside_network.") or name == "command_post.map_state":
        return True
    if name == "command_post.dashboard":
        return any(token in message for token in ("Operator selected", "Operator authorized", "ONA", "Mission", "Webots"))
    if name == "executor_robot.navigator":
        return any(token in message for token in ("PACKET_RECEIVED", "mission", "Mission", "Arrival", "beacon", "Beacon"))
    if name == "writer_robot.explorer":
        return any(token in message for token in ("Beacon", "beacon", "KILLED"))
    return False


def _communication_source(record: logging.LogRecord) -> str:
    if record.name == "outside_network.receiver":
        return "ONA RECEIVE"
    if record.name == "outside_network.frame_transform":
        return "ONA TRANSLATE"
    if record.name in ("outside_network.lora_uplink", "outside_network.rf_mesh"):
        return "ONA CARRY"
    if record.name == "outside_network.heartbeat":
        return "ONA HEARTBEAT"
    if record.name == "outside_network.mission_relay":
        return "ONA BRIEF"
    if record.name == "command_post.map_state":
        return "COMMAND POST"
    if record.name == "executor_robot.navigator":
        return "EXECUTOR"
    if record.name == "writer_robot.explorer":
        return "WRITER"
    return "COMMAND POST"


class _CommunicationLogHandler(logging.Handler):
    """Copies existing communication logs into a bounded, browser-readable buffer."""

    def emit(self, record: logging.LogRecord) -> None:
        global _communication_log_sequence
        if not _is_communication_record(record):
            return
        try:
            message = record.getMessage()
            with _communication_log_lock:
                _communication_log_sequence += 1
                _communication_log.append({
                    "sequence": _communication_log_sequence,
                    "timestamp": time.strftime("%H:%M:%S", time.localtime(record.created)),
                    "source": _communication_source(record),
                    "level": record.levelname,
                    "message": message,
                })
                if len(_communication_log) > _COMMUNICATION_LOG_CAPACITY:
                    del _communication_log[:-_COMMUNICATION_LOG_CAPACITY]
        except Exception:
            self.handleError(record)


def _reset_communication_log() -> None:
    """Start each simulation cycle with its own communication timeline."""
    with _communication_log_lock:
        _communication_log.clear()


def _ensure_communication_log_handler() -> None:
    """Attach exactly one read-only handler, even across repeated dashboard cycles."""
    global _communication_log_handler
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.INFO)
    if _communication_log_handler is None:
        _communication_log_handler = _CommunicationLogHandler(level=logging.INFO)
        root_logger.addHandler(_communication_log_handler)


def push_cycle_summary(summary: dict) -> None:
    """
    Called by simulation scripts immediately after mission complete.
    Stores the run recap for the dashboard to display as the summary modal.
    Clears the failure log so the next cycle starts fresh.
    Pass an empty dict to clear/reset (e.g. at the start of a new cycle).
    """
    global _failure_log
    with _cycle_summary_lock:
        _cycle_summary.clear()
        _cycle_summary.update(summary)
    if summary.get("mission_id"):
        try:
            from command_post.mission_pdf import export_mission_status
            export_path = export_mission_status(summary)
            if export_path:
                logger.info("[COMMAND POST] Mission status PDF saved: %s", export_path)
        except Exception:
            logger.exception("[COMMAND POST] Mission status PDF export failed")
    with _failure_log_lock:
        _failure_log = []


def haversine_km(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Computes great-circle distance between two GPS coordinates in kilometres."""
    R = 6371.0
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2 +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c


def init_dashboard(
    live_map,
    mission_generator,
    cp_coords: Optional[Tuple[float, float]] = None,
    building_coords: Optional[Tuple[float, float]] = None,
    writer_coords: Optional[Tuple[float, float]] = None,
    executor_coords: Optional[Tuple[float, float]] = None,
    writer=None,
    satellite_driver=None,
    satellite_uplink=None,
    beacon_store=None,
    executor_units: Optional[list[dict]] = None,
    heartbeat_monitor=None,
    baseline_alert=None,
    integrity_ledger=None,
    webots_visualization_enabled: bool = False,
    webots_beacon_source=None,
    webots_mission_source=None,
    webots_completion_source=None,
    webots_world: Optional[Path] = None,
    webots_launch_label: str = "Pioneer LiDAR visualization",
):
    """Initializes the dashboard with live map state and optional geodetic coordinates."""
    global _live_map, _mission_generator, _last_briefing, _executor_status
    global _cp_lat, _cp_lon, _building_lat, _building_lon
    global _writer_lat, _writer_lon, _executor_lat, _executor_lon
    global _writer_battery, _executor_battery, _writer_target, _carry_activity_source, _heartbeat_status_source, _baseline_alert_source, _integrity_ledger_source, _webots_visualization_enabled, _webots_beacon_source, _webots_mission_source, _webots_completion_source, _webots_world, _webots_launch_label
    global _executor_dispatch_activity_source
    global _executor_units, _selected_executor_unit_id, _primary_executor_unit_id

    _ensure_communication_log_handler()
    _reset_communication_log()
    _live_map = live_map
    _mission_generator = mission_generator
    _carry_activity_source = satellite_uplink
    _heartbeat_status_source = heartbeat_monitor
    _baseline_alert_source = baseline_alert
    _integrity_ledger_source = integrity_ledger
    _webots_visualization_enabled = webots_visualization_enabled
    _webots_beacon_source = webots_beacon_source if webots_visualization_enabled else None
    _webots_mission_source = webots_mission_source if webots_visualization_enabled else None
    _webots_completion_source = webots_completion_source if webots_visualization_enabled else None
    _webots_world = webots_world if webots_visualization_enabled else None
    _webots_launch_label = webots_launch_label
    _executor_dispatch_activity_source = getattr(mission_generator, "relay", None)

    # The Flask process survives across demo cycles, so reset every dashboard
    # value that belongs to the previous robots/mission before binding the new
    # cycle's freshly-created map and beacon store.
    with _last_briefing_lock:
        _last_briefing = None
    _auth_event.clear()
    with _executor_status_lock:
        _executor_status = "IDLE"

    _writer_lat, _writer_lon = writer_coords or (DEFAULT_WRITER_LAT, DEFAULT_WRITER_LON)
    unit_definitions = executor_units or [{
        "unit_id": "EXECUTOR-2",
        "display_name": "Executor Robot 2",
        "job_type": "intervention",
        "lat": executor_coords[0] if executor_coords else DEFAULT_EXECUTOR_LAT,
        "lon": executor_coords[1] if executor_coords else DEFAULT_EXECUTOR_LON,
        "battery": 100.0,
        "status": "AVAILABLE",
        "icon_url": "",
        "icon_tint": "#2563eb",
    }]
    with _executor_units_lock:
        _executor_units = {
            unit["unit_id"]: {
                **unit,
                "battery": float(unit.get("battery", 100.0)),
                "status": unit.get("status", "AVAILABLE"),
            }
            for unit in unit_definitions
        }
        _primary_executor_unit_id = next(iter(_executor_units), None)
        _selected_executor_unit_id = None
        primary_unit = _executor_units.get(_primary_executor_unit_id, {})
        _executor_lat = primary_unit.get("lat", DEFAULT_EXECUTOR_LAT)
        _executor_lon = primary_unit.get("lon", DEFAULT_EXECUTOR_LON)
        _executor_battery = primary_unit.get("battery", 100.0)
    _writer_battery = 100.0

    # A WriterRobot is created after dashboard initialization. Do not leave the
    # failure controls bound to the previous cycle's robot during that window.
    _writer_target = None

    if cp_coords:
        _cp_lat, _cp_lon = cp_coords
    if building_coords:
        _building_lat, _building_lon = building_coords
    register_failure_targets(writer=writer, satellite_driver=satellite_driver, beacon_store=beacon_store)


def set_rover_coords(
    writer_coords: Optional[Tuple[float, float]] = None,
    executor_coords: Optional[Tuple[float, float]] = None,
    writer_battery: Optional[float] = None,
    executor_battery: Optional[float] = None,
) -> None:
    """Dynamically updates rover positions on the tactical map."""
    global _writer_lat, _writer_lon, _executor_lat, _executor_lon, _writer_battery, _executor_battery
    if writer_coords:
        _writer_lat, _writer_lon = writer_coords
    if executor_coords:
        _executor_lat, _executor_lon = executor_coords
    if writer_battery is not None:
        _writer_battery = max(0.0, min(100.0, writer_battery))
    if executor_battery is not None:
        _executor_battery = max(0.0, min(100.0, executor_battery))


def set_executor_unit_state(
    unit_id: str,
    *,
    executor_coords: Optional[Tuple[float, float]] = None,
    battery: Optional[float] = None,
    status: Optional[str] = None,
) -> None:
    """Update one unit independently while preserving legacy selected-unit telemetry."""
    global _executor_lat, _executor_lon, _executor_battery
    with _executor_units_lock:
        unit = _executor_units.get(unit_id)
        if unit is None:
            raise KeyError(f"Unknown Executor unit: {unit_id}")
        if executor_coords is not None:
            unit["lat"], unit["lon"] = executor_coords
        if battery is not None:
            unit["battery"] = max(0.0, min(100.0, battery))
        if status is not None:
            unit["status"] = status
        if unit_id == (_selected_executor_unit_id or _primary_executor_unit_id):
            _executor_lat = unit["lat"]
            _executor_lon = unit["lon"]
            _executor_battery = unit["battery"]


def get_selected_executor_unit_id() -> Optional[str]:
    with _executor_units_lock:
        return _selected_executor_unit_id


def set_executor_mission_status(status: str) -> None:
    """Publish the selected unit's terminal mission state to legacy dashboard UI."""
    global _executor_status
    with _executor_status_lock:
        _executor_status = status


def _executor_units_payload() -> list[dict]:
    with _last_briefing_lock:
        briefing = dict(_last_briefing) if _last_briefing else None
    event_types = [
        (waypoint.get("event_type", ""), waypoint.get("severity", 1.0))
        for waypoint in briefing.get("waypoints", [])
    ] if briefing else []
    with _executor_units_lock:
        units = []
        for unit in _executor_units.values():
            payload = dict(unit)
            payload["compatibility_score"] = mission_compatibility_score(event_types, payload["job_type"])
            payload["selected"] = payload["unit_id"] == _selected_executor_unit_id
            units.append(payload)
    return sorted(units, key=lambda unit: (-unit["compatibility_score"], unit["unit_id"]))


def _display_executor_unit() -> dict:
    with _executor_units_lock:
        unit_id = _selected_executor_unit_id or _primary_executor_unit_id
        return dict(_executor_units[unit_id]) if unit_id else {}


app = Flask(__name__)
app.config["JSON_SORT_KEYS"] = False


# ── HTML Dashboard ────────────────────────────────────────────────────────────

@app.route("/")
def index():
    html = _build_dashboard_html(
        _cp_lat, _cp_lon, _building_lat, _building_lon,
        _writer_lat, _writer_lon, _executor_lat, _executor_lon,
        json.dumps(_executor_units_payload()),
        _webots_visualization_enabled,
    )
    resp = make_response(html)
    resp.headers["Content-Type"] = "text/html; charset=utf-8"
    # Prevent browser caching so updates render immediately without manual clear
    resp.headers["Cache-Control"] = "no-cache, no-store, must-revalidate, max-age=0"
    resp.headers["Pragma"] = "no-cache"
    resp.headers["Expires"] = "0"
    return resp


@app.route("/communications")
def communications_console():
    """Open the detailed, terminal-style communications console in a browser tab."""
    resp = make_response(_build_communications_console_html())
    resp.headers["Content-Type"] = "text/html; charset=utf-8"
    resp.headers["Cache-Control"] = "no-cache, no-store, must-revalidate, max-age=0"
    resp.headers["Pragma"] = "no-cache"
    resp.headers["Expires"] = "0"
    return resp


# ── API Endpoints ─────────────────────────────────────────────────────────────

@app.route("/api/config")
def api_config():
    executor = _display_executor_unit()
    dist_km = round(haversine_km(_building_lat, _building_lon, _cp_lat, _cp_lon), 2)
    rover_sep_km = round(haversine_km(
        _writer_lat,
        _writer_lon,
        executor.get("lat", _executor_lat),
        executor.get("lon", _executor_lon),
    ), 2)
    return jsonify({
        "command_post": {
            "name": "Incident Command Post HQ",
            "lat": _cp_lat,
            "lon": _cp_lon,
            "role": "Tactical Base of Operations & Satellite Telemetry Gateway",
        },
        "building_site": {
            "name": "Disaster Zone Alpha (Collapsed Structure)",
            "lat": _building_lat,
            "lon": _building_lon,
            "role": "GPS-Denied Exploration Site",
        },
        "writer_robot": {
            "id": "WRITER-1",
            "name": "Writer Robot 1 (Scout)",
            "lat": _writer_lat,
            "lon": _writer_lon,
            "battery": round(_writer_battery, 1),
            "battery": round(_writer_battery, 1),
            "role": "On-Scene In-Situ Sensor Exploration (at event location)",
        },
        "executor_robot": {
            "id": executor.get("unit_id", "EXECUTOR-2"),
            "name": executor.get("display_name", "Executor Robot 2 (Intervention)"),
            "lat": executor.get("lat", _executor_lat),
            "lon": executor.get("lon", _executor_lon),
            "battery": round(executor.get("battery", _executor_battery), 1),
            "role": executor.get("job_type", "intervention"),
        },
        "distance_km": dist_km,
        "rover_separation_km": rover_sep_km,
        "map_engine": "Leaflet.js v1.9.4 + OpenStreetMap Standard (Open-Source, Zero API Keys)",
    })


@app.route("/api/rovers")
def api_rovers():
    executor = _display_executor_unit()
    rover_sep_km = round(haversine_km(_writer_lat, _writer_lon, executor.get("lat", _executor_lat), executor.get("lon", _executor_lon)), 2)
    return jsonify({
        "writer": {
            "id": "WRITER-1",
            "name": "Writer Robot 1",
            "role": "Scout (On Scene)",
            "lat": _writer_lat,
            "lon": _writer_lon,
            "battery": round(_writer_battery, 1),
        },
        "executor": {
            "id": executor.get("unit_id", "EXECUTOR-2"),
            "name": executor.get("display_name", "Executor Robot 2"),
            "role": executor.get("job_type", "intervention"),
            "lat": executor.get("lat", _executor_lat),
            "lon": executor.get("lon", _executor_lon),
            "battery": round(executor.get("battery", _executor_battery), 1),
        },
        "separation_km": rover_sep_km,
    })


@app.route("/api/executor/units")
def api_executor_units():
    """Return every dispatchable unit with independent state and mission fit."""
    return jsonify({
        "units": _executor_units_payload(),
        "selected_unit_id": get_selected_executor_unit_id(),
    })


@app.route("/api/executor/select", methods=["POST"])
def api_select_executor_unit():
    """Store the operator's unit selection before ONA authorization."""
    global _selected_executor_unit_id, _executor_lat, _executor_lon, _executor_battery
    payload = request.get_json(silent=True) or {}
    unit_id = payload.get("unit_id")
    if not isinstance(unit_id, str):
        return jsonify({"error": "unit_id is required"}), 400
    with _executor_status_lock:
        if _executor_status != "AWAITING_AUTHORIZATION":
            return jsonify({"error": "Select a unit after preparing a mission and before authorization"}), 409
    with _executor_units_lock:
        unit = _executor_units.get(unit_id)
        if unit is None:
            return jsonify({"error": f"Unknown Executor unit: {unit_id}"}), 404
        if _selected_executor_unit_id and _selected_executor_unit_id in _executor_units:
            _executor_units[_selected_executor_unit_id]["status"] = "AVAILABLE"
        _selected_executor_unit_id = unit_id
        unit["status"] = "SELECTED"
        _executor_lat, _executor_lon = unit["lat"], unit["lon"]
        _executor_battery = unit["battery"]
    logger.info("[COMMAND POST] Operator selected Executor unit %s", unit_id)
    return jsonify({"status": "selected", "unit_id": unit_id})


@app.route("/api/signal_strength")
def api_signal_strength():
    """
    Live link margin combines current Writer-to-ONA distance with satellite
    pass/weather state. The exponential distance term is deliberately gentle for
    satellite service, while still reflecting the additional ground-terminal cost.
    """
    import time as _t
    link_quality = (
        _satellite_target.link_quality()
        if _satellite_target is not None and hasattr(_satellite_target, "link_quality")
        else None
    )
    satellite_down = _satellite_target is not None and _satellite_target.link_disrupted
    if satellite_down:
        strength = 0.0
        link_state = "blackout"
    else:
        dropout = _satellite_target.packet_loss_prob if _satellite_target is not None else 0.02
        distance_km = haversine_km(_writer_lat, _writer_lon, _cp_lat, _cp_lon)
        distance_margin = 100.0 * math.exp(-distance_km / 25.0)
        strength = round(max(0.0, distance_margin * (1.0 - dropout)), 1)
        link_state = "nominal" if dropout <= 0.03 else "degraded"

    if strength > 66:
        zone = "green"
    elif strength > 33:
        zone = "yellow"
    else:
        zone = "red"

    ts = _t.time()
    ts_str = _t.strftime("%H:%M:%S", _t.localtime(ts)) + f".{int((ts%1)*1000):03d}"

    return jsonify({
        "strength": round(strength, 1),
        "link_quality": link_quality,
        "zone": zone,
        "link_state": link_state,
        "writer_lat": _writer_lat,
        "writer_lon": _writer_lon,
        "ona_lat": _cp_lat,
        "ona_lon": _cp_lon,
        "dist_km": round(haversine_km(_writer_lat, _writer_lon, _cp_lat, _cp_lon), 3),
        "ts": ts_str,
    })


@app.route("/api/heartbeat")
def api_heartbeat():
    """Return the optional ONA↔Command Post trust-heartbeat state."""
    source = _heartbeat_status_source
    if source is None or not hasattr(source, "status"):
        return jsonify({"enabled": False})
    try:
        return jsonify(source.status())
    except Exception:
        logger.exception("[COMMAND POST] Unable to read heartbeat status")
        return jsonify({"enabled": False}), 503


@app.route("/api/signal_integrity")
def api_signal_integrity():
    """Return showcase-only end-to-end payload integrity evidence."""
    source = _integrity_ledger_source
    if source is None or not hasattr(source, "snapshot"):
        return jsonify({"enabled": False})
    try:
        return jsonify(source.snapshot())
    except Exception:
        logger.exception("[COMMAND POST] Unable to read signal-integrity ledger")
        return jsonify({"enabled": False}), 503


@app.route("/api/baseline_alert")
def api_baseline_alert():
    """Return an optional persistent alert for the intentionally unsolved baselines."""
    source = _baseline_alert_source
    if source is None or not hasattr(source, "status"):
        return jsonify({"enabled": False, "active": False})
    return jsonify(source.status())


def _webots_is_running() -> bool:
    """Check the tracked process and a manually started Webots instance."""
    if _webots_process is not None and _webots_process.poll() is None:
        return True
    try:
        result = subprocess.run(
            ["tasklist", "/FI", "IMAGENAME eq webots.exe", "/FO", "CSV", "/NH"],
            capture_output=True,
            text=True,
            timeout=3,
            check=False,
        )
        return "webots.exe" in result.stdout.lower()
    except (OSError, subprocess.SubprocessError):
        return False


@app.route("/api/launch/webots_visualization", methods=["POST"])
def api_launch_webots_visualization():
    """Launch the optional Pioneer Webots companion without touching simulation state."""
    global _webots_process

    if not _webots_visualization_enabled:
        return jsonify({"status": "unavailable", "message": "Webots visualization is available only in the Webotsim live mission."}), 403
    if _webots_is_running():
        return jsonify({"status": "already_running", "message": "Webots visualization is already running."})
    if not _WEBOTS_EXECUTABLE.is_file():
        return jsonify({"status": "error", "message": f"Webots launcher not found: {_WEBOTS_EXECUTABLE}"}), 500
    world = _webots_world or _WEBOTS_WORLD
    if not world.is_file():
        return jsonify({"status": "error", "message": f"Webots world not found: {world}"}), 500

    try:
        _webots_process = subprocess.Popen(
            [
                str(_WEBOTS_EXECUTABLE),
                "--mode=realtime",
                str(world),
            ],
            cwd=str(world.parent.parent),
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            creationflags=getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0),
        )
    except OSError as exc:
        logger.exception("[COMMAND POST] Webots visualization launch failed")
        return jsonify({"status": "error", "message": f"Unable to launch Webots: {exc}"}), 500

    logger.info("[COMMAND POST] Webots launched: %s", _webots_launch_label)
    return jsonify({
        "status": "launching",
        "message": f"Launching {_webots_launch_label} in Webots.",
    }), 202


@app.route("/webots_vision")
def webots_vision():
    """Open the visual-only Pioneer front-camera and detection monitor."""
    if not _webots_visualization_enabled:
        return "Webots vision is available only in the Webotsim live mission.", 403
    response = make_response(_build_webots_vision_html())
    response.headers["Content-Type"] = "text/html; charset=utf-8"
    response.headers["Cache-Control"] = "no-cache, no-store, must-revalidate, max-age=0"
    return response


@app.route("/api/carry_transmission")
def api_carry_transmission():
    """Read the current ONA Role 3 CARRY attempt for the transient map visual."""
    source = _carry_activity_source
    if source is None or not hasattr(source, "transmission_activity"):
        return jsonify({"active": False})
    try:
        return jsonify(source.transmission_activity())
    except Exception:
        logger.exception("[COMMAND POST] Unable to read CARRY transmission activity")
        return jsonify({"active": False}), 503


@app.route("/api/rf_mesh")
def api_rf_mesh():
    """Expose read-only RF-mesh topology and per-hop CARRY activity when present."""
    source = _carry_activity_source
    if source is None or not hasattr(source, "mesh_snapshot"):
        return jsonify({"enabled": False, "nodes": [], "links": [], "activity": {"active": False}})
    try:
        return jsonify(source.mesh_snapshot())
    except Exception:
        logger.exception("[COMMAND POST] Unable to read RF mesh activity")
        return jsonify({"enabled": False, "nodes": [], "links": [], "activity": {"active": False}}), 503


@app.route("/api/executor/dispatch_transmissions")
def api_executor_dispatch_transmissions():
    """Read active ONA Role 4 downlinks for the selected-unit map visual."""
    source = _executor_dispatch_activity_source
    if source is None or not hasattr(source, "dispatch_activity"):
        return jsonify({"active": False, "transmissions": []})
    try:
        return jsonify(source.dispatch_activity())
    except Exception:
        logger.exception("[COMMAND POST] Unable to read Executor dispatch activity")
        return jsonify({"active": False, "transmissions": []}), 503


@app.route("/api/executor/position")
def api_executor_position():
    """Lightweight endpoint used to animate the selected unit's marker."""
    return jsonify({"lat": _executor_lat, "lon": _executor_lon})


@app.route("/api/events")
def api_events():
    if _live_map is None:
        return jsonify({"error": "Map not initialized"}), 503
    events = [e.to_dict() for e in _live_map.get_events()]
    return jsonify({"events": events, "count": len(events)})


@app.route("/api/webots/beacon", methods=["POST"])
def api_webots_beacon():
    """Accept a local Webots sensor event and hand it to the active beacon pipeline."""
    source = _webots_beacon_source
    if source is None:
        return jsonify({"error": "Webots beacon bridge is inactive for this demo"}), 503
    payload = request.get_json(silent=True)
    if not isinstance(payload, dict):
        return jsonify({"error": "Expected a JSON beacon payload"}), 400
    try:
        result = source(payload)
    except (TypeError, ValueError) as exc:
        return jsonify({"error": str(exc)}), 400
    except Exception:
        logger.exception("[Webots] Beacon bridge failed")
        return jsonify({"error": "Webots beacon bridge failed"}), 500
    logger.info(
        "[Webots] Beacon accepted: %s (%s) → Beacon Network → ONA",
        result["beacon_id"], result["event_type"],
    )
    return jsonify({"status": "queued_for_ona", **result}), 202


@app.route("/api/webots/executor/mission")
def api_webots_executor_mission():
    """Expose only an already-delivered ONA Role 4 briefing to the Webots unit."""
    source = _webots_mission_source
    if source is None:
        return jsonify({"error": "Webots Executor bridge is inactive for this demo"}), 503
    try:
        return jsonify(source())
    except Exception:
        logger.exception("[Webots] Unable to read ONA Role 4 mission")
        return jsonify({"error": "ONA mission bridge failed"}), 500


@app.route("/api/webots/executor/completion", methods=["POST"])
def api_webots_executor_completion():
    """Accept a physical completion beacon from the Webots Executor adapter."""
    source = _webots_completion_source
    if source is None:
        return jsonify({"error": "Webots Executor bridge is inactive for this demo"}), 503
    payload = request.get_json(silent=True)
    if not isinstance(payload, dict):
        return jsonify({"error": "Expected a JSON completion payload"}), 400
    try:
        result = source(payload)
    except (TypeError, ValueError) as exc:
        return jsonify({"error": str(exc)}), 400
    except Exception:
        logger.exception("[Webots] Completion beacon bridge failed")
        return jsonify({"error": "Webots completion bridge failed"}), 500
    logger.info("[Webots Executor] mission_completed beacon accepted: %s", result["beacon_id"])
    return jsonify({"status": "queued_for_ona", **result}), 202


@app.route("/api/communications")
def api_communications():
    """Return the dashboard's bounded, read-only communication timeline."""
    try:
        after = max(0, int(request.args.get("after", "0")))
    except ValueError:
        return jsonify({"error": "after must be an integer"}), 400
    try:
        limit = min(500, max(1, int(request.args.get("limit", "500"))))
    except ValueError:
        return jsonify({"error": "limit must be an integer"}), 400

    with _communication_log_lock:
        events = [dict(event) for event in _communication_log if event["sequence"] > after]
        events = events[-limit:]
        latest_sequence = _communication_log_sequence
    return jsonify({"events": events, "latest_sequence": latest_sequence})

_writer_target = None
_satellite_target = None
_beacon_store_target = None


def register_failure_targets(writer=None, satellite_driver=None, beacon_store=None):
    global _writer_target, _satellite_target, _beacon_store_target
    if writer is not None:
        _writer_target = writer
    if satellite_driver is not None:
        _satellite_target = satellite_driver
    if beacon_store is not None:
        _beacon_store_target = beacon_store


@app.route("/api/failure/writer", methods=["POST"])
def trigger_writer_failure():
    """Inject catastrophic writer robot loss via the running demo."""
    import time as _t
    target = _writer_target
    if target is None:
        try:
            from simulation.run_failure_demo import writer_instance
            target = writer_instance
        except (ImportError, AttributeError):
            pass
    if target is not None:
        if hasattr(target, "kill"):
            target.kill()
        with _failure_log_lock:
            _failure_log.append({
                "type": "writer_killed",
                "label": "Writer Robot Killed",
                "timestamp": _t.time(),
                "ts_str": _t.strftime("%H:%M:%S", _t.localtime()),
                "detail": "Operator-triggered via dashboard sidebar",
            })
        return jsonify({"status": "writer killed"}), 200
    return jsonify({"error": "writer not available"}), 500


@app.route("/api/failure/satellite", methods=["POST"])
def trigger_satellite_failure():
    """Simulate a total satellite pass/weather disruption."""
    import time as _t
    target = _satellite_target
    if target is None:
        try:
            from simulation.run_failure_demo import satellite_driver_instance
            target = satellite_driver_instance
        except (ImportError, AttributeError):
            pass
    if target is not None:
        target.link_disrupted = True
        with _failure_log_lock:
            _failure_log.append({
                "type": "satellite_disrupted",
                "label": "Satellite Link Disrupted",
                "timestamp": _t.time(),
                "ts_str": _t.strftime("%H:%M:%S", _t.localtime()),
                "detail": "Operator-triggered satellite disruption; store-and-forward buffering engaged",
            })
        return jsonify({"status": "satellite disruption set"}), 200
    return jsonify({"error": "satellite driver not available"}), 500


@app.route("/api/failure/beacon", methods=["POST"])
def trigger_beacon_failure():
    """Simulate beacon TTL expiration on all active beacons."""
    import time as _t
    target = _beacon_store_target
    if target is None:
        try:
            from simulation.run_failure_demo import beacon_store_instance
            target = beacon_store_instance
        except (ImportError, AttributeError):
            pass
    if target is not None:
        count = target.expire_all()
        with _failure_log_lock:
            _failure_log.append({
                "type": "beacon_expired",
                "label": f"Beacon TTL Expired ({count} beacon(s))",
                "timestamp": _t.time(),
                "ts_str": _t.strftime("%H:%M:%S", _t.localtime()),
                "detail": f"{count} active beacon(s) force-expired — Executor skip-logic activated",
            })
        return jsonify({"status": "beacons expired", "count": count}), 200
    return jsonify({"error": "beacon store not available"}), 500


@app.route("/api/failure/stacked", methods=["POST"])
def trigger_stacked_failure():
    """Inject all three failures simultaneously (stacked combined failure)."""
    import time as _t
    results = {}
    ts = _t.time()
    ts_str = _t.strftime("%H:%M:%S", _t.localtime(ts))

    if _writer_target is not None and hasattr(_writer_target, "kill"):
        _writer_target.kill()
        results["writer"] = "killed"
    if _satellite_target is not None:
        _satellite_target.link_disrupted = True
        results["satellite"] = "disrupted"
    if _beacon_store_target is not None:
        count = _beacon_store_target.expire_all()
        results["beacons"] = f"{count} expired"

    with _failure_log_lock:
        _failure_log.append({
            "type": "stacked",
            "label": "Stacked Failure (Writer + Satellite + Beacon TTL)",
            "timestamp": ts,
            "ts_str": ts_str,
            "detail": f"Combined injection: writer killed, satellite link disrupted, beacon TTL forced — results: {results}",
        })
    return jsonify({"status": "stacked failure injected", "results": results}), 200


@app.route("/api/events/geojson")
def api_geojson():
    if _live_map is None:
        return jsonify({}), 503
    return jsonify(_live_map.to_geojson())


@app.route("/api/summary")
def api_summary():
    if _live_map is None:
        return jsonify({"error": "Map not initialized"}), 503
    return jsonify(_live_map.get_summary())


@app.route("/api/cycle_summary")
def api_cycle_summary():
    """Returns the latest mission-cycle summary for the dashboard modal."""
    with _cycle_summary_lock:
        return jsonify(dict(_cycle_summary))


@app.route("/api/export_mission_pdf")
def api_export_mission_pdf():
    """Generates and serves the mission PDF report on demand."""
    with _cycle_summary_lock:
        summary = dict(_cycle_summary)

    req_mission_id = request.args.get("mission_id")
    if req_mission_id and req_mission_id.strip():
        summary["mission_id"] = req_mission_id.strip()

    if not summary or not summary.get("mission_id"):
        return jsonify({"error": "No completed mission summary available for PDF export"}), 404

    try:
        from command_post.mission_pdf import export_mission_status
        pdf_path = export_mission_status(summary, force=True)
        if pdf_path and pdf_path.exists():
            safe_id = "".join(c if c.isalnum() or c in "-_" else "_" for c in str(summary.get("mission_id", "mission")))
            return send_file(
                str(pdf_path),
                mimetype="application/pdf",
                as_attachment=True,
                download_name=f"{safe_id}_mission_report.pdf",
            )
        return jsonify({"error": "PDF generation failed"}), 500
    except Exception as e:
        logger.exception("[COMMAND POST] On-demand PDF export failed")
        return jsonify({"error": str(e)}), 500



@app.route("/api/generate_mission", methods=["POST"])
def api_generate_mission():
    global _last_briefing
    if _mission_generator is None:
        return jsonify({"error": "Generator not initialized"}), 503
    briefing = _mission_generator.generate()
    if briefing is None:
        return jsonify({"error": "No active events to brief"}), 400
    bd = briefing.to_dict()
    with _last_briefing_lock:
        _last_briefing = bd
    return jsonify({"status": "sent", "briefing": bd})


@app.route("/api/mission/latest")
def api_latest_mission():
    with _last_briefing_lock:
        with _executor_status_lock:
            status = _executor_status
        if _last_briefing is None:
            return jsonify({"briefing": None, "message": "No mission generated yet", "executor_status": status, "selected_unit_id": get_selected_executor_unit_id()}), 200
        return jsonify({"briefing": _last_briefing, "executor_status": status, "selected_unit_id": get_selected_executor_unit_id()})


@app.route("/api/mission/authorize", methods=["POST"])
def api_authorize_mission():
    """Operator click routes authorization to ONA; Role 4 then briefs Executor."""
    global _executor_status
    with _last_briefing_lock:
        briefing = dict(_last_briefing) if _last_briefing is not None else None
    if briefing is None:
        return jsonify({"error": "No prepared mission briefing to authorize"}), 409
    selected_unit_id = get_selected_executor_unit_id()
    if selected_unit_id is None:
        return jsonify({"error": "Select an Executor unit before authorizing the mission"}), 409
    with _executor_status_lock:
        if _executor_status != "AWAITING_AUTHORIZATION":
            return jsonify({"error": f"Cannot authorize: status is '{_executor_status}' (must be AWAITING_AUTHORIZATION)"}), 409
    logger.info(
        "[COMMAND POST] Operator authorized %s for %s; sending authorization to ONA",
        briefing.get("briefing_id"), selected_unit_id,
    )
    try:
        relay_id = _mission_generator.dispatch_authorized(briefing, selected_unit_id)
    except Exception:
        logger.exception("[COMMAND POST] ONA authorization relay failed")
        return jsonify({"error": "ONA could not deliver the authorized mission"}), 503
    with _executor_status_lock:
        _executor_status = "EN_ROUTE"
    set_executor_unit_state(selected_unit_id, status="EN_ROUTE")
    _auth_event.set()   # unblock wait_for_authorization()
    return jsonify({"status": "authorized", "relay_id": relay_id, "executor_status": "EN_ROUTE", "unit_id": selected_unit_id})


@app.route("/api/executor/status")
def api_executor_status():
    """Returns current Executor-2 authorization/navigation status."""
    with _executor_status_lock:
        return jsonify({"executor_status": _executor_status})


def set_latest_briefing(bd: dict) -> None:
    """Called by simulation scripts to store the latest briefing for display."""
    global _last_briefing
    with _last_briefing_lock:
        _last_briefing = bd


def set_awaiting_authorization() -> None:
    """
    Called after a briefing is delivered. Puts Executor into AWAITING_AUTHORIZATION:
    clears the auth event (blocks navigation) and updates the status label.
    """
    global _executor_status, _selected_executor_unit_id
    _auth_event.clear()          # block any waiting wait_for_authorization() call
    with _executor_status_lock:
        _executor_status = "AWAITING_AUTHORIZATION"
    with _executor_units_lock:
        _selected_executor_unit_id = None
        for unit in _executor_units.values():
            unit["status"] = "AVAILABLE"
    logger.info("[COMMAND POST] Executor status set to AWAITING_AUTHORIZATION")


def wait_for_authorization(timeout_s: float = 300.0) -> bool:
    """
    Blocks the calling thread until the operator clicks the Send Mission button
    (or until timeout_s elapses).
    Returns True if authorized, False if timed out.
    """
    authorized = _auth_event.wait(timeout=timeout_s)
    return authorized


# ── HTML Builder ──────────────────────────────────────────────────────────────

def _build_dashboard_html(
    cp_lat: float, cp_lon: float, building_lat: float, building_lon: float,
    writer_lat: float, writer_lon: float, executor_lat: float, executor_lon: float,
    executor_units_json: str,
    webots_visualization_enabled: bool = False,
) -> str:
    distance_km = round(haversine_km(building_lat, building_lon, cp_lat, cp_lon), 2)
    rover_sep_km = round(haversine_km(writer_lat, writer_lon, executor_lat, executor_lon), 2)
    webots_button_html = f"""
    <button class=\"webots-visualization-btn\" id=\"webots-visualization-btn\" type=\"button\" onclick=\"launchWebotsVisualization()\" title=\"Open {_webots_launch_label}\">
      <span class=\"webots-status-dot\"></span> LAUNCH {_webots_launch_label.upper()}
    </button>""" if webots_visualization_enabled else ""
    webots_vision_button_html = """
    <button class=\"webots-vision-btn\" type=\"button\" onclick=\"openWebotsVision()\" title=\"Open the Pioneer front-camera and target-detection monitor\">
      ◉ OPEN ROBOT CAMERA
    </button>""" if webots_visualization_enabled else ""
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <meta http-equiv="Cache-Control" content="no-cache, no-store, must-revalidate">
  <meta http-equiv="Pragma" content="no-cache">
  <meta http-equiv="Expires" content="0">
  <title>The Living Map — Command Post Tactical Display (OpenStreetMap)</title>
  <link rel="preconnect" href="https://fonts.googleapis.com">
  <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
  <link href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:ital,wght@0,400;0,500;0,600;0,700;1,400&family=IBM+Plex+Sans:wght@400;500;600;700&family=Rajdhani:wght@500;600;700&display=swap" rel="stylesheet">
  <!-- Open-Source Leaflet.js + CSS -->
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" integrity="sha256-p4NxAoJBhIIN+hmNHrzRCf9tD/miZyoHS5obTRR9BMY=" crossorigin="" />
  <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js" integrity="sha256-20nQCchB9co0qIjJZRGuk2/Z9VM+kNiyxNV1lvTlZBo=" crossorigin=""></script>
  <style>
    :root {{
      --ops-bg: #05080c;
      --ops-rack: #090e15;
      --ops-panel: #0d141f;
      --ops-panel-subtle: #111a28;
      --ops-panel-elevated: #152233;
      --ops-border: #1a2838;
      --ops-border-strong: #25394f;
      --ops-border-bright: #38bdf8;
      --ops-cyan: #06b6d4;
      --ops-cyan-glow: rgba(6, 182, 212, 0.4);

      --ops-text: #f8fafc;
      --ops-text-muted: #94a3b8;
      --ops-text-dim: #64748b;

      --status-nominal: #10b981;
      --status-nominal-bg: rgba(16, 185, 129, 0.12);
      --status-nominal-border: rgba(16, 185, 129, 0.35);

      --status-warning: #f59e0b;
      --status-warning-bg: rgba(245, 158, 11, 0.12);
      --status-warning-border: rgba(245, 158, 11, 0.35);

      --status-critical: #ef4444;
      --status-critical-bg: rgba(239, 68, 68, 0.14);
      --status-critical-border: rgba(239, 68, 68, 0.4);

      --status-action: #06b6d4;
      --status-action-hover: #22d3ee;
      --status-action-bg: rgba(6, 182, 212, 0.12);

      --font-display: 'Rajdhani', 'Chakra Petch', 'Trebuchet MS', 'Arial Black', sans-serif;
      --font-mono: 'IBM Plex Mono', 'Consolas', 'Courier New', monospace;
      --font-sans: 'IBM Plex Sans', -apple-system, BlinkMacSystemFont, sans-serif;
    }}

    * {{ margin: 0; padding: 0; box-sizing: border-box; }}

    html, body {{
      height: 100vh;
      width: 100vw;
      margin: 0;
      padding: 0;
      overflow: hidden;
      background: var(--ops-bg);
      color: var(--ops-text);
      font-family: var(--font-sans);
      -webkit-font-smoothing: antialiased;
      -moz-osx-font-smoothing: grayscale;
    }}

    body {{
      display: flex;
      flex-direction: column;
    }}

    /* ── Top Military Classification & Command Bar ── */
    header {{
      height: 54px;
      min-height: 54px;
      background: var(--ops-rack);
      border-bottom: 2px solid var(--ops-border-strong);
      padding: 0 16px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      flex-shrink: 0;
      z-index: 1000;
      position: relative;
    }}
    header::after {{
      content: '';
      position: absolute;
      bottom: -2px;
      left: 0;
      right: 0;
      height: 2px;
      background: linear-gradient(90deg, #06b6d4 0%, #1a2838 25%, #25394f 75%, #06b6d4 100%);
      pointer-events: none;
    }}
    .header-brand {{
      display: flex;
      align-items: center;
      gap: 12px;
    }}
    .header-logo {{
      width: 34px;
      height: 34px;
      background: #05080c;
      border: 1px solid var(--ops-cyan);
      display: flex;
      align-items: center;
      justify-content: center;
      color: var(--ops-cyan);
      box-shadow: 0 0 12px rgba(6, 182, 212, 0.35), inset 0 0 8px rgba(6, 182, 212, 0.2);
    }}
    .header-logo svg {{
      width: 20px;
      height: 20px;
    }}
    .header-title {{
      font-family: var(--font-display);
      font-size: 18px;
      font-weight: 700;
      color: #ffffff;
      letter-spacing: 0.12em;
      line-height: 1.1;
      text-transform: uppercase;
      text-shadow: 0 0 8px rgba(6,182,212,0.5);
    }}
    .header-sub {{
      font-family: var(--font-mono);
      font-size: 10px;
      color: var(--ops-cyan);
      font-weight: 600;
      letter-spacing: 0.08em;
      text-transform: uppercase;
    }}
    .header-badges {{
      display: flex;
      gap: 8px;
      align-items: center;
    }}
    .badge {{
      padding: 4px 9px;
      font-family: var(--font-mono);
      font-size: 10px;
      font-weight: 600;
      letter-spacing: 0.06em;
      border: 1px solid var(--ops-border);
      background: #070b10;
      color: var(--ops-text-muted);
      display: inline-flex;
      align-items: center;
      gap: 6px;
      text-transform: uppercase;
    }}
    .badge-live {{
      background: var(--status-nominal-bg);
      color: var(--status-nominal);
      border-color: var(--status-nominal-border);
      box-shadow: 0 0 8px rgba(16, 185, 129, 0.2);
    }}
    .badge-live-dot {{
      width: 6px;
      height: 6px;
      border-radius: 50%;
      background: var(--status-nominal);
      box-shadow: 0 0 8px var(--status-nominal);
      animation: pulse-live 1.6s ease-in-out infinite;
    }}
    @keyframes pulse-live {{
      0%, 100% {{ opacity: 1; transform: scale(1); }}
      50% {{ opacity: 0.3; transform: scale(0.7); }}
    }}
    .badge-osm {{
      background: rgba(6, 182, 212, 0.1);
      color: var(--ops-cyan);
      border-color: rgba(6, 182, 212, 0.35);
    }}
    .badge-dist {{
      background: #090e15;
      color: #e2e8f0;
      border-color: var(--ops-border-strong);
      font-family: var(--font-mono);
    }}
    .badge-mono {{
      font-family: var(--font-mono);
      color: var(--ops-text-muted);
    }}

    /* ── Main Layout: Instrument Bay & Tactical Radar ── */
    .main-layout {{
      display: grid;
      grid-template-columns: 400px minmax(0, 1fr);
      flex: 1;
      height: calc(100vh - 54px);
      min-height: 0;
      overflow: hidden;
      position: relative;
      background: var(--ops-bg);
    }}
    .main-layout.showcase-subtitle-active {{ grid-template-columns:400px minmax(0,1fr) 350px; }}

    /* ── Left Sidebar (Instrument Rack with Chassis Framing) ── */
    .sidebar {{
      height: 100%;
      min-height: 0;
      background: var(--ops-rack);
      border-right: 2px solid var(--ops-border-strong);
      overflow-y: auto;
      overflow-x: hidden;
      display: flex;
      flex-direction: column;
      gap: 1px;
      scrollbar-width: thin;
      scrollbar-color: var(--ops-border-strong) var(--ops-rack);
      background-image: radial-gradient(#152233 1px, transparent 1px);
      background-size: 16px 16px;
    }}
    .sidebar::-webkit-scrollbar {{ width: 5px; }}
    .sidebar::-webkit-scrollbar-track {{ background: var(--ops-rack); }}
    .sidebar::-webkit-scrollbar-thumb {{ background: var(--ops-border-strong); }}

    /* Modular Instrument Bay Panels */
    .sidebar-section, .conn-gauge-section {{
      padding: 12px 14px;
      background: rgba(13, 20, 31, 0.96);
      border: 1px solid var(--ops-border);
      border-left: 3px solid var(--ops-border-strong);
      margin: 6px 8px 0 8px;
      position: relative;
      flex-shrink: 0;
    }}
    .sidebar-section:last-child {{ margin-bottom: 12px; }}

    /* Technical Corner Registration Brackets */
    .sidebar-section::before, .conn-gauge-section::before {{
      content: '';
      position: absolute;
      top: -1px;
      right: -1px;
      width: 6px;
      height: 6px;
      border-top: 1px solid var(--ops-cyan);
      border-right: 1px solid var(--ops-cyan);
      pointer-events: none;
    }}
    .sidebar-section::after, .conn-gauge-section::after {{
      content: '';
      position: absolute;
      bottom: -1px;
      right: -1px;
      width: 6px;
      height: 6px;
      border-bottom: 1px solid var(--ops-cyan);
      border-right: 1px solid var(--ops-cyan);
      pointer-events: none;
    }}

    .section-title {{
      font-family: var(--font-display);
      font-size: 12px;
      font-weight: 700;
      letter-spacing: 0.1em;
      text-transform: uppercase;
      color: #93c5fd;
      margin-bottom: 9px;
      display: flex;
      align-items: center;
      gap: 8px;
      border-bottom: 1px solid var(--ops-border);
      padding-bottom: 4px;
    }}
    .section-title svg {{
      width: 13px;
      height: 13px;
      color: var(--ops-cyan);
    }}

    /* ── Calibrated Segmented Bar Meters ── */
    .conn-gauge-header {{
      display: flex;
      align-items: center;
      justify-content: space-between;
      margin-bottom: 5px;
    }}
    .conn-gauge-title {{
      font-family: var(--font-display);
      font-size: 12px;
      font-weight: 700;
      letter-spacing: 0.06em;
      text-transform: uppercase;
      color: var(--ops-text);
      display: flex;
      align-items: center;
      gap: 6px;
    }}
    .conn-gauge-title-icon {{
      display: inline-flex;
      align-items: center;
      justify-content: center;
      color: var(--ops-cyan);
    }}
    .conn-gauge-title-icon svg {{
      width: 14px;
      height: 14px;
    }}
    .conn-gauge-value {{
      font-size: 16px;
      font-weight: 700;
      font-family: var(--font-mono);
      line-height: 1;
      font-variant-numeric: tabular-nums;
    }}
    .conn-gauge-value.green {{ color: var(--status-nominal); text-shadow: 0 0 6px rgba(16,185,129,0.4); }}
    .conn-gauge-value.yellow {{ color: var(--status-warning); text-shadow: 0 0 6px rgba(245,158,11,0.4); }}
    .conn-gauge-value.red {{ color: var(--status-critical); text-shadow: 0 0 6px rgba(239,68,68,0.5); }}

    /* Segmented Meter Track */
    .conn-gauge-track {{
      height: 8px;
      background: #05080c;
      border: 1px solid var(--ops-border);
      overflow: hidden;
      position: relative;
      background-image: repeating-linear-gradient(90deg, transparent 0, transparent 4px, #070b10 4px, #070b10 6px);
    }}
    .conn-gauge-fill {{
      height: 100%;
      transition: width 0.28s ease, background-color 0.3s ease;
    }}
    .conn-gauge-fill.green {{
      background: linear-gradient(90deg, #059669 0%, #10b981 100%);
      box-shadow: 0 0 8px rgba(16, 185, 129, 0.4);
    }}
    .conn-gauge-fill.yellow {{
      background: linear-gradient(90deg, #d97706 0%, #f59e0b 100%);
      box-shadow: 0 0 8px rgba(245, 158, 11, 0.4);
    }}
    .conn-gauge-fill.red {{
      background: linear-gradient(90deg, #b91c1c 0%, #ef4444 100%);
      box-shadow: 0 0 8px rgba(239, 68, 68, 0.5);
    }}

    /* Scale Ticks */
    .scale-ticks {{
      display: flex;
      justify-content: space-between;
      font-family: var(--font-mono);
      font-size: 8px;
      color: var(--ops-text-dim);
      margin-top: 2px;
      padding: 0 1px;
    }}

    .conn-gauge-meta {{
      display: flex;
      align-items: center;
      justify-content: space-between;
      margin-top: 5px;
    }}
    .conn-gauge-state {{
      font-size: 10px;
      font-family: var(--font-mono);
      font-weight: 600;
      letter-spacing: 0.06em;
    }}
    .conn-gauge-state.nominal {{ color: var(--status-nominal); }}
    .conn-gauge-state.blackout {{ color: var(--status-critical); font-weight: 700; }}
    /* Separate trust signal: amber means traffic may flow, but keepalive proof is absent. */
    .heartbeat-section {{ display: none; border-left-color: #10b981; }}
    .heartbeat-section.enabled {{ display: block; }}
    .heartbeat-section.degraded {{ border-left-color: #f59e0b; box-shadow: inset 0 0 22px rgba(245,158,11,.10); }}
    .heartbeat-led {{ width: 10px; height: 10px; border-radius: 50%; background: #10b981; box-shadow: 0 0 9px rgba(16,185,129,.7); }}
    .heartbeat-section.degraded .heartbeat-led {{ background: #f59e0b; box-shadow: 0 0 12px rgba(245,158,11,.9); animation: heartbeat-warning 0.7s infinite alternate; }}
    @keyframes heartbeat-warning {{ from {{ opacity:.35; transform:scale(.78); }} to {{ opacity:1; transform:scale(1.15); }} }}
    .heartbeat-title-row {{ display:flex; align-items:center; justify-content:space-between; gap:8px; }}
    .heartbeat-state {{ margin-top:8px; font:700 15px var(--font-display); letter-spacing:.08em; color:#10b981; }}
    .heartbeat-section.degraded .heartbeat-state {{ color:#fbbf24; }}
    .heartbeat-gap {{ margin-top:4px; font:600 11px var(--font-mono); color:var(--ops-text-muted); }}
    .heartbeat-data-proof {{ margin-top:8px; padding:6px 7px; border:1px solid rgba(16,185,129,.28); color:#6ee7b7; background:rgba(16,185,129,.07); font:600 9px var(--font-mono); letter-spacing:.03em; }}
    .integrity-section {{ display:none; border-left-color:#22c55e; }}
    .integrity-section.enabled {{ display:block; }}
    .integrity-grid {{ display:grid; grid-template-columns:repeat(2,minmax(0,1fr)); gap:6px; margin-top:9px; }}
    .integrity-cell {{ padding:6px; border:1px solid rgba(34,197,94,.2); background:rgba(34,197,94,.05); }}
    .integrity-label {{ font:600 8px var(--font-mono); color:var(--ops-text-muted); letter-spacing:.08em; }}
    .integrity-value {{ margin-top:3px; font:700 18px var(--font-display); color:#86efac; }}
    .integrity-phase {{ margin-top:8px; font:700 10px var(--font-mono); color:#67e8f9; letter-spacing:.06em; }}
    .showcase-subtitle {{
      display:none; min-width:0; height:100%; padding:18px;
      border-left:2px solid var(--ops-border-strong); border-right:1px solid var(--ops-border);
      background:var(--ops-rack); box-shadow:inset 12px 0 28px rgba(0,0,0,.25);
      color:#ecfeff; opacity:0; transition:opacity .28s ease, transform .28s ease;
      flex-direction:column; justify-content:center; gap:12px; transform:translateX(8px);
    }}
    .showcase-subtitle.enabled {{ display:flex; opacity:1; transform:translateX(0); }}
    .showcase-subtitle.changing {{ opacity:.22; transform:translateX(6px); }}
    .showcase-subtitle-header {{ width:100%; color:#67e8f9; font:700 11px var(--font-mono); letter-spacing:.18em; text-align:left; }}
    .showcase-subtitle-rule {{ width:100%; height:1px; background:linear-gradient(90deg,#67e8f9,transparent); opacity:.6; }}
    .showcase-subtitle-body {{ width:100%; padding:18px; border:1px solid rgba(103,232,249,.38); border-left:4px solid #67e8f9; background:linear-gradient(135deg,rgba(8,22,34,.96),rgba(4,11,19,.94)); box-shadow:0 10px 24px rgba(0,0,0,.28); color:#ecfeff; font:600 19px/1.48 var(--font-display); letter-spacing:.02em; text-align:left; }}
    .showcase-subtitle-footer {{ width:100%; color:var(--ops-text-muted); font:600 9px var(--font-mono); letter-spacing:.1em; text-align:left; }}
    .conn-gauge-dist {{
      font-size: 10px;
      font-family: var(--font-mono);
      color: var(--ops-text-muted);
    }}

    /* ── Tactical Geography Bay ── */
    .geo-card {{
      background: #080d14;
      border: 1px solid var(--ops-border);
      padding: 8px 10px;
      display: flex;
      flex-direction: column;
      gap: 5px;
    }}
    .geo-row {{
      display: flex;
      align-items: center;
      justify-content: space-between;
      font-size: 11px;
    }}
    .geo-label {{
      color: var(--ops-text-muted);
      font-family: var(--font-display);
      font-size: 11px;
      letter-spacing: 0.04em;
      text-transform: uppercase;
    }}
    .geo-val {{
      font-family: var(--font-mono);
      color: var(--ops-text);
      font-size: 11px;
      font-variant-numeric: tabular-nums;
    }}
    .geo-dist-bar {{
      height: 3px;
      background: #05080c;
      overflow: hidden;
      margin-top: 3px;
    }}
    .geo-dist-fill {{
      height: 100%;
      width: 100%;
      background: var(--ops-cyan);
      box-shadow: 0 0 6px var(--ops-cyan);
    }}

    /* ── Zone Overview Telemetry Counters ── */
    .stats-grid {{
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 6px;
    }}
    .stat-card {{
      background: #070c12;
      border: 1px solid var(--ops-border);
      padding: 8px 10px;
      position: relative;
    }}
    .stat-card::before {{
      content: '';
      position: absolute;
      top: 0;
      left: 0;
      width: 2px;
      height: 100%;
      background: var(--ops-border-strong);
    }}
    .stat-value {{
      font-size: 22px;
      font-weight: 700;
      font-family: var(--font-mono);
      color: #ffffff;
      line-height: 1;
      font-variant-numeric: tabular-nums;
    }}
    .stat-label {{
      font-family: var(--font-display);
      font-size: 10px;
      font-weight: 700;
      letter-spacing: 0.08em;
      text-transform: uppercase;
      color: var(--ops-text-muted);
      margin-top: 4px;
    }}

    /* ── Unit Roster Bay ── */
    .unit-roster {{ display: flex; flex-direction: column; gap: 6px; }}
    .unit-card {{
      background: #080d14;
      border: 1px solid var(--ops-border);
      border-left: 3px solid var(--ops-text-muted);
      padding: 8px 10px;
      display: grid;
      grid-template-columns: 32px 1fr auto;
      gap: 8px;
      align-items: center;
      transition: all 0.15s ease;
    }}
    .unit-card.selected {{
      border-color: var(--ops-cyan);
      background: rgba(6, 182, 212, 0.08);
      box-shadow: inset 0 0 10px rgba(6, 182, 212, 0.1);
    }}
    .unit-card-icon {{ width: 28px; height: 28px; object-fit: contain; }}
    .unit-card-name {{
      font-family: var(--font-display);
      font-size: 13px;
      font-weight: 700;
      letter-spacing: 0.06em;
      text-transform: uppercase;
      color: #ffffff;
    }}
    .unit-card-meta {{
      font-size: 9px;
      color: var(--ops-text-muted);
      font-family: var(--font-mono);
      margin-top: 2px;
    }}
    .unit-score {{
      font-family: var(--font-mono);
      font-size: 13px;
      font-weight: 700;
      color: var(--status-nominal);
      text-align: right;
    }}
    .unit-select-btn {{
      grid-column: 2 / 4;
      border: 1px solid var(--ops-border-strong);
      background: #05080c;
      color: var(--ops-text);
      font-family: var(--font-display);
      font-size: 11px;
      font-weight: 700;
      letter-spacing: 0.08em;
      text-transform: uppercase;
      padding: 5px 8px;
      cursor: pointer;
      text-align: left;
      transition: all 0.15s ease;
    }}
    .unit-select-btn:hover:not(:disabled) {{
      border-color: var(--ops-cyan);
      color: var(--ops-cyan);
      background: rgba(6, 182, 212, 0.15);
    }}
    .unit-select-btn:disabled {{
      opacity: 0.55;
      cursor: default;
    }}

    /* ── Mission Briefing & Tactical Dispatch ── */
    .mission-briefing-card {{
      background: #080d14;
      border: 1px solid var(--ops-border);
      border-left: 3px solid var(--ops-cyan);
      padding: 10px;
      margin-bottom: 8px;
    }}
    .briefing-header {{
      display: flex;
      align-items: center;
      justify-content: space-between;
      margin-bottom: 8px;
      padding-bottom: 5px;
      border-bottom: 1px solid var(--ops-border);
    }}
    .briefing-badge {{
      font-family: var(--font-display);
      font-size: 11px;
      font-weight: 700;
      letter-spacing: 0.08em;
      text-transform: uppercase;
      color: var(--ops-cyan);
    }}
    .briefing-status {{
      font-size: 10px;
      font-family: var(--font-mono);
      font-weight: 600;
      letter-spacing: 0.06em;
      color: var(--ops-text-muted);
    }}
    .briefing-item {{
      background: #05080c;
      border: 1px solid var(--ops-border);
      border-left: 2px solid var(--ops-cyan);
      padding: 6px 8px;
      margin-bottom: 5px;
    }}
    .briefing-item:last-child {{ margin-bottom: 0; }}
    .briefing-item-id {{
      font-family: var(--font-mono);
      font-size: 10px;
      font-weight: 600;
      color: var(--ops-cyan);
      margin-right: 6px;
    }}
    .briefing-item-type {{
      font-family: var(--font-display);
      font-size: 11px;
      font-weight: 700;
      letter-spacing: 0.06em;
      text-transform: uppercase;
      color: var(--ops-text);
    }}
    .briefing-item-meta {{
      font-size: 10px;
      color: var(--ops-text-muted);
      font-family: var(--font-mono);
      margin-top: 2px;
    }}

    /* Tactical Action Switch Button */
    .send-mission-btn {{
      width: 100%;
      background: #065f46;
      color: #ffffff;
      border: 1px solid #10b981;
      padding: 11px 14px;
      font-family: var(--font-display);
      font-size: 14px;
      font-weight: 700;
      letter-spacing: 0.12em;
      text-transform: uppercase;
      cursor: pointer;
      display: flex;
      align-items: center;
      justify-content: center;
      gap: 8px;
      transition: all 0.15s ease;
      box-shadow: 0 0 14px rgba(16, 185, 129, 0.35);
    }}
    .send-mission-btn:not(:disabled):hover {{
      background: #059669;
      border-color: #34d399;
      box-shadow: 0 0 22px rgba(16, 185, 129, 0.6);
    }}
    .send-mission-btn:disabled {{
      background: #0c1219;
      border-color: var(--ops-border);
      color: var(--ops-text-dim);
      cursor: not-allowed;
      box-shadow: none;
    }}

    .exec-status-label {{
      font-size: 10px;
      font-family: var(--font-mono);
      font-weight: 600;
      letter-spacing: 0.06em;
      padding: 6px 8px;
      text-align: center;
      margin-top: 6px;
      border: 1px solid transparent;
      text-transform: uppercase;
    }}
    .exec-status-label.awaiting {{
      background: var(--status-warning-bg);
      color: var(--status-warning);
      border-color: var(--status-warning-border);
    }}
    .exec-status-label.en-route {{
      background: var(--status-nominal-bg);
      color: var(--status-nominal);
      border-color: var(--status-nominal-border);
    }}
    .exec-status-label.idle {{
      background: #05080c;
      color: var(--ops-text-muted);
      border-color: var(--ops-border);
    }}

    .mission-panel {{
      background: #080d14;
      border: 1px solid var(--ops-border);
      padding: 10px;
      margin-top: 8px;
      display: none;
    }}
    .mission-panel.visible {{ display: block; }}
    .mission-panel pre {{
      font-size: 10px;
      color: var(--ops-text-muted);
      font-family: var(--font-mono);
      white-space: pre-wrap;
      word-break: break-all;
    }}

    /* ── Live Beacon SITREP Event Feed ── */
    .event-list {{
      display: flex;
      flex-direction: column;
      gap: 5px;
    }}
    .event-card {{
      background: #080d14;
      border: 1px solid var(--ops-border);
      border-left: 3px solid #475569;
      padding: 7px 9px;
      cursor: pointer;
      transition: all 0.15s ease;
    }}
    .event-card:hover {{
      background: #111a28;
      border-color: var(--ops-border-strong);
    }}
    .event-card.victim   {{ border-left-color: var(--status-critical); }}
    .event-card.fire     {{ border-left-color: #ea580c; }}
    .event-card.hazard   {{ border-left-color: var(--status-warning); }}
    .event-card.gas      {{ border-left-color: var(--status-nominal); }}

    .event-header {{
      display: flex;
      align-items: center;
      justify-content: space-between;
      margin-bottom: 3px;
    }}
    .event-type {{
      font-family: var(--font-display);
      font-size: 11px;
      font-weight: 700;
      letter-spacing: 0.06em;
      text-transform: uppercase;
    }}
    .event-sev {{
      font-size: 10px;
      font-family: var(--font-mono);
      padding: 1px 5px;
      background: #05080c;
      border: 1px solid var(--ops-border);
      color: var(--ops-text);
      font-weight: 600;
    }}
    .event-id {{
      font-size: 10px;
      color: var(--ops-text-muted);
      font-family: var(--font-mono);
    }}
    .event-coords {{
      font-size: 10px;
      color: #7dd3fc;
      font-family: var(--font-mono);
      margin-top: 2px;
    }}
    .event-ttl {{
      font-size: 10px;
      color: var(--ops-text-dim);
      font-family: var(--font-mono);
      margin-top: 2px;
    }}
    .stale-badge {{
      font-size: 9px;
      font-family: var(--font-mono);
      color: var(--status-warning);
      background: var(--status-warning-bg);
      border: 1px solid rgba(245, 158, 11, 0.3);
      padding: 1px 4px;
      margin-left: 4px;
    }}

    /* ── Tactical Radar Map Theater ── */
    .map-container {{
      height: 100%;
      min-height: 0;
      position: relative;
      background: #e2e8f0;
    }}
    #map {{
      width: 100%;
      height: 100%;
      background: #e2e8f0;
    }}

    /* HUD Reticle Coordinate Ticker in Map */
    .map-hud-coords {{
      position: absolute;
      top: 14px;
      left: 14px;
      z-index: 500;
      font-family: var(--font-mono);
      font-size: 10px;
      color: var(--ops-cyan);
      background: rgba(5, 8, 12, 0.85);
      border: 1px solid var(--ops-border-strong);
      padding: 4px 8px;
      letter-spacing: 0.08em;
      pointer-events: none;
    }}

    /* HUD Corner Registration Marks */
    .map-container::before {{
      content: '+';
      position: absolute;
      top: 10px;
      right: 10px;
      font-family: var(--font-mono);
      font-size: 16px;
      color: var(--ops-cyan);
      z-index: 600;
      pointer-events: none;
    }}
    .map-container::after {{
      content: '+';
      position: absolute;
      bottom: 10px;
      left: 10px;
      font-family: var(--font-mono);
      font-size: 16px;
      color: var(--ops-cyan);
      z-index: 600;
      pointer-events: none;
    }}

    /* Top-Right Tactical Buttons */
    .communication-console-btn {{
      position: absolute;
      z-index: 900;
      top: 14px;
      right: 14px;
      display: inline-flex;
      align-items: center;
      gap: 8px;
      padding: 8px 14px;
      color: #ffffff;
      background: #090e15;
      border: 1px solid var(--ops-border-strong);
      border-left: 3px solid var(--ops-cyan);
      font-family: var(--font-display);
      font-size: 12px;
      font-weight: 700;
      letter-spacing: 0.08em;
      text-transform: uppercase;
      text-decoration: none;
      box-shadow: 0 4px 16px rgba(0, 0, 0, 0.6);
      cursor: pointer;
      transition: all 0.15s ease;
    }}
    .communication-console-btn:hover {{
      color: var(--ops-cyan);
      border-color: var(--ops-cyan);
      background: #0f1722;
      box-shadow: 0 0 16px rgba(6, 182, 212, 0.35);
    }}
    .communication-console-btn svg {{
      width: 14px;
      height: 14px;
    }}
    .webots-visualization-btn {{
      position: absolute;
      z-index: 900;
      top: 58px;
      right: 14px;
      display: inline-flex;
      align-items: center;
      gap: 8px;
      padding: 8px 14px;
      color: #d1fae5;
      background: #071510;
      border: 1px solid rgba(16, 185, 129, .55);
      border-left: 3px solid #10b981;
      font-family: var(--font-display);
      font-size: 12px;
      font-weight: 700;
      letter-spacing: .08em;
      text-transform: uppercase;
      box-shadow: 0 4px 16px rgba(0, 0, 0, .6);
      cursor: pointer;
      transition: all .15s ease;
    }}
    .webots-visualization-btn:hover:not(:disabled) {{
      color: #ffffff;
      background: #0d2a20;
      box-shadow: 0 0 16px rgba(16, 185, 129, .30);
    }}
    .webots-visualization-btn:disabled {{ opacity: .7; cursor: wait; }}
    .webots-vision-btn {{
      position: absolute; z-index: 900; top: 106px; right: 14px;
      padding: 8px 14px; color: #bae6fd; background: #07131d;
      border: 1px solid rgba(56, 189, 248, .6); border-left: 3px solid #38bdf8;
      font-family: var(--font-display); font-size: 12px; font-weight: 700;
      letter-spacing: .08em; cursor: pointer; box-shadow: 0 4px 16px rgba(0,0,0,.6);
    }}
    .webots-vision-btn:hover {{ background: #0a2637; color: #fff; }}
    .webots-status-dot {{
      width: 7px;
      height: 7px;
      border-radius: 50%;
      background: #10b981;
      box-shadow: 0 0 8px #10b981;
    }}

    /* Tactical Map Legend */
    .map-overlay {{
      position: absolute;
      top: 56px;
      right: 14px;
      z-index: 500;
      background: rgba(9, 14, 21, 0.94);
      backdrop-filter: blur(6px);
      border: 1px solid var(--ops-border-strong);
      border-top: 2px solid var(--ops-cyan);
      padding: 10px 14px;
      min-width: 200px;
      box-shadow: 0 8px 24px rgba(0, 0, 0, 0.7);
    }}
    .overlay-title {{
      font-family: var(--font-display);
      font-size: 11px;
      font-weight: 700;
      letter-spacing: 0.1em;
      text-transform: uppercase;
      color: #93c5fd;
      margin-bottom: 8px;
      padding-bottom: 4px;
      border-bottom: 1px solid var(--ops-border);
    }}
    .legend-item {{
      display: flex;
      align-items: center;
      gap: 8px;
      font-size: 11px;
      color: var(--ops-text);
      margin-bottom: 5px;
    }}
    .legend-item:last-child {{ margin-bottom: 0; }}
    .legend-item svg {{
      width: 14px;
      height: 14px;
      flex-shrink: 0;
    }}
    .legend-dot {{
      width: 9px;
      height: 9px;
      border-radius: 50%;
      flex-shrink: 0;
    }}
    .legend-label {{
      font-size: 10px;
      color: var(--ops-text-muted);
      font-family: var(--font-mono);
    }}

    .map-reset-btn {{
      position: absolute;
      bottom: 18px;
      right: 14px;
      z-index: 500;
      background: #090e15;
      color: var(--ops-text);
      border: 1px solid var(--ops-border-strong);
      padding: 8px 12px;
      font-family: var(--font-display);
      font-size: 11px;
      font-weight: 700;
      letter-spacing: 0.08em;
      text-transform: uppercase;
      cursor: pointer;
      box-shadow: 0 4px 16px rgba(0,0,0,0.6);
      display: flex;
      align-items: center;
      gap: 6px;
      transition: all 0.15s ease;
    }}
    .map-reset-btn:hover {{
      background: #111a24;
      border-color: var(--ops-cyan);
      color: var(--ops-cyan);
    }}
    .map-reset-btn svg {{
      width: 13px;
      height: 13px;
    }}

    /* ── Leaflet Dark Tactical Theme Overrides ── */
    .leaflet-popup-content-wrapper {{
      background: #090e15 !important;
      color: #e2e8f0 !important;
      border: 1px solid #1f2e40 !important;
      border-left: 3px solid var(--ops-cyan) !important;
      border-radius: 0 !important;
      box-shadow: 0 8px 24px rgba(0, 0, 0, 0.85), 0 0 1px var(--ops-cyan) !important;
      padding: 0 !important;
    }}
    .leaflet-popup-tip {{
      background: #090e15 !important;
      border: 1px solid #1f2e40 !important;
    }}
    .leaflet-popup-content {{
      margin: 10px 14px !important;
      line-height: 1.4 !important;
    }}
    /* Beacon telemetry stays readable without obscuring the tactical map. */
    .leaflet-popup.beacon-event-popup .leaflet-popup-content {{
      width: 210px !important;
      margin: 7px 10px !important;
      line-height: 1.2 !important;
    }}
    .leaflet-popup.beacon-event-popup .tactical-popup-title {{
      font-size: 11px !important;
      margin-bottom: 3px !important;
    }}
    .leaflet-popup.beacon-event-popup .tactical-popup-sub {{
      font-size: 8px !important;
      margin-bottom: 4px !important;
    }}
    .leaflet-popup.beacon-event-popup .tactical-popup-row {{
      font-size: 8.5px !important;
      margin-top: 2px !important;
    }}
    .leaflet-popup.beacon-event-popup .tactical-popup-status {{
      font-size: 8.5px !important;
      margin-top: 3px !important;
    }}
    .leaflet-popup.beacon-event-popup .leaflet-popup-close-button {{
      padding: 3px !important;
      top: 2px !important;
      right: 2px !important;
    }}
    /* RF Mesh Relay Demo: one compact, short-lived first-beacon notice. */
    .leaflet-popup.mesh-transient-popup {{
      position: fixed !important;
      top: 72px !important;
      left: 416px !important;
      bottom: auto !important;
      transform: none !important;
      z-index: 1200 !important;
    }}
    .leaflet-popup.mesh-transient-popup .leaflet-popup-content {{
      margin: 6px 9px !important;
      line-height: 1.18 !important;
      min-width: 0 !important;
      width: 180px !important;
    }}
    .leaflet-popup.mesh-transient-popup .leaflet-popup-tip-container {{ display: none !important; }}
    .leaflet-popup.mesh-transient-popup .tactical-popup-title {{
      font-size: 10px !important;
      letter-spacing: .07em !important;
    }}
    .leaflet-popup.mesh-transient-popup .tactical-popup-sub,
    .leaflet-popup.mesh-transient-popup .tactical-popup-row {{
      font-size: 9px !important;
      margin-top: 3px !important;
    }}
    .leaflet-container a.leaflet-popup-close-button {{
      color: #94a3b8 !important;
      padding: 6px !important;
      top: 4px !important;
      right: 4px !important;
    }}
    .leaflet-container a.leaflet-popup-close-button:hover {{
      color: var(--status-critical) !important;
    }}

    .leaflet-tooltip {{
      background: #05080c !important;
      color: #38bdf8 !important;
      border: 1px solid #1e293b !important;
      border-radius: 0 !important;
      font-family: var(--font-mono) !important;
      font-size: 10px !important;
      letter-spacing: 0.06em !important;
      box-shadow: 0 4px 12px rgba(0,0,0,0.8) !important;
      padding: 3px 6px !important;
    }}
    .leaflet-bar a {{
      background-color: #090e15 !important;
      border-bottom: 1px solid #1c2736 !important;
      color: #94a3b8 !important;
      border-radius: 0 !important;
      transition: all 0.15s ease !important;
    }}
    .leaflet-bar a:hover {{
      background-color: #141e2a !important;
      color: var(--ops-cyan) !important;
    }}
    .leaflet-control-layers {{
      background: #090e15 !important;
      border: 1px solid #1c2736 !important;
      border-radius: 0 !important;
      color: #cbd5e1 !important;
      font-family: var(--font-mono) !important;
      font-size: 11px !important;
      box-shadow: 0 4px 16px rgba(0,0,0,0.8) !important;
    }}

    /* Tactical Marker Styles */
    .tactical-marker-pin {{
      display: flex;
      flex-direction: column;
      align-items: center;
    }}
    .tactical-pin-box {{
      width: 36px;
      height: 36px;
      display: flex;
      align-items: center;
      justify-content: center;
    }}
    .tactical-pin-label {{
      background: #05080c;
      font-family: var(--font-mono);
      font-size: 9px;
      font-weight: 700;
      padding: 2px 6px;
      margin-top: 3px;
      white-space: nowrap;
      letter-spacing: 0.05em;
    }}
    .tactical-popup-title {{
      font-family: var(--font-display);
      font-size: 13px;
      font-weight: 700;
      letter-spacing: 0.08em;
      text-transform: uppercase;
      margin-bottom: 4px;
    }}
    .tactical-popup-sub {{
      font-size: 10px;
      color: var(--ops-text-muted);
      margin-bottom: 6px;
      font-family: var(--font-sans);
    }}
    .tactical-popup-row {{
      font-size: 10px;
      font-family: var(--font-mono);
      margin-top: 2px;
      color: #e2e8f0;
    }}
    .tactical-popup-status {{
      font-size: 10px;
      font-weight: 600;
      font-family: var(--font-mono);
      margin-top: 4px;
    }}

    /* ── Continuity Demo Banner ── */
    .continuity-overlay {{
      position: absolute;
      top: 14px;
      left: 50%;
      transform: translateX(-50%);
      z-index: 1000;
      pointer-events: none;
      opacity: 0;
      transition: opacity 0.3s ease;
      width: 90%;
      max-width: 640px;
    }}
    .continuity-overlay.visible {{
      opacity: 1;
      pointer-events: auto;
    }}
    .continuity-banner {{
      padding: 9px 14px;
      font-family: var(--font-sans);
      font-size: 12px;
      font-weight: 600;
      box-shadow: 0 4px 20px rgba(0,0,0,0.8);
      border: 1px solid transparent;
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
    }}
    .continuity-banner.before {{
      background: #7f1d1d;
      color: #fef2f2;
      border-color: #ef4444;
    }}
    .continuity-banner.after {{
      background: #064e3b;
      color: #f0fdf4;
      border-color: #10b981;
    }}
    .continuity-banner-text {{
      flex: 1;
      text-align: left;
      line-height: 1.4;
    }}
    .continuity-skip-btn {{
      background: rgba(255,255,255,0.15);
      color: white;
      border: 1px solid rgba(255,255,255,0.25);
      padding: 3px 8px;
      font-family: var(--font-mono);
      font-size: 11px;
      cursor: pointer;
      font-weight: 600;
      flex-shrink: 0;
    }}

    /* ── Satellite-to-mesh failover notification ──
       Reuses the same tactical overlay language as the continuity banner. */
    .failover-overlay {{
      position: absolute;
      top: 20px;
      left: 50%;
      z-index: 1500;
      display: flex;
      align-items: center;
      justify-content: center;
      padding: 0;
      transform: translateX(-50%);
      opacity: 0;
      visibility: hidden;
      pointer-events: none;
      background: transparent;
      transition: opacity .22s ease, visibility .22s ease;
    }}
    .failover-overlay.visible {{
      opacity: 1;
      visibility: visible;
    }}
    .failover-banner {{
      display: flex;
      align-items: center;
      gap: 10px;
      width: min(580px, calc(100vw - 80px));
      padding: 20px 24px;
      background: linear-gradient(135deg, #32140b, #180c0a);
      border: 1px solid #f97316;
      border-left: 4px solid #fb923c;
      box-shadow: 0 10px 26px rgba(0,0,0,.72), 0 0 18px rgba(249,115,22,.30);
      color: #fff7ed;
      font-family: var(--font-sans);
      transform: scale(.94);
      transition: transform .22s ease;
    }}
    .failover-overlay.visible .failover-banner {{
      transform: scale(1);
    }}
    .failover-banner-icon {{
      color: #fb923c;
      font-size: 42px;
      line-height: 1;
      text-shadow: 0 0 18px rgba(251,146,60,.8);
    }}
    .failover-banner-title {{
      color: #fed7aa;
      font-family: var(--font-display);
      font-size: 28px;
      font-weight: 700;
      letter-spacing: .06em;
      text-transform: uppercase;
    }}
    .failover-banner-detail {{
      margin-top: 3px;
      color: #fdba74;
      font-family: var(--font-mono);
      font-size: 13px;
      font-weight: 600;
      letter-spacing: .08em;
    }}

    /* Comparison baselines: intentionally large, persistent and unmissable. */
    .baseline-failure-overlay {{
      position:absolute; inset:0; z-index:2000; display:none; align-items:center; justify-content:center;
      padding:28px; background:rgba(2,6,12,.78); pointer-events:none;
    }}
    .baseline-failure-overlay.visible {{ display:flex; }}
    .baseline-failure-card {{
      width:min(760px, 92%); padding:34px 40px; text-align:center;
      background:linear-gradient(135deg,#3b0909,#170a0a); border:2px solid #ef4444; border-top:9px solid #fb7185;
      box-shadow:0 22px 70px rgba(0,0,0,.85),0 0 50px rgba(239,68,68,.42);
      color:#fff1f2; animation:baseline-alert-pulse 1.1s infinite alternate;
    }}
    @keyframes baseline-alert-pulse {{ from {{ transform:scale(.98); }} to {{ transform:scale(1); box-shadow:0 22px 70px rgba(0,0,0,.85),0 0 70px rgba(239,68,68,.70); }} }}
    .baseline-failure-icon {{ font-size:62px; line-height:1; color:#fda4af; }}
    .baseline-failure-title {{ margin-top:10px; font:700 clamp(28px,4vw,48px) var(--font-display); letter-spacing:.09em; text-transform:uppercase; }}
    .baseline-failure-detail {{ margin-top:14px; color:#fecdd3; font:600 14px/1.55 var(--font-mono); letter-spacing:.04em; }}
    .baseline-failure-note {{ margin-top:18px; color:#fda4af; font:700 11px var(--font-mono); letter-spacing:.10em; text-transform:uppercase; }}

    /* ── Communication Console Dialog ── */
    .communication-console-backdrop {{
      display: none;
      position: fixed;
      z-index: 5000;
      inset: 0;
      padding: 20px;
      background: rgba(3, 6, 10, 0.88);
      backdrop-filter: blur(4px);
    }}
    .communication-console-backdrop.visible {{ display: flex; }}
    .communication-console-dialog {{
      display: flex;
      flex-direction: column;
      width: min(1500px, 100%);
      height: min(920px, 100%);
      margin: auto;
      overflow: hidden;
      background: #05080c;
      border: 1px solid var(--ops-border-strong);
      border-top: 3px solid var(--ops-cyan);
      box-shadow: 0 22px 70px rgba(0, 0, 0, 0.85);
    }}
    .communication-console-dialog-header {{
      display: flex;
      align-items: center;
      justify-content: space-between;
      padding: 10px 14px;
      color: var(--ops-text);
      background: #0a111a;
      border-bottom: 1px solid var(--ops-border-strong);
      font-family: var(--font-mono);
      font-weight: 600;
      font-size: 11px;
      letter-spacing: 0.06em;
    }}
    .communication-console-close {{
      padding: 6px 10px;
      color: var(--ops-text);
      background: transparent;
      border: 1px solid var(--ops-border-strong);
      font-family: var(--font-mono);
      font-size: 10px;
      font-weight: 600;
      cursor: pointer;
      transition: all 0.15s ease;
    }}
    .communication-console-close:hover {{
      color: var(--ops-cyan);
      border-color: var(--ops-cyan);
    }}
    .communication-console-frame {{
      flex: 1;
      width: 100%;
      border: 0;
      background: #05080c;
    }}

    /* ── Mission Summary Modal ── */
    .summary-backdrop {{
      display: none;
      position: fixed;
      inset: 0;
      background: rgba(2, 4, 8, 0.9);
      backdrop-filter: blur(6px);
      z-index: 2000;
      align-items: center;
      justify-content: center;
    }}
    .summary-backdrop.visible {{
      display: flex;
    }}
    .summary-modal {{
      background: #0a1017;
      border: 1px solid var(--ops-border-strong);
      border-top: 3px solid var(--ops-cyan);
      box-shadow: 0 16px 40px rgba(0, 0, 0, 0.85);
      width: min(840px, 94vw);
      max-height: 88vh;
      overflow-y: auto;
      padding: 24px 28px;
      position: relative;
    }}
    .summary-header {{
      display: flex;
      align-items: flex-start;
      justify-content: space-between;
      margin-bottom: 20px;
      padding-bottom: 14px;
      border-bottom: 1px solid var(--ops-border);
      gap: 16px;
    }}
    .summary-header-left {{ flex: 1; }}
    .summary-badge {{
      display: inline-flex;
      align-items: center;
      font-family: var(--font-display);
      font-size: 11px;
      font-weight: 700;
      letter-spacing: 0.08em;
      text-transform: uppercase;
      color: var(--status-nominal);
      background: var(--status-nominal-bg);
      border: 1px solid var(--status-nominal-border);
      padding: 3px 8px;
      margin-bottom: 8px;
    }}
    .summary-title {{
      font-family: var(--font-display);
      font-size: 22px;
      font-weight: 700;
      letter-spacing: 0.06em;
      text-transform: uppercase;
      color: var(--ops-text);
      margin-bottom: 4px;
    }}
    .summary-meta {{
      font-size: 11px;
      color: var(--ops-text-muted);
      font-family: var(--font-mono);
    }}
    .summary-close-btn {{
      background: #090e15;
      border: 1px solid var(--ops-border);
      color: var(--ops-text-muted);
      width: 30px;
      height: 30px;
      cursor: pointer;
      font-size: 14px;
      display: flex;
      align-items: center;
      justify-content: center;
      flex-shrink: 0;
      transition: all 0.15s ease;
    }}
    .summary-close-btn:hover {{
      background: #152233;
      color: #fff;
    }}
    .summary-sections {{
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 14px;
      margin-bottom: 18px;
    }}
    .summary-card {{
      background: #070c12;
      border: 1px solid var(--ops-border);
      border-left: 3px solid var(--ops-border-strong);
      padding: 14px;
    }}
    .summary-card.full-width {{
      grid-column: 1 / -1;
    }}
    .summary-card-title {{
      font-family: var(--font-display);
      font-size: 12px;
      font-weight: 700;
      letter-spacing: 0.08em;
      text-transform: uppercase;
      color: #93c5fd;
      margin-bottom: 10px;
      display: flex;
      align-items: center;
      gap: 6px;
    }}
    .summary-card-title svg {{
      width: 14px;
      height: 14px;
      color: var(--ops-cyan);
    }}
    .summary-metric {{
      display: flex;
      align-items: baseline;
      gap: 6px;
      margin-bottom: 8px;
    }}
    .summary-metric-val {{
      font-size: 28px;
      font-weight: 700;
      font-family: var(--font-mono);
      line-height: 1;
    }}
    .summary-metric-val.green {{ color: var(--status-nominal); }}
    .summary-metric-val.yellow {{ color: var(--status-warning); }}
    .summary-metric-val.red {{ color: var(--status-critical); }}
    .summary-metric-unit {{
      font-size: 11px;
      color: var(--ops-text-muted);
      font-family: var(--font-sans);
    }}
    .summary-progress {{
      height: 6px;
      background: #05080c;
      overflow: hidden;
      margin: 6px 0 10px;
      border: 1px solid var(--ops-border);
    }}
    .summary-progress-fill {{
      height: 100%;
      background: var(--status-nominal);
      transition: width 0.5s ease;
    }}
    .summary-row {{
      display: flex;
      justify-content: space-between;
      align-items: center;
      font-size: 11px;
      padding: 4px 0;
      border-bottom: 1px solid rgba(255,255,255,0.04);
    }}
    .summary-row:last-child {{
      border-bottom: none;
    }}
    .summary-row-label {{ color: var(--ops-text-muted); font-family: var(--font-sans); }}
    .summary-row-val {{
      font-family: var(--font-mono);
      font-weight: 600;
      color: var(--ops-text);
    }}

    .beacon-priority-list {{
      display: flex;
      flex-direction: column;
      gap: 5px;
    }}
    .beacon-priority-item {{
      display: flex;
      align-items: center;
      gap: 8px;
      font-size: 11px;
    }}
    .beacon-priority-rank {{
      width: 18px;
      height: 18px;
      background: #05080c;
      border: 1px solid var(--ops-border);
      color: var(--ops-text-muted);
      font-family: var(--font-mono);
      font-size: 10px;
      font-weight: 600;
      display: flex;
      align-items: center;
      justify-content: center;
      flex-shrink: 0;
    }}
    .beacon-priority-bar {{
      flex: 1;
      height: 4px;
      background: #05080c;
      overflow: hidden;
    }}
    .beacon-priority-fill {{
      height: 100%;
      background: var(--ops-cyan);
    }}
    .beacon-priority-pct {{
      font-family: var(--font-mono);
      color: var(--ops-text);
      font-size: 10px;
      width: 32px;
      text-align: right;
    }}

    .failure-recap-list {{
      display: flex;
      flex-direction: column;
      gap: 8px;
    }}
    .failure-recap-item {{
      background: #05080c;
      border: 1px solid var(--ops-border);
      border-left: 3px solid var(--status-critical);
      padding: 8px 12px;
    }}
    .failure-recap-item.satellite {{ border-left-color: #ea580c; }}
    .failure-recap-item.beacon {{ border-left-color: var(--status-warning); }}
    .failure-recap-item.stacked {{ border-left-color: #a855f7; }}
    .failure-recap-header {{
      display: flex;
      align-items: center;
      justify-content: space-between;
      margin-bottom: 3px;
    }}
    .failure-recap-label {{
      font-family: var(--font-display);
      font-size: 11px;
      font-weight: 700;
      letter-spacing: 0.06em;
      text-transform: uppercase;
      color: var(--ops-text);
    }}
    .failure-recap-ts {{
      font-size: 10px;
      font-family: var(--font-mono);
      color: var(--ops-text-muted);
    }}
    .failure-recap-detail {{
      font-size: 11px;
      color: var(--ops-text-muted);
      line-height: 1.4;
    }}
    .nominal-badge {{
      display: inline-flex;
      align-items: center;
      font-family: var(--font-mono);
      font-size: 11px;
      color: var(--status-nominal);
      background: var(--status-nominal-bg);
      border: 1px solid var(--status-nominal-border);
      padding: 6px 12px;
      font-weight: 600;
    }}

    .summary-footer {{
      display: flex;
      align-items: center;
      justify-content: space-between;
      padding-top: 14px;
      border-top: 1px solid var(--ops-border);
    }}
    .summary-countdown {{
      font-size: 11px;
      color: var(--ops-text-muted);
      font-family: var(--font-mono);
    }}
    .summary-export-btn {{
      background: #0284c7;
      color: #ffffff;
      border: 1px solid #38bdf8;
      padding: 8px 16px;
      font-family: var(--font-display);
      font-weight: 700;
      font-size: 12px;
      letter-spacing: 0.08em;
      text-transform: uppercase;
      cursor: pointer;
      margin-right: 8px;
      display: inline-flex;
      align-items: center;
      gap: 6px;
      transition: all 0.15s ease;
      box-shadow: 0 0 12px rgba(2, 132, 199, 0.35);
    }}
    .summary-export-btn:hover {{
      background: #0369a1;
      border-color: #7dd3fc;
      box-shadow: 0 0 18px rgba(2, 132, 199, 0.6);
    }}
    .summary-export-btn svg {{
      width: 14px;
      height: 14px;
    }}
    .summary-continue-btn {{
      background: #111a26;
      color: #f1f5f9;
      border: 1px solid #25394f;
      padding: 8px 18px;
      font-family: var(--font-display);
      font-size: 12px;
      font-weight: 700;
      letter-spacing: 0.08em;
      text-transform: uppercase;
      cursor: pointer;
      transition: all 0.15s ease;
    }}
    .summary-continue-btn:hover {{
      background: #1c2b3d;
      border-color: var(--ops-cyan);
    }}

    @keyframes battery-critical-pulse {{ 50% {{ filter: drop-shadow(0 0 10px #d8645d); opacity: .55; }} }}
    .battery-critical {{ animation: battery-critical-pulse 0.85s steps(2, end) infinite; }}
    .map-battery-indicator {{ width:54px; pointer-events:none; font-family:var(--font-mono); }}
    .map-battery-label {{ color:#d8e4e8; font-size:8px; line-height:9px; text-align:center; text-shadow:0 1px 2px #000; }}
    .map-battery-track {{ height:4px; border:1px solid #395460; background:#05080c; margin-top:2px; }}
    .map-battery-fill {{ height:100%; width:100%; transition:width .25s linear,background-color .25s linear; }}
  </style>
</head>
<body>

<header>
  <div class="header-brand">
    <div class="header-logo"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><circle cx="12" cy="12" r="10"/><line x1="22" y1="12" x2="18" y2="12"/><line x1="6" y1="12" x2="2" y2="12"/><line x1="12" y1="6" x2="12" y2="2"/><line x1="12" y1="22" x2="12" y2="18"/></svg></div>
    <div>
      <div class="header-title">THE LIVING MAP // INCIDENT COMMAND POST</div>
    </div>
  </div>
  <div class="header-badges">
    <span id="clock-badge" class="badge badge-mono">UTC --:--:--</span>
    <span id="last-update" class="badge badge-mono">—</span>
    <span class="badge badge-live"><span class="badge-live-dot"></span>LIVE TELEMETRY</span>
    <span class="badge badge-osm">OpenStreetMap (Standard)</span>
    <span class="badge badge-dist">BASELINE: {distance_km} KM</span>
  </div>
</header>

<div class="main-layout">
  <!-- Left Command Sidebar with Independent Scroll -->
  <aside class="sidebar">
    <!-- ── Connection Strength Gauge ── -->
    <div class="conn-gauge-section" id="conn-gauge-section">
      <div class="conn-gauge-header">
        <div class="conn-gauge-title">
          <span class="conn-gauge-title-icon"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M4.9 19.1C1 15.2 1 8.8 4.9 4.9"/><path d="M7.8 16.2c-2.3-2.3-2.3-6.1 0-8.5"/><circle cx="12" cy="12" r="2"/><path d="M16.2 7.8c2.3 2.3 2.3 6.1 0 8.5"/><path d="M19.1 4.9C23 8.8 23 15.1 19.1 19"/></svg></span>
          WRITER TO ONA TELEMETRY LINK
        </div>
        <div class="conn-gauge-value green" id="conn-gauge-value">—%</div>
      </div>
      <div class="conn-gauge-track">
        <div class="conn-gauge-fill green" id="conn-gauge-fill" style="width:0%"></div>
      </div>
      <div class="scale-ticks"><span>0%</span><span>25%</span><span>50%</span><span>75%</span><span>100%</span></div>
      <div class="conn-gauge-meta">
        <span class="conn-gauge-state nominal" id="conn-gauge-state">INITIALIZING…</span>
        <span class="conn-gauge-dist" id="conn-gauge-dist">—</span>
      </div>
    </div>

    <!-- Enabled only by the isolated Zombie Link heartbeat-loss demo. -->
    <div class="conn-gauge-section heartbeat-section" id="heartbeat-section">
      <div class="heartbeat-title-row">
        <div class="conn-gauge-title"><span class="heartbeat-led" id="heartbeat-led"></span> ONA ↔ CP LINK TRUST</div>
        <div class="conn-gauge-value green" id="heartbeat-count">0 RX</div>
      </div>
      <div class="heartbeat-state" id="heartbeat-state">HEALTHY</div>
      <div class="heartbeat-gap" id="heartbeat-gap">Waiting for heartbeat telemetry…</div>
      <div class="heartbeat-data-proof" id="heartbeat-data-proof">BEACON CARRY: MONITORING</div>
    </div>

    <!-- Enabled only by the Signal Integrity Showcase. -->
    <div class="conn-gauge-section integrity-section" id="integrity-section">
      <div class="conn-gauge-title">SIGNAL INTEGRITY PROOF</div>
      <div class="integrity-grid">
        <div class="integrity-cell"><div class="integrity-label">SENT</div><div class="integrity-value" id="integrity-sent">0</div></div>
        <div class="integrity-cell"><div class="integrity-label">DELIVERED</div><div class="integrity-value" id="integrity-delivered">0</div></div>
        <div class="integrity-cell"><div class="integrity-label">LOST</div><div class="integrity-value" id="integrity-lost">0</div></div>
        <div class="integrity-cell"><div class="integrity-label">CORRUPTED</div><div class="integrity-value" id="integrity-corrupted">0</div></div>
      </div>
      <div class="integrity-phase" id="integrity-phase">SATELLITE PRIMARY</div>
    </div>

    <!-- ── Battery Telemetry Bay ── -->
    <div class="conn-gauge-section" id="battery-gauge-section">
      <div class="conn-gauge-header">
        <div class="conn-gauge-title">
          <span class="conn-gauge-title-icon"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><rect width="16" height="10" x="2" y="7" rx="1"/><line x1="22" x2="22" y1="11" y2="13"/></svg></span>
          WRITER SCOUT 1 POWER
        </div>
        <div class="conn-gauge-value green" id="writer-battery-value">100%</div>
      </div>
      <div class="conn-gauge-track">
        <div class="conn-gauge-fill green" id="writer-battery-fill" style="width:100%"></div>
      </div>
      <div class="conn-gauge-meta"><span class="conn-gauge-state nominal" id="writer-battery-state">NOMINAL</span></div>

      <div class="conn-gauge-header" style="margin-top:10px">
        <div class="conn-gauge-title">
          <span class="conn-gauge-title-icon"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><rect width="16" height="10" x="2" y="7" rx="1"/><line x1="22" x2="22" y1="11" y2="13"/></svg></span>
          EXECUTOR UNIT 2 POWER
        </div>
        <div class="conn-gauge-value green" id="executor-battery-value">100%</div>
      </div>
      <div class="conn-gauge-track">
        <div class="conn-gauge-fill green" id="executor-battery-fill" style="width:100%"></div>
      </div>
      <div class="conn-gauge-meta"><span class="conn-gauge-state nominal" id="executor-battery-state">NOMINAL</span></div>
    </div>

    <!-- Tactical Geography -->
    <div class="sidebar-section" id="tactical-geography-panel">
      <div class="section-title"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><circle cx="12" cy="12" r="10"/><line x1="22" y1="12" x2="18" y2="12"/><line x1="6" y1="12" x2="2" y2="12"/><line x1="12" y1="6" x2="12" y2="2"/><line x1="12" y1="22" x2="12" y2="18"/></svg> Tactical Geography & Fix</div>
      <div class="geo-card">
        <div class="geo-row">
          <span class="geo-label">Command Post HQ:</span>
          <span class="geo-val">{cp_lat:.4f}° N, {cp_lon:.4f}° E</span>
        </div>
        <div class="geo-row">
          <span class="geo-label">Disaster Ground Zero:</span>
          <span class="geo-val">{building_lat:.4f}° N, {building_lon:.4f}° E</span>
        </div>
        <div class="geo-row">
          <span class="geo-label">Direct Separation:</span>
          <span class="geo-val" style="color:var(--ops-cyan)">Satellite link | 650 ms model</span>
        </div>
        <div class="geo-dist-bar"><div class="geo-dist-fill"></div></div>
      </div>
    </div>

    <!-- Stats -->
    <div class="sidebar-section" id="incident-counters-panel">
      <div class="section-title"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M4.9 19.1C1 15.2 1 8.8 4.9 4.9"/><path d="M7.8 16.2c-2.3-2.3-2.3-6.1 0-8.5"/><circle cx="12" cy="12" r="2"/><path d="M16.2 7.8c2.3 2.3 2.3 6.1 0 8.5"/><path d="M19.1 4.9C23 8.8 23 15.1 19.1 19"/></svg> Zone Incident Counters</div>
      <div class="stats-grid">
        <div class="stat-card">
          <div class="stat-value" id="stat-total">0</div>
          <div class="stat-label">Events Detected</div>
        </div>
        <div class="stat-card">
          <div class="stat-value" id="stat-active">0</div>
          <div class="stat-label">Active Beacons</div>
        </div>
        <div class="stat-card">
          <div class="stat-value" id="stat-victims" style="color:var(--status-critical)">0</div>
          <div class="stat-label">Victim Sites</div>
        </div>
        <div class="stat-card">
          <div class="stat-value" id="stat-fire" style="color:#ea580c">0</div>
          <div class="stat-label">Fire / Hazard</div>
        </div>
      </div>
    </div>

    <!-- Executor Unit Selection -->
    <div class="sidebar-section" id="unit-roster-panel">
      <div class="section-title"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><circle cx="12" cy="12" r="10"/><line x1="22" y1="12" x2="18" y2="12"/><line x1="6" y1="12" x2="2" y2="12"/><line x1="12" y1="6" x2="12" y2="2"/><line x1="12" y1="22" x2="12" y2="18"/></svg> Tactical Response Units</div>
      <div class="unit-roster" id="unit-roster">
        <div style="color:var(--ops-text-muted);font-size:11px;text-align:center;padding:10px 0;font-family:var(--font-mono);">AWAITING ROSTER DOWNLINK…</div>
      </div>
    </div>

    <!-- Mission Control & Briefing -->
    <div class="sidebar-section" id="mission-control-panel">
      <div class="section-title"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M4.9 19.1C1 15.2 1 8.8 4.9 4.9"/><path d="M7.8 16.2c-2.3-2.3-2.3-6.1 0-8.5"/><circle cx="12" cy="12" r="2"/><path d="M16.2 7.8c2.3 2.3 2.3 6.1 0 8.5"/><path d="M19.1 4.9C23 8.8 23 15.1 19.1 19"/></svg> Mission Briefing (ONA Downlink)</div>
      <div class="mission-briefing-card" id="mission-briefing-card">
        <div class="briefing-header">
          <span class="briefing-badge">MISSION BRIEFING</span>
          <span class="briefing-status" id="briefing-status">AWAITING</span>
        </div>
        <div id="briefing-details">
          <div style="color:var(--ops-text-muted);font-size:11px;text-align:center;padding:10px 0;font-family:var(--font-mono);">
            Awaiting mission downlink from Outside Network Area…
          </div>
        </div>
      </div>
      <button class="send-mission-btn" id="send-mission-btn" disabled onclick="sendMission()">
        [ AUTHORIZE & DISPATCH MISSION ]
      </button>
      <div id="ona-brief-progress" style="margin-top:7px;padding:7px 8px;border:1px solid rgba(148,163,184,.28);background:rgba(15,23,42,.64);color:var(--ops-text-muted);font:10px/1.35 var(--font-mono);letter-spacing:.03em;">
        STEP 1/2: WAIT FOR A MISSION BRIEFING
      </div>
      <div class="exec-status-label idle" id="exec-status-label">EXECUTOR-2: IDLE</div>
      <div class="mission-panel" id="mission-panel">
        <div class="section-title" style="margin-bottom:6px">Latest Briefing RAW</div>
        <pre id="mission-content">—</pre>
      </div>
    </div>

    <!-- Event List -->
    <div class="sidebar-section" id="beacon-feed-panel" style="flex:1">
      <div class="section-title"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M4.9 19.1C1 15.2 1 8.8 4.9 4.9"/><path d="M7.8 16.2c-2.3-2.3-2.3-6.1 0-8.5"/><circle cx="12" cy="12" r="2"/><path d="M16.2 7.8c2.3 2.3 2.3 6.1 0 8.5"/><path d="M19.1 4.9C23 8.8 23 15.1 19.1 19"/></svg> Live Beacon SITREP Feed</div>
      <div class="event-list" id="event-list">
        <div style="color:var(--ops-text-muted);font-size:11px;text-align:center;padding:16px 0;font-family:var(--font-mono);">
          Waiting for beacon telemetry via Outside Network Area…
        </div>
      </div>
    </div>
  </aside>

  <!-- Tactical Map Theater -->
  <div class="map-container">
    <div class="map-hud-coords">GRID-REF: SECTOR-ALPHA // LAT {cp_lat:.4f}° N // LON {cp_lon:.4f}° E</div>
    <div id="map"></div>
    <button class="communication-console-btn" type="button" onclick="openCommunicationConsole()" title="Open the detailed packet and beacon tables without leaving the dashboard">
      <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><polyline points="4 17 10 11 4 5"/><line x1="12" x2="20" y1="19" y2="19"/></svg> OPEN COMMUNICATION CONSOLE
    </button>
    {webots_button_html}
    {webots_vision_button_html}
    <div class="continuity-overlay" id="continuity-overlay">
      <div class="continuity-banner" id="continuity-banner">
        <span class="continuity-banner-text" id="continuity-banner-text"></span>
        <button class="continuity-skip-btn" onclick="dismissContinuityDemo()">Skip ✕</button>
      </div>
    </div>
    <div class="failover-overlay" id="failover-overlay" role="status" aria-live="assertive">
      <div class="failover-banner">
        <div class="failover-banner-icon">⚠</div>
        <div>
          <div class="failover-banner-title">Satellite link failure detected</div>
          <div class="failover-banner-detail">ONA ROLE 3 — SWITCHING TO RF MESH MODE</div>
        </div>
      </div>
    </div>
    <div class="baseline-failure-overlay" id="baseline-failure-overlay" role="alert" aria-live="assertive">
      <div class="baseline-failure-card">
        <div class="baseline-failure-icon">⚠</div>
        <div class="baseline-failure-title" id="baseline-failure-title">CONNECTION FAILED</div>
        <div class="baseline-failure-detail" id="baseline-failure-detail"></div>
        <div class="baseline-failure-note">BASELINE DEMO — RESILIENCE SOLUTION NOT APPLIED</div>
      </div>
    </div>
    <div class="map-overlay">
      <div class="overlay-title">Tactical Map Legend</div>
      <div class="legend-item"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M4 22h16"/><path d="M7 11h10"/><path d="M12 2v20"/><path d="m17 7-5-5-5 5"/><circle cx="12" cy="7" r="1"/></svg> <span class="legend-label">Command Post HQ [CP-ALPHA]</span></div>
      <div class="legend-item"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="m21.73 18-8-14a2 2 0 0 0-3.48 0l-8 14A2 2 0 0 0 4 21h16a2 2 0 0 0 1.73-3Z"/><line x1="12" y1="9" x2="12" y2="13"/><line x1="12" y1="17" x2="12.01" y2="17"/></svg> <span class="legend-label">Disaster Zone Alpha [Origin]</span></div>
      <div class="legend-item">
        <svg width="15" height="17" viewBox="0 0 38 42" style="flex-shrink:0"><polygon points="19,2 35,11 35,31 19,40 3,31 3,11" fill="#d97706" stroke="#fbbf24" stroke-width="3"/></svg>
        <span class="legend-label">Writer-1 (Scout | Disaster Site)</span>
      </div>
      <div class="legend-item">
        <svg width="15" height="17" viewBox="0 0 38 42" style="flex-shrink:0"><polygon points="19,2 35,7 35,27 19,40 3,27 3,7" fill="#1e40af" stroke="#60a5fa" stroke-width="3"/></svg>
        <span class="legend-label">Executor-2 (Intervention Unit)</span>
      </div>
      <div class="legend-item"><div class="legend-dot" style="background:#3b82f6;border:1px dashed #93c5fd"></div><span class="legend-label">Satellite Telemetry Link</span></div>
      <div class="legend-item"><div class="legend-dot" style="background:#ef4444"></div><span class="legend-label">Victim Detected</span></div>
      <div class="legend-item"><div class="legend-dot" style="background:#ea580c"></div><span class="legend-label">Fire Detected</span></div>
    </div>
    <button class="map-reset-btn" onclick="fitTacticalView()">
      <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><polyline points="15 3 21 3 21 9"/><polyline points="9 21 3 21 3 15"/><line x1="21" x2="14" y1="3" y2="10"/><line x1="3" x2="10" y1="21" y2="14"/></svg> FIT TACTICAL OVERVIEW ({distance_km} KM)
    </button>
  </div>

  <!-- Showcase-only right-side explanation dock; it reserves space instead of covering the map. -->
  <aside class="showcase-subtitle" id="showcase-subtitle" aria-live="polite">
    <div class="showcase-subtitle-header">LIVE EXPLANATION</div>
    <div class="showcase-subtitle-rule"></div>
    <div class="showcase-subtitle-body" id="showcase-subtitle-body"></div>
    <div class="showcase-subtitle-footer">EVENT-DRIVEN SHOWCASE NARRATION</div>
  </aside>

</div>

<div class="communication-console-backdrop" id="communication-console-backdrop" role="dialog" aria-modal="true" aria-label="Live communication console">
  <div class="communication-console-dialog" id="communication-console-dialog">
    <div class="communication-console-dialog-header">
      <span>LIVE COMMUNICATION CONSOLE — PACKETS AND BEACON REGISTER</span>
      <button class="communication-console-close" type="button" onclick="closeCommunicationConsole()">Close Console ✕</button>
    </div>
    <iframe class="communication-console-frame" id="communication-console-frame" title="Live communication console"></iframe>
  </div>
</div>

<!-- ── Mission Summary Modal ── -->
<div class="summary-backdrop" id="summary-backdrop">
  <div class="summary-modal" id="summary-modal" role="dialog" aria-modal="true" aria-labelledby="summary-title">
    <div class="summary-header">
      <div class="summary-header-left">
        <div class="summary-badge">MISSION CYCLE COMPLETE</div>
        <div class="summary-title" id="summary-title">Tactical Mission Debrief</div>
        <div class="summary-meta" id="summary-meta">—</div>
      </div>
      <button class="summary-close-btn" onclick="dismissSummary()" title="Dismiss">✕</button>
    </div>

    <div class="summary-sections" id="summary-sections">
      <!-- Populated by JS -->
    </div>

    <div class="summary-footer">
      <div class="summary-countdown" id="summary-countdown">Auto-closing in 15s…</div>
      <button class="summary-export-btn" id="summary-export-btn" onclick="exportMissionPDF()"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M14.5 2H6a2 2 0 0 0-2 2v16a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V7.5L14.5 2z"/><polyline points="14 2 14 8 20 8"/><path d="M12 18v-6"/><path d="m9 15 3 3 3-3"/></svg> Export Mission PDF Report</button>
      <button class="summary-continue-btn" onclick="dismissSummary()">Continue to Next Cycle</button>
    </div>
  </div>
</div>

<script>

// ── Live Mission Clock & Lucide Icon System ────────────────────────────────
function updateMissionClock() {{
  const clockEl = document.getElementById('clock-badge');
  if (!clockEl) return;
  const now = new Date();
  const utc = now.toISOString().substring(11, 19);
  clockEl.textContent = `UTC ${{utc}}`;
}}
setInterval(updateMissionClock, 1000);
updateMissionClock();

function initLucideIcons() {{
  if (window.lucide && typeof window.lucide.createIcons === 'function') {{
    window.lucide.createIcons();
  }}
}}
window.addEventListener('DOMContentLoaded', initLucideIcons);
setTimeout(initLucideIcons, 100);
setTimeout(initLucideIcons, 800);

// ── Communication Console ───────────────────────────────────────────────────
function openCommunicationConsole() {{
  const backdrop = document.getElementById('communication-console-backdrop');
  const frame = document.getElementById('communication-console-frame');
  if (!backdrop || !frame) return;
  frame.src = '/communications';
  backdrop.classList.add('visible');
}}

function closeCommunicationConsole() {{
  const backdrop = document.getElementById('communication-console-backdrop');
  const frame = document.getElementById('communication-console-frame');
  if (backdrop) backdrop.classList.remove('visible');
  if (frame) frame.src = 'about:blank';
}}

document.addEventListener('keydown', (event) => {{
  if (event.key === 'Escape') closeCommunicationConsole();
}});

// ── Coordinates (Configured with {distance_km} km separation) ────────────────
const CP_LAT = {cp_lat};
const CP_LON = {cp_lon};
const BUILDING_LAT = {building_lat};
const BUILDING_LON = {building_lon};
const WRITER_LAT = {writer_lat};
const WRITER_LON = {writer_lon};
const EXECUTOR_LAT = {executor_lat} + 0.0010; // increased offset for better visual separation
const EXECUTOR_LON = {executor_lon};
const DISTANCE_KM = {distance_km};
const ROVER_SEP_KM = {rover_sep_km};
const EXECUTOR_UNITS = {executor_units_json};

// ── Connection Strength Gauge ─────────────────────────────────────────────────
// Polls /api/signal_strength every 300ms and updates the gauge bar + label.
// Uses the server-computed RSS path-loss value (strength = max(0, 100 - k * dist_km))
// and the server-side link_state, which reads satellite pass/weather availability.
let _prevGaugeZone = null;   // track zone transitions for threshold-crossing logs
let linkQualityRing = null;
let failoverBlackoutObserved = false;
let rfMeshTransportMode = null;
let failoverNoticeShown = false;
let failoverNoticeTimer = null;
let pendingFailoverMeshSnapshot = null;

async function updateSignalGauge() {{
  try {{
    const resp = await fetch('/api/signal_strength');
    if (!resp.ok) return;
    const d = await resp.json();

    const strength = d.strength ?? 0;
    const zone = d.zone ?? 'red';
    const state = d.link_state ?? 'nominal';
    const distKm = d.dist_km ?? 0;
    const linkQuality = d.link_quality;
    const qualityLabel = Number.isFinite(linkQuality) ? ` · Q ${{(linkQuality * 100).toFixed(0)}}%` : '';
    updateLinkQualityOverlay(linkQuality, d.writer_lat, d.writer_lon);

    // ── Update gauge fill bar ──────────────────────────────────────────────
    const fillEl = document.getElementById('conn-gauge-fill');
    const valueEl = document.getElementById('conn-gauge-value');
    const stateEl = document.getElementById('conn-gauge-state');
    const distEl = document.getElementById('conn-gauge-dist');

    if (!fillEl || !valueEl || !stateEl || !distEl) return;

    // Set bar width and colour class
    fillEl.style.width = strength.toFixed(1) + '%';
    fillEl.className = 'conn-gauge-fill ' + zone;

    valueEl.textContent = strength.toFixed(0) + '%';
    valueEl.className = 'conn-gauge-value ' + zone;

    if (state === 'blackout') {{
      stateEl.textContent = '⚠ LINK DOWN';
      stateEl.className = 'conn-gauge-state blackout';
    }} else if (zone === 'red') {{
      stateEl.textContent = '▼ DEGRADED' + qualityLabel;
      stateEl.className = 'conn-gauge-state blackout';  // red colour matches
    }} else if (zone === 'yellow') {{
      stateEl.textContent = '~ MARGINAL' + qualityLabel;
      stateEl.className = 'conn-gauge-state';
      stateEl.style.color = '#eab308';
    }} else {{
      stateEl.textContent = '● NOMINAL' + qualityLabel;
      stateEl.className = 'conn-gauge-state nominal';
      stateEl.style.color = '';
    }}

    distEl.textContent = distKm.toFixed(2) + ' km';

    // This is driven only by the real satellite state.  Retrying while the
    // outage persists protects the visible notice from a transient status-poll
    // failure at the exact handover moment.
    if (state === 'blackout') {{
      failoverBlackoutObserved = true;
      maybeShowFailoverNotice();
    }}

    // ── Threshold-crossing log (mirrors existing [COMMAND POST] log format) ──
    if (_prevGaugeZone !== zone) {{
      const ts = d.ts || new Date().toLocaleTimeString();
      if (zone === 'red' && _prevGaugeZone !== null) {{
        // Entering red zone
        console.warn(
          `[${{ts}}] [COMMAND POST] Connection strength critical — link degraded ` +
          `(${{strength.toFixed(0)}}% @ ${{distKm.toFixed(2)}} km) [zone: ${{_prevGaugeZone}} → red]`
        );
      }} else if (zone === 'green' && (_prevGaugeZone === 'red' || _prevGaugeZone === 'yellow')) {{
        // Recovering back to green
        console.info(
          `[${{ts}}] [COMMAND POST] Connection strength restored — link healthy ` +
          `(${{strength.toFixed(0)}}% @ ${{distKm.toFixed(2)}} km) [zone: ${{_prevGaugeZone}} → green]`
        );
      }}
      _prevGaugeZone = zone;
    }}
  }} catch(e) {{
    // Transient error — do not crash the gauge loop
    console.warn('[SIGNAL GAUGE] Poll error:', e);
  }}
}}

// Start gauge polling immediately and sustain every 300ms
updateSignalGauge();
setInterval(updateSignalGauge, 300);

// ── Zombie Link heartbeat trust monitor (separate from signal strength) ──────
async function updateHeartbeatTrust() {{
  try {{
    const response = await fetch('/api/heartbeat', {{ cache: 'no-store' }});
    if (!response.ok) return;
    const data = await response.json();
    const section = document.getElementById('heartbeat-section');
    if (!section) return;
    if (!data.enabled) {{ section.classList.remove('enabled', 'degraded'); return; }}

    const state = data.state || 'healthy';
    const degraded = state === 'degraded_unconfirmed';
    const gap = Number(data.gap_s || 0).toFixed(1);
    const threshold = Number(data.threshold_s || 0).toFixed(1);
    section.classList.add('enabled');
    section.classList.toggle('degraded', degraded);
    document.getElementById('heartbeat-count').textContent = `${{data.received_count || 0}} RX`;
    document.getElementById('heartbeat-state').textContent = degraded
      ? '▲ DEGRADED / UNCONFIRMED'
      : state === 'awaiting_heartbeat' ? '~ HEARTBEAT GAP GROWING' : '● HEALTHY / CONFIRMED';
    document.getElementById('heartbeat-gap').textContent =
      `${{gap}}s since last heartbeat · threshold ${{threshold}}s · ${{data.suppressed_count || 0}} suppressed`;
    const dataProof = document.getElementById('heartbeat-data-proof');
    dataProof.textContent = degraded
      ? `BEACON CARRY STILL FLOWING · ${{data.data_while_degraded || 0}} ARRIVAL(S) WHILE DEGRADED`
      : `BEACON CARRY: ${{data.data_arrivals || 0}} COMMAND POST ARRIVAL(S)`;
  }} catch (_) {{ /* Keep the normal dashboard alive if heartbeat telemetry is absent. */ }}
}}

async function launchWebotsVisualization() {{
  const button = document.getElementById('webots-visualization-btn');
  if (!button) return;
  const original = button.innerHTML;
  button.disabled = true;
  button.textContent = 'LAUNCHING WEBOTS…';
  try {{
    const response = await fetch('/api/launch/webots_visualization', {{ method: 'POST' }});
    const result = await response.json();
    if (!response.ok || result.status === 'error') throw new Error(result.message || 'Webots launch failed.');
    button.textContent = result.status === 'already_running' ? 'WEBOTS ALREADY RUNNING' : 'WEBOTS VISUALIZATION ACTIVE';
  }} catch (error) {{
    button.textContent = `WEBOTS ERROR: ${{error.message || 'LAUNCH FAILED'}}`;
    button.title = error.message || 'Webots launch failed';
    window.setTimeout(() => {{ button.innerHTML = original; button.disabled = false; }}, 5000);
    return;
  }}
}}

function openWebotsVision() {{
  window.open('/webots_vision', 'living-map-webots-vision', 'width=720,height=610,resizable=yes');
}}

updateHeartbeatTrust();
setInterval(updateHeartbeatTrust, 200);

// ── Signal Integrity Showcase: payload evidence across every failure phase ──
let lastShowcaseSubtitle = null;
async function updateSignalIntegrity() {{
  try {{
    const response = await fetch('/api/signal_integrity', {{ cache: 'no-store' }});
    if (!response.ok) return;
    const data = await response.json();
    const section = document.getElementById('integrity-section');
    if (!section) return;
    const subtitle = document.getElementById('showcase-subtitle');
    const subtitleBody = document.getElementById('showcase-subtitle-body');
    const layout = document.querySelector('.main-layout');
    if (!data.enabled) {{
      section.classList.remove('enabled');
      if (subtitle) subtitle.classList.remove('enabled', 'changing');
      if (layout) layout.classList.remove('showcase-subtitle-active');
      lastShowcaseSubtitle = null;
      return;
    }}
    section.classList.add('enabled');
    document.getElementById('integrity-sent').textContent = data.sent ?? 0;
    document.getElementById('integrity-delivered').textContent = data.delivered ?? 0;
    document.getElementById('integrity-lost').textContent = data.lost ?? 0;
    document.getElementById('integrity-corrupted').textContent = data.corrupted ?? 0;
    document.getElementById('integrity-phase').textContent = data.phase || 'SIGNAL INTEGRITY ACTIVE';
    if (subtitle && subtitleBody && data.subtitle) {{
      if (layout) layout.classList.add('showcase-subtitle-active');
      if (data.subtitle !== lastShowcaseSubtitle) {{
        subtitle.classList.add('changing');
        window.setTimeout(() => {{
          subtitleBody.textContent = data.subtitle;
          subtitle.classList.add('enabled');
          subtitle.classList.remove('changing');
        }}, 150);
        lastShowcaseSubtitle = data.subtitle;
      }}
    }}
  }} catch (_) {{ /* Optional showcase telemetry must not affect normal demos. */ }}
}}
updateSignalIntegrity();
setInterval(updateSignalIntegrity, 200);

// ── Persistent comparison-baseline failure screen ────────────────────────────
let baselineFailureScreenShown = false;
async function updateBaselineFailureScreen() {{
  try {{
    const response = await fetch('/api/baseline_alert', {{ cache: 'no-store' }});
    if (!response.ok) return;
    const data = await response.json();
    if (!data.enabled || !data.active || baselineFailureScreenShown) return;
    baselineFailureScreenShown = true;
    document.getElementById('baseline-failure-title').textContent = data.title || 'CONNECTION FAILED';
    document.getElementById('baseline-failure-detail').textContent = data.detail || '';
    document.getElementById('baseline-failure-overlay').classList.add('visible');
  }} catch (_) {{ /* Baseline alert is optional and must not affect the map. */ }}
}}
updateBaselineFailureScreen();
setInterval(updateBaselineFailureScreen, 200);

// ── Leaflet Open-Source Tile Layers (100% Free, Zero API Keys) ────────────────
// 1. Standard OpenStreetMap raster tiles (vibrant, rich topographic detail)
const osmStandard = L.tileLayer('https://tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
  maxZoom: 19,
  attribution: '&copy; <a href="https://www.openstreetmap.org/copyright" target="_blank">OpenStreetMap</a> contributors'
}});

// 2. CartoDB Dark Matter tiles (optional dark mode alternate)
const cartoDark = L.tileLayer('https://{{s}}.basemaps.cartocdn.com/dark_all/{{z}}/{{x}}/{{y}}{{r}}.png', {{
  subdomains: 'abcd',
  maxZoom: 20,
  attribution: '&copy; OpenStreetMap contributors &copy; CARTO'
}});

// Initialize Map: Defaulting to Tactical Dark CartoDB
const map = L.map('map', {{
  center: [(CP_LAT + BUILDING_LAT) / 2, (CP_LON + BUILDING_LON) / 2],
  zoom: 13,
  layers: [osmStandard],
  zoomControl: true,
  preferCanvas: false
}});

// Layer control for switching tiles
L.control.layers({{
  "OpenStreetMap (Standard)": osmStandard,
  "Tactical Dark (CartoDB)": cartoDark
}}, null, {{ position: 'topleft' }}).addTo(map);

// ── 1. Tactical Command Post Marker ──────────────────────────────────────────
const cpIcon = L.divIcon({{
  html: `<div class="tactical-marker-pin">
          <div class="tactical-pin-box" style="background:#09121a;border:1.5px solid #0284c7;box-shadow:0 0 12px rgba(2,132,199,0.5);">
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="#38bdf8" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M4 22h16"/><path d="M7 11h10"/><path d="M12 2v20"/><path d="m17 7-5-5-5 5"/><circle cx="12" cy="7" r="1"/></svg>
          </div>
          <div class="tactical-pin-label" style="border:1px solid #1e3a8a;color:#93c5fd;">CP-HQ [ALPHA]</div>
         </div>`,
  iconSize: [44, 60],
  iconAnchor: [22, 30],
  className: 'cp-marker'
}});

const cpMarker = L.marker([CP_LAT, CP_LON], {{ icon: cpIcon }}).addTo(map);
cpMarker.bindPopup(`
  <div style="font-family:var(--font-sans);min-width:230px">
    <div class="tactical-popup-title" style="color:#38bdf8;border-left:3px solid #0284c7;padding-left:6px;">INCIDENT COMMAND POST</div>
    <div class="tactical-popup-sub">Regional Tactical Base & Satellite Gateway</div>
    <div class="tactical-popup-row"><b>GPS:</b> ${{CP_LAT.toFixed(6)}}° N, ${{CP_LON.toFixed(6)}}° E</div>
    <div class="tactical-popup-row"><b>Satellite Link:</b> Pass window available · 650ms</div>
    <div class="tactical-popup-status" style="color:#10b981;">● Tactical Network Online</div>
  </div>
`);

// ── 2. Disaster Zone Alpha (Building Exploration Site) ────────────────────────
const zoneCircle = L.circle([BUILDING_LAT, BUILDING_LON], {{
  color: '#dc2626',
  fillColor: '#ef4444',
  fillOpacity: 0.18,
  radius: 120,
  weight: 2,
  dashArray: '6, 6'
}}).addTo(map);

const buildingIcon = L.divIcon({{
  html: `<div class="tactical-marker-pin">
          <div class="tactical-pin-box" style="background:#180b0b;border:1.5px solid #dc2626;box-shadow:0 0 12px rgba(220,38,38,0.5);">
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="#ef4444" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="m21.73 18-8-14a2 2 0 0 0-3.48 0l-8 14A2 2 0 0 0 4 21h16a2 2 0 0 0 1.73-3Z"/><line x1="12" y1="9" x2="12" y2="13"/><line x1="12" y1="17" x2="12.01" y2="17"/></svg>
          </div>
          <div class="tactical-pin-label" style="border:1px solid #7f1d1d;color:#fca5a5;">DISASTER ZONE [ALPHA]</div>
         </div>`,
  iconSize: [40, 54],
  iconAnchor: [20, 27],
  className: 'building-marker'
}});

const buildingMarker = L.marker([BUILDING_LAT, BUILDING_LON], {{ icon: buildingIcon }}).addTo(map);
buildingMarker.bindPopup(`
  <div style="font-family:var(--font-sans);min-width:230px">
    <div class="tactical-popup-title" style="color:#ef4444;border-left:3px solid #dc2626;padding-left:6px;">DISASTER ZONE ALPHA</div>
    <div class="tactical-popup-sub">Collapsed Commercial Structure (Exploration Site)</div>
    <div class="tactical-popup-row"><b>Anchor GPS:</b> ${{BUILDING_LAT.toFixed(6)}}° N, ${{BUILDING_LON.toFixed(6)}}° E</div>
    <div class="tactical-popup-row"><b>Distance to CP:</b> ${{DISTANCE_KM}} km</div>
    <div class="tactical-popup-status" style="color:#f59e0b;">⚠ GPS-Denied Interior Enclave</div>
  </div>
`);

// ── 3. Tactical Rover Fleet (Distinct Car Markers) ───────────────────────────
// Writer Robot 1: Amber Off-Road Scout Car Icon
const writerIcon = L.divIcon({{
  html: `<div style="display:flex;flex-direction:column;align-items:center;">
          <svg width="40" height="38" viewBox="0 0 48 40" fill="none" xmlns="http://www.w3.org/2000/svg" style="filter:drop-shadow(0 0 8px rgba(245,158,11,0.95));">
            <!-- Car Body -->
            <path d="M6 24 C6 21 8 19 11 19 L15 11 C16.5 8.5 19 7 22 7 L32 7 C35 7 37 8.5 38.5 11 L42 19 C44 19 46 21 46 24 L46 29 C46 30.5 44.5 32 43 32 L5 32 C3.5 32 2 30.5 2 29 Z" fill="#d97706" stroke="#fbbf24" stroke-width="2"/>
            <!-- Windshield & Windows -->
            <path d="M16 18 L19 11 L29 11 L31 18 Z" fill="#0f172a" stroke="#fbbf24" stroke-width="1.2"/>
            <!-- Wheels -->
            <circle cx="11" cy="31" r="5" fill="#1e293b" stroke="#fbbf24" stroke-width="2"/>
            <circle cx="11" cy="31" r="2" fill="#ffffff"/>
            <circle cx="37" cy="31" r="5" fill="#1e293b" stroke="#fbbf24" stroke-width="2"/>
            <circle cx="37" cy="31" r="2" fill="#ffffff"/>
            <!-- Roof Sensor Antenna -->
            <path d="M24 7 L24 2" stroke="#ffffff" stroke-width="1.8" stroke-linecap="round"/>
            <circle cx="24" cy="2" r="2" fill="#ffffff"/>
          </svg>
          <div style="background:#070a0f;color:#fbbf24;font-size:9px;font-weight:700;padding:2px 6px;border-radius:var(--radius-sm);border:1px solid #d97706;margin-top:2px;white-space:nowrap;box-shadow:0 2px 6px rgba(0,0,0,0.6);font-family:var(--font-mono);letter-spacing:0.04em;">SCOUT-01 [WRITER]</div>
         </div>`,
  iconSize: [44, 52],
  iconAnchor: [22, 26],
  className: 'writer-marker'
}});

function executorUnitIcon(unit) {{
  return L.divIcon({{
    html: `<div style="display:flex;flex-direction:column;align-items:center;">
            <div role="img" aria-label="${{unit.display_name}}" style="width:40px;height:38px;background:${{unit.icon_tint}};mask:url('${{unit.icon_url}}') center/contain no-repeat;-webkit-mask:url('${{unit.icon_url}}') center/contain no-repeat;filter:drop-shadow(0 0 8px ${{unit.icon_tint}});"></div>
            <div style="background:#0f172a;color:${{unit.icon_tint}};font-size:9px;font-weight:700;padding:2px 6px;border-radius:4px;border:1px solid ${{unit.icon_tint}};margin-top:2px;white-space:nowrap;box-shadow:0 2px 6px rgba(0,0,0,0.6);font-family:'JetBrains Mono',monospace;">${{unit.unit_id}}</div>
           </div>`,
    iconSize: [44, 52],
    iconAnchor: [22, 26],
    className: 'executor-marker',
  }});
}}

const writerMarker = L.marker([WRITER_LAT, WRITER_LON], {{ icon: writerIcon }}).addTo(map);
linkQualityRing = L.circle([WRITER_LAT, WRITER_LON], {{
  radius: 140,
  color: '#54b889',
  fillColor: '#54b889',
  weight: 2,
  opacity: 0.9,
  fillOpacity: 0.12,
  interactive: false,
}}).addTo(map).bindTooltip('Satellite link quality: awaiting telemetry', {{
  direction: 'top',
  permanent: false,
}});

function updateLinkQualityOverlay(quality, lat, lon) {{
  if (linkQualityRing === null || !Number.isFinite(quality)) return;
  const normalized = Math.max(0, Math.min(1, quality));
  const color = normalized > 0.66 ? '#54b889' : normalized > 0.33 ? '#e3a13d' : '#d8645d';
  linkQualityRing.setLatLng([lat, lon]);
  linkQualityRing.setRadius(80 + (1 - normalized) * 160);
  linkQualityRing.setStyle({{
    color,
    fillColor: color,
    opacity: 0.3 + normalized * 0.65,
    fillOpacity: 0.04 + normalized * 0.16,
  }});
  linkQualityRing.setTooltipContent(`Satellite link quality: ${{(normalized * 100).toFixed(0)}}%`);
}}

writerMarker.bindPopup(`
  <div style="font-family:var(--font-sans);min-width:230px">
    <div class="tactical-popup-title" style="color:#f59e0b;border-left:3px solid #d97706;padding-left:6px;">WRITER-01 · RECON SCOUT</div>
    <div class="tactical-popup-sub">On-Scene Exploration & In-Situ Sensing</div>
    <div class="tactical-popup-row"><b>Location:</b> Inside Disaster Zone Alpha</div>
    <div class="tactical-popup-row"><b>Coordinates:</b> ${{WRITER_LAT.toFixed(6)}}° N, ${{WRITER_LON.toFixed(6)}}° E</div>
    <div class="tactical-popup-status" style="color:#10b981;">● Deployed on scene at event origin</div>
  </div>
`);

const executorMarkers = {{}};
let executorMarker = null;
let selectedExecutorId = null;
function executorUnitPopup(unit) {{
  return `<div style="font-family:var(--font-sans);min-width:220px">
    <div class="tactical-popup-title" style="color:${{unit.icon_tint}};border-left:3px solid ${{unit.icon_tint}};padding-left:6px;">${{unit.display_name}}</div>
    <div class="tactical-popup-sub">${{unit.job_type}} tactical response unit</div>
    <div class="tactical-popup-row"><b>Unit ID:</b> ${{unit.unit_id}}</div>
    <div class="tactical-popup-row"><b>Battery:</b> ${{Number(unit.battery).toFixed(1)}}%</div>
    <div class="tactical-popup-status" style="color:${{unit.icon_tint}};">● Status: ${{unit.status}}</div>
  </div>`;
}}
function renderExecutorMarkers(units) {{
  const liveIds = new Set(units.map(unit => unit.unit_id));
  Object.entries(executorMarkers).forEach(([unitId, marker]) => {{
    if (!liveIds.has(unitId)) {{ map.removeLayer(marker); delete executorMarkers[unitId]; }}
  }});
  units.forEach(unit => {{
    let marker = executorMarkers[unit.unit_id];
    if (!marker) {{
      marker = L.marker([unit.lat, unit.lon], {{ icon: executorUnitIcon(unit) }}).addTo(map);
      executorMarkers[unit.unit_id] = marker;
    }}
    // The selected vehicle is animated from /api/executor/position at 100ms.
    // Do not snap it back to a slower roster snapshot while it is en route.
    const livePositionOwnsMarker = unit.selected && unit.status === 'EN_ROUTE';
    if (!livePositionOwnsMarker) marker.setLatLng([unit.lat, unit.lon]);
    marker.bindPopup(executorUnitPopup(unit));
    marker.getElement()?.classList.toggle('battery-critical', unit.battery < 20);
  }});
  const selected = units.find(unit => unit.selected);
  const displayUnit = selected || units[0];
  selectedExecutorId = selected ? selected.unit_id : null;
  if (displayUnit) {{
    executorMarker = executorMarkers[displayUnit.unit_id];
  }}
}}
renderExecutorMarkers(EXECUTOR_UNITS);
function batteryMapBar(label) {{ return L.divIcon({{className:'map-battery-indicator', iconSize:[54,16], iconAnchor:[27,42], html:`<div class="map-battery-label">${{label}}</div><div class="map-battery-track"><div class="map-battery-fill"></div></div>`}}); }}
const writerBatteryBar = L.marker([WRITER_LAT, WRITER_LON], {{icon:batteryMapBar('WR-1'), interactive:false, keyboard:false}}).addTo(map);
const executorBatteryBar = L.marker([EXECUTOR_LAT, EXECUTOR_LON], {{icon:batteryMapBar(EXECUTOR_UNITS[0]?.unit_id || 'EX-UNIT'), interactive:false, keyboard:false}}).addTo(map);

// ── 3a. ONA Role 3 CARRY — visible only while a beacon frame is in flight ──
let carryTransmissionLink = null;
let carryTransmissionPulse = null;
let carryTransmissionPacketId = null;
let carryTransmissionFrame = null;

function hideCarryTransmission() {{
  if (carryTransmissionFrame !== null) cancelAnimationFrame(carryTransmissionFrame);
  carryTransmissionFrame = null;
  carryTransmissionPacketId = null;
  if (carryTransmissionPulse) {{ map.removeLayer(carryTransmissionPulse); carryTransmissionPulse = null; }}
  if (carryTransmissionLink) {{ map.removeLayer(carryTransmissionLink); carryTransmissionLink = null; }}
}}

function carrySourceCoordinates(transmission) {{
  const source = transmission.source || {{}};
  const lat = Number(source.lat);
  const lon = Number(source.lon);
  if (Number.isFinite(lat) && Number.isFinite(lon)) return [lat, lon];
  const sourceMarker = executorMarkers[transmission.source_unit_id] || writerMarker;
  const position = sourceMarker.getLatLng();
  return [position.lat, position.lng];
}}

function animateCarryTransmission(packetId, start, end) {{
  let cycleStartedAt = performance.now();
  function step(now) {{
    if (carryTransmissionPacketId !== packetId || !carryTransmissionPulse) return;
    const progress = Math.min((now - cycleStartedAt) / 650, 1);
    carryTransmissionPulse.setLatLng([
      start[0] + (end[0] - start[0]) * progress,
      start[1] + (end[1] - start[1]) * progress,
    ]);
    if (progress >= 1) cycleStartedAt = now;
    carryTransmissionFrame = requestAnimationFrame(step);
  }}
  carryTransmissionFrame = requestAnimationFrame(step);
}}

function renderCarryTransmission(transmission) {{
  if (!transmission.active) {{ hideCarryTransmission(); return; }}
  const start = carrySourceCoordinates(transmission);
  const end = [CP_LAT, CP_LON];
  const sender = transmission.source_unit_id || 'beacon source';
  const eventType = (transmission.event_type || 'beacon frame').replace(/_/g, ' ');
  const tooltip = `ONA Role 3 CARRY active · ${{sender}} · ${{eventType}}`;

  if (!carryTransmissionLink) {{
    carryTransmissionLink = L.polyline([start, end], {{
      color: '#3b82f6', weight: 3.5, dashArray: '8, 6', opacity: 0.95,
    }}).addTo(map).bindTooltip(tooltip, {{ permanent: false, direction: 'center' }});
  }} else {{
    carryTransmissionLink.setLatLngs([start, end]);
    carryTransmissionLink.setTooltipContent(tooltip);
  }}

  if (carryTransmissionPacketId !== transmission.packet_id) {{
    if (carryTransmissionFrame !== null) cancelAnimationFrame(carryTransmissionFrame);
    if (carryTransmissionPulse) map.removeLayer(carryTransmissionPulse);
    carryTransmissionPacketId = transmission.packet_id;
    carryTransmissionPulse = L.circleMarker(start, {{
      radius: 7, fillColor: '#60a5fa', color: '#ffffff', weight: 2,
      fillOpacity: 1, className: 'pulse-signal-particle', interactive: false,
    }}).addTo(map);
    animateCarryTransmission(transmission.packet_id, start, end);
  }}
}}

async function pollCarryTransmission() {{
  try {{
    const response = await fetch('/api/carry_transmission', {{ cache: 'no-store' }});
    if (!response.ok) return;
    renderCarryTransmission(await response.json());
  }} catch (error) {{
    console.warn('CARRY transmission status update failed', error);
  }}
}}

pollCarryTransmission();
setInterval(pollCarryTransmission, 100);

// ── 3a-mesh. RF relay mesh — enabled only by the dedicated mesh demo ──────
const rfMeshNodeMarkers = {{}};
const rfMeshStaticLinks = {{}};
let rfMeshActiveLink = null;
let rfMeshPulse = null;
let rfMeshBrokenRoute = null;
let rfMeshAnimationFrame = null;
let rfMeshActiveHopKey = null;
let rfMeshInitialViewFitted = false;

function clearRFMeshActivity() {{
  if (rfMeshAnimationFrame !== null) cancelAnimationFrame(rfMeshAnimationFrame);
  rfMeshAnimationFrame = null;
  rfMeshActiveHopKey = null;
  if (rfMeshPulse) {{ map.removeLayer(rfMeshPulse); rfMeshPulse = null; }}
  if (rfMeshActiveLink) {{ map.removeLayer(rfMeshActiveLink); rfMeshActiveLink = null; }}
  if (rfMeshBrokenRoute) {{ map.removeLayer(rfMeshBrokenRoute); rfMeshBrokenRoute = null; }}
}}

function clearRFMesh() {{
  clearRFMeshActivity();
  Object.values(rfMeshNodeMarkers).forEach(marker => map.removeLayer(marker));
  Object.values(rfMeshStaticLinks).forEach(link => map.removeLayer(link));
  Object.keys(rfMeshNodeMarkers).forEach(key => delete rfMeshNodeMarkers[key]);
  Object.keys(rfMeshStaticLinks).forEach(key => delete rfMeshStaticLinks[key]);
  rfMeshInitialViewFitted = false;
}}

function meshLinkKey(first, second) {{ return [first, second].sort().join('::'); }}

function animateRFMeshHop(hopKey, start, end) {{
  const startedAt = performance.now();
  function step(now) {{
    if (rfMeshActiveHopKey !== hopKey || !rfMeshPulse) return;
    const progress = Math.min((now - startedAt) / 460, 1);
    rfMeshPulse.setLatLng([
      start[0] + (end[0] - start[0]) * progress,
      start[1] + (end[1] - start[1]) * progress,
    ]);
    if (progress < 1) rfMeshAnimationFrame = requestAnimationFrame(step);
  }}
  rfMeshAnimationFrame = requestAnimationFrame(step);
}}

function renderRFMeshActivity(activity) {{
  if (!activity || !activity.active || !Array.isArray(activity.path)) {{ clearRFMeshActivity(); return; }}
  const hopIndex = Number(activity.current_hop_index);
  const path = activity.path;
  if (!Number.isInteger(hopIndex) || hopIndex < 0 || hopIndex >= path.length - 1) {{ clearRFMeshActivity(); return; }}
  const from = path[hopIndex], to = path[hopIndex + 1];
  const start = [Number(from.lat), Number(from.lon)], end = [Number(to.lat), Number(to.lon)];
  if (!start.every(Number.isFinite) || !end.every(Number.isFinite)) {{ clearRFMeshActivity(); return; }}
  const hopKey = `${{activity.packet_id || 'mesh'}}:${{hopIndex}}`;
  if (activity.rerouted && Array.isArray(activity.previous_path) && activity.previous_path.length > 1) {{
    const oldRoute = activity.previous_path.map(node => [Number(node.lat), Number(node.lon)]);
    if (oldRoute.every(point => point.every(Number.isFinite))) {{
      if (!rfMeshBrokenRoute) {{
        rfMeshBrokenRoute = L.polyline(oldRoute, {{ color:'#ef4444', weight:4, dashArray:'10, 9', opacity:.8 }}).addTo(map)
          .bindTooltip(`ROUTE INTERRUPTED · ${{activity.failed_node_id || 'relay'}} OFFLINE`, {{ direction:'center' }});
      }} else {{
        rfMeshBrokenRoute.setLatLngs(oldRoute);
      }}
    }}
  }}
  const tooltip = `RF mesh CARRY active · ${{from.node_id}} → ${{to.node_id}} · hop ${{hopIndex + 1}}/${{activity.hop_count || path.length - 1}}`;
  if (!rfMeshActiveLink) {{
    rfMeshActiveLink = L.polyline([start, end], {{ color:'#fbbf24', weight:5, opacity:1 }}).addTo(map).bindTooltip(tooltip, {{ direction:'center' }});
  }} else {{
    rfMeshActiveLink.setLatLngs([start, end]);
    rfMeshActiveLink.setTooltipContent(tooltip);
  }}
  if (rfMeshActiveHopKey !== hopKey) {{
    if (rfMeshAnimationFrame !== null) cancelAnimationFrame(rfMeshAnimationFrame);
    if (rfMeshPulse) map.removeLayer(rfMeshPulse);
    rfMeshActiveHopKey = hopKey;
    rfMeshPulse = L.circleMarker(start, {{ radius:8, color:'#fff7d6', weight:2, fillColor:'#f59e0b', fillOpacity:1, interactive:false }}).addTo(map);
    animateRFMeshHop(hopKey, start, end);
  }}
}}

function isSatelliteMeshFailoverDemo() {{
  return rfMeshTransportMode === 'satellite_primary' || rfMeshTransportMode === 'mesh_failover';
}}

function showFailoverNotice() {{
  if (failoverNoticeShown) return;
  failoverNoticeShown = true;
  const overlay = document.getElementById('failover-overlay');
  if (overlay) overlay.classList.add('visible');

  // Keep a mesh snapshot pending for the next polling pass.  That makes the
  // centered failure alert visibly precede node and hop rendering.
  if (failoverNoticeTimer) clearTimeout(failoverNoticeTimer);
  failoverNoticeTimer = setTimeout(() => overlay?.classList.remove('visible'), 4500);
}}

async function maybeShowFailoverNotice() {{
  // The failover scenario itself only permits the real outage after its
  // satellite-primary delivery phase has completed.  Confirm the existing
  // mesh-status state here so two independent polling responses cannot race.
  if (!failoverBlackoutObserved || failoverNoticeShown) return;
  if (isSatelliteMeshFailoverDemo()) {{
    showFailoverNotice();
    return;
  }}
  try {{
    const response = await fetch('/api/rf_mesh', {{ cache:'no-store' }});
    if (!response.ok) return;
    const snapshot = await response.json();
    if (snapshot?.mode === 'satellite_primary' || snapshot?.mode === 'mesh_failover') {{
      rfMeshTransportMode = snapshot.mode;
      showFailoverNotice();
    }}
  }} catch (error) {{
    console.warn('Failover notice status check failed', error);
  }}
}}

function renderRFMesh(snapshot) {{
  rfMeshTransportMode = snapshot?.mode || null;
  if (snapshot?.mode === 'mesh_failover' && !failoverNoticeShown) {{
    pendingFailoverMeshSnapshot = snapshot;
    maybeShowFailoverNotice();
    return;
  }}
  if (snapshot?.mode === 'mesh_failover') pendingFailoverMeshSnapshot = null;
  renderRFMeshLayers(snapshot);
  maybeShowFailoverNotice();
}}

function renderRFMeshLayers(snapshot) {{
  if (!snapshot || !snapshot.enabled) {{ clearRFMesh(); return; }}
  const nodes = snapshot.nodes || [];
  // The mesh-only demo starts centered on the disaster zone.  Fit its relay
  // corridor once so every static node and link is immediately inspectable.
  if (!rfMeshInitialViewFitted && nodes.length >= 2) {{
    map.invalidateSize({{ pan:false }});
    map.fitBounds(nodes.map(node => [Number(node.lat), Number(node.lon)]), {{ padding:[12, 12] }});
    rfMeshInitialViewFitted = true;
  }}
  const liveNodeIds = new Set(nodes.map(node => node.node_id));
  Object.entries(rfMeshNodeMarkers).forEach(([nodeId, marker]) => {{
    if (!liveNodeIds.has(nodeId)) {{ map.removeLayer(marker); delete rfMeshNodeMarkers[nodeId]; }}
  }});
  nodes.forEach(node => {{
    const coordinates = [Number(node.lat), Number(node.lon)];
    if (!coordinates.every(Number.isFinite)) return;
    const relaying = node.state === 'relaying';
    const failed = node.state === 'failed';
    const color = failed ? '#475569' : node.gateway ? '#a855f7' : relaying ? '#f59e0b' : '#38bdf8';
    const label = node.gateway ? 'RF Gateway to Command Post' : 'RF Relay Node';
    let marker = rfMeshNodeMarkers[node.node_id];
    if (!marker) {{
      marker = L.circleMarker(coordinates, {{ radius:8, color:'#e0f2fe', weight:2, fillColor:color, fillOpacity:1 }}).addTo(map);
      rfMeshNodeMarkers[node.node_id] = marker;
    }}
    marker.setLatLng(coordinates);
    marker.setStyle({{ fillColor:color, color:failed ? '#ef4444' : '#e0f2fe', dashArray:failed ? '3, 4' : null }});
    marker.setRadius(failed ? 10 : relaying ? 11 : 8);
    marker.bindTooltip(`RF-MESH NODE ${{node.node_id}} · ${{label}} · ${{String(node.state || 'idle').toUpperCase()}}`, {{ direction:'top' }});
  }});

  const nodeById = Object.fromEntries(nodes.map(node => [node.node_id, node]));
  const links = snapshot.links || [];
  const liveLinkIds = new Set();
  links.forEach(link => {{
    const first = nodeById[link.from], second = nodeById[link.to];
    if (!first || !second) return;
    const linkId = meshLinkKey(link.from, link.to); liveLinkIds.add(linkId);
    const failedLink = first.state === 'failed' || second.state === 'failed';
    if (!rfMeshStaticLinks[linkId]) {{
      rfMeshStaticLinks[linkId] = L.polyline([[first.lat, first.lon], [second.lat, second.lon]], {{
        color:failedLink ? '#ef4444' : '#64748b', weight:failedLink ? 2.5 : 1.5, dashArray:failedLink ? '8, 7' : '2, 8', opacity:failedLink ? .8 : .58,
      }}).addTo(map).bindTooltip(`RF link · ${{link.from}} ↔ ${{link.to}} · ${{Number(link.distance_km).toFixed(2)}} km`);
    }} else {{
      rfMeshStaticLinks[linkId].setStyle({{ color:failedLink ? '#ef4444' : '#64748b', weight:failedLink ? 2.5 : 1.5, dashArray:failedLink ? '8, 7' : '2, 8', opacity:failedLink ? .8 : .58 }});
    }}
  }});
  Object.entries(rfMeshStaticLinks).forEach(([linkId, line]) => {{
    if (!liveLinkIds.has(linkId)) {{ map.removeLayer(line); delete rfMeshStaticLinks[linkId]; }}
  }});
  renderRFMeshActivity(snapshot.activity);
}}

async function pollRFMesh() {{
  try {{
    const response = await fetch('/api/rf_mesh', {{ cache:'no-store' }});
    if (!response.ok) return;
    renderRFMesh(await response.json());
  }} catch (error) {{
    console.warn('RF mesh status update failed', error);
  }}
}}

pollRFMesh();
setInterval(pollRFMesh, 100);

// ── 3b. ONA Role 4 → selected Executor — active downlink only ───────────────
let executorDispatchLink = null;
let executorDispatchPulse = null;
let executorDispatchRelayId = null;
let executorDispatchFrame = null;
let onaBriefInFlight = false;

function hideExecutorDispatchTransmission() {{
  if (executorDispatchFrame !== null) cancelAnimationFrame(executorDispatchFrame);
  executorDispatchFrame = null;
  executorDispatchRelayId = null;
  if (executorDispatchPulse) {{ map.removeLayer(executorDispatchPulse); executorDispatchPulse = null; }}
  if (executorDispatchLink) {{ map.removeLayer(executorDispatchLink); executorDispatchLink = null; }}
}}

function animateExecutorDispatch(relayId, end) {{
  let cycleStartedAt = performance.now();
  function step(now) {{
    if (executorDispatchRelayId !== relayId || !executorDispatchPulse) return;
    const progress = Math.min((now - cycleStartedAt) / 1200, 1);
    executorDispatchPulse.setLatLng([
      CP_LAT + (end[0] - CP_LAT) * progress,
      CP_LON + (end[1] - CP_LON) * progress,
    ]);
    if (progress >= 1) cycleStartedAt = now;
    executorDispatchFrame = requestAnimationFrame(step);
  }}
  executorDispatchFrame = requestAnimationFrame(step);
}}

function renderExecutorDispatchTransmission(transmissions) {{
  const transmission = transmissions.find(item =>
    item.active && item.target_unit_id === selectedExecutorId
  );
  if (!selectedExecutorId || !transmission) {{
    const justDelivered = onaBriefInFlight;
    onaBriefInFlight = false;
    hideExecutorDispatchTransmission();
    if (justDelivered) {{
      const btn = document.getElementById('send-mission-btn');
      if (btn) btn.textContent = '✓ ONA BRIEF DELIVERED';
      updateExecStatusUI('EN_ROUTE');
    }}
    return;
  }}
  onaBriefInFlight = true;
  const marker = executorMarkers[transmission.target_unit_id];
  if (!marker) {{ hideExecutorDispatchTransmission(); return; }}
  const destination = marker.getLatLng();
  const end = [destination.lat, destination.lng];
  const tooltip = `ONA Role 4 downlink active · ${{transmission.target_unit_id}} · ${{transmission.briefing_id}}`;
  const statusEl = document.getElementById('briefing-status');
  if (statusEl) {{
    statusEl.textContent = `ONA BRIEF IN FLIGHT ($${{transmission.relay_id}})`;
    statusEl.style.color = '#c084fc';
  }}
  setOnaBriefProgress(`ONA ROLE 4: ${{transmission.briefing_id}} TRANSMITTING TO ${{transmission.target_unit_id}}`, '#c084fc');

  if (!executorDispatchLink) {{
    executorDispatchLink = L.polyline([[CP_LAT, CP_LON], end], {{
      color: '#a855f7', weight: 3.5, dashArray: '4, 8', opacity: 0.95,
    }}).addTo(map).bindTooltip(tooltip, {{ permanent: false, direction: 'center' }});
  }} else {{
    executorDispatchLink.setLatLngs([[CP_LAT, CP_LON], end]);
    executorDispatchLink.setTooltipContent(tooltip);
  }}

  if (executorDispatchRelayId !== transmission.relay_id) {{
    if (executorDispatchFrame !== null) cancelAnimationFrame(executorDispatchFrame);
    if (executorDispatchPulse) map.removeLayer(executorDispatchPulse);
    executorDispatchRelayId = transmission.relay_id;
    executorDispatchPulse = L.circleMarker([CP_LAT, CP_LON], {{
      radius: 8, fillColor: '#c084fc', color: '#ffffff', weight: 2,
      fillOpacity: 1, className: 'pulse-signal-particle', interactive: false,
    }}).addTo(map);
    animateExecutorDispatch(transmission.relay_id, end);
  }}
}}

async function pollExecutorDispatchTransmission() {{
  try {{
    const response = await fetch('/api/executor/dispatch_transmissions', {{ cache: 'no-store' }});
    if (!response.ok) return;
    const data = await response.json();
    renderExecutorDispatchTransmission(data.transmissions || []);
  }} catch (error) {{
    console.warn('Executor dispatch status update failed', error);
  }}
}}

pollExecutorDispatchTransmission();
setInterval(pollExecutorDispatchTransmission, 100);
// NOTE: There is intentionally NO Writer↔Executor direct line. All relay goes via ONA.

// ── 3c. Outdoor OSRM Street Route (Staging Depot → Building Entrance) ───────
async function getStreetRoute(startLat, startLon, endLat, endLon) {{
  try {{
    const url = `https://router.project-osrm.org/route/v1/driving/${{startLon}},${{startLat}};${{endLon}},${{endLat}}?geometries=geojson&overview=full`;
    const res = await fetch(url);
    const data = await res.json();
    if (data.code === 'Ok' && data.routes && data.routes.length > 0) {{
      return data.routes[0].geometry.coordinates.map(pt => [pt[1], pt[0]]);
    }}
  }} catch (e) {{
    console.warn('[OSRM JS] Route fetch failed:', e);
  }}
  return [[startLat, startLon], [endLat, endLon]];
}}

let streetRoutePolyline = null;
let streetRouteUnitId = null;
let streetRouteLoadingFor = null;
let streetRouteRequestId = 0;

function clearSelectedExecutorRoute() {{
  streetRouteRequestId += 1;
  streetRouteUnitId = null;
  streetRouteLoadingFor = null;
  if (streetRoutePolyline) {{
    map.removeLayer(streetRoutePolyline);
    streetRoutePolyline = null;
  }}
}}

function renderSelectedExecutorRoute() {{
  const unitId = selectedExecutorId;
  if (!unitId) {{ clearSelectedExecutorRoute(); return; }}
  if (streetRouteUnitId === unitId && streetRoutePolyline) return;
  if (streetRouteLoadingFor === unitId) return;

  const marker = executorMarkers[unitId];
  if (!marker) {{ clearSelectedExecutorRoute(); return; }}
  if (streetRoutePolyline) {{ map.removeLayer(streetRoutePolyline); streetRoutePolyline = null; }}

  const start = marker.getLatLng();
  const requestId = ++streetRouteRequestId;
  streetRouteLoadingFor = unitId;
  getStreetRoute(start.lat, start.lng, BUILDING_LAT, BUILDING_LON).then(pts => {{
    if (requestId !== streetRouteRequestId || selectedExecutorId !== unitId || !pts?.length) return;
    streetRoutePolyline = L.polyline(pts, {{
      color: '#10b981', weight: 3, dashArray: '6, 6', opacity: 0.85,
    }}).addTo(map);
    streetRoutePolyline.bindTooltip(`🧭 ${{unitId}} selected route to disaster zone`, {{ permanent: false }});
    streetRouteUnitId = unitId;
    streetRouteLoadingFor = null;
  }}).catch(() => {{
    if (requestId === streetRouteRequestId) streetRouteLoadingFor = null;
  }});
}}

// ── 5. Bounds Fit Framing Entire Tactical Theatre ────────────────────────────
function fitTacticalView() {{
  const bounds = L.latLngBounds([
    [BUILDING_LAT, BUILDING_LON],
    [CP_LAT, CP_LON],
    [WRITER_LAT, WRITER_LON],
    [EXECUTOR_LAT, EXECUTOR_LON]
  ]);
  map.fitBounds(bounds, {{ padding: [80, 80], maxZoom: 14 }});
}}
fitTacticalView();

// ── 6. Beacon Event Markers & Confidence Halos ──────────────────────────────
const markers = {{}};
let meshRelayIntroPopupShown = false;
const halos = {{}};
let _latestEvents = [];
const EVENT_ICONS = {{
  victim_detected:   {{ color: '#ef4444', svg: '<svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="#ef4444" stroke-width="2.5"><circle cx="12" cy="12" r="10"/><line x1="12" y1="8" x2="12" y2="16"/><line x1="8" y1="12" x2="16" y2="12"/></svg>' }},
  fire_detected:     {{ color: '#f97316', svg: '<svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="#f97316" stroke-width="2.5"><path d="M8.5 14.5A2.5 2.5 0 0 0 11 12c0-1.38-.5-2-1-3-1.072-2.143-.224-4.054 2-6 .5 2.5 2 4.9 4 6.5 2 1.6 3 3.5 3 5.5a7 7 0 1 1-14 0c0-1.153.433-2.294 1-3a2.5 2.5 0 0 0 2.5 2.5z"/></svg>' }},
}};

function makeIcon(event_type, severity) {{
  const info = EVENT_ICONS[event_type] || {{ color: '#94a3b8', svg: '<svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="#94a3b8" stroke-width="2.5"><circle cx="12" cy="12" r="4"/><circle cx="12" cy="12" r="9" stroke-dasharray="2 2"/></svg>' }};
  const size = Math.round(18 + severity * 6);
  return L.divIcon({{
    html: `<div class="pulse-marker" style="width:${{size}}px;height:${{size}}px;display:flex;align-items:center;justify-content:center;filter:drop-shadow(0 0 6px ${{info.color}});">${{info.svg}}</div>`,
    iconSize: [size, size],
    iconAnchor: [size/2, size/2],
    className: '',
  }});
}}

function updateBeaconHalo(evt) {{
  if (!evt || !evt.beacon_id) return;
  const sev = evt.severity ?? 0.5;
  const color = sev >= 0.8 ? '#ef4444' : sev >= 0.5 ? '#f97316' : '#eab308';
  const baseRadius = Math.round(18 + sev * 16);
  const ttl = evt.ttl_seconds || 3600;
  const age = evt.age_s || 0;
  const remainingRatio = Math.max(0, 1.0 - (age / ttl));

  let style;
  if (evt.stale || age >= ttl || remainingRatio <= 0) {{
    // Expired state: dashed grey outline
    style = {{
      color: '#64748b',
      weight: 1.5,
      dashArray: '4, 4',
      fillColor: '#64748b',
      fillOpacity: 0.05,
      opacity: 0.4,
      radius: 12
    }};
  }} else {{
    // Active decaying confidence halo (tied to real elapsed age vs TTL)
    const currentRadius = Math.max(8, Math.round(baseRadius * (0.35 + 0.65 * remainingRatio)));
    const fillOpacity = (0.08 + 0.40 * remainingRatio);
    const strokeOpacity = (0.25 + 0.70 * remainingRatio);
    const weight = Math.max(1, Math.round(1.5 + 2 * remainingRatio));
    style = {{
      color: color,
      weight: weight,
      fillColor: color,
      fillOpacity: fillOpacity,
      opacity: strokeOpacity,
      radius: currentRadius,
      dashArray: null
    }};
  }}

  if (!halos[evt.beacon_id]) {{
    halos[evt.beacon_id] = L.circleMarker([evt.lat, evt.lon], style).addTo(map);
  }} else {{
    halos[evt.beacon_id].setLatLng([evt.lat, evt.lon]);
    halos[evt.beacon_id].setRadius(style.radius);
    halos[evt.beacon_id].setStyle(style);
  }}

  if (window._continuityDemoHiding && halos[evt.beacon_id]) {{
    halos[evt.beacon_id].setStyle({{ opacity: 0, fillOpacity: 0 }});
  }}
}}

// ── 6. Update Loop ───────────────────────────────────────────────────────────
async function fetchAndUpdate() {{
  try {{
    const resp = await fetch('/api/events');
    const data = await resp.json();
    const events = data.events || [];
    _latestEvents = events;

    // Stats
    document.getElementById('stat-total').textContent = events.length;
    document.getElementById('stat-active').textContent = events.filter(e => !e.stale).length;
    document.getElementById('stat-victims').textContent = events.filter(e => e.event_type === 'victim_detected').length;
    document.getElementById('stat-fire').textContent = events.filter(e => e.event_type === 'fire_detected').length;

    // Render Event Markers and Confidence Halos
    let newEventsAdded = false;
    events.forEach(evt => {{
      updateBeaconHalo(evt);
      if (!markers[evt.beacon_id]) {{
        newEventsAdded = true;
        const isOriginalMeshDemo = rfMeshTransportMode === 'mesh';
        const m = L.marker([evt.lat, evt.lon], {{ icon: makeIcon(evt.event_type, evt.severity) }})
          .addTo(map)
          .bindPopup(`
            <div style="font-family:var(--font-sans);min-width:220px">
              <div class="tactical-popup-title" style="color:${{evt.color || '#ef4444'}};border-left:3px solid ${{evt.color || '#ef4444'}};padding-left:6px;">${{evt.event_type.replace(/_/g,' ').toUpperCase()}}</div>
              <div class="tactical-popup-sub">BEACON TELEMETRY · IN-SITU VERIFIED</div>
              <div class="tactical-popup-row"><b>Beacon ID:</b> ${{evt.beacon_id}}</div>
              <div class="tactical-popup-row"><b>CRC-32:</b> ${{evt.crc32 || 'UNVERIFIED'}}</div>
              <div class="tactical-popup-row"><b>Severity:</b> ${{(evt.severity*100).toFixed(0)}}% (Criticality)</div>
              <div class="tactical-popup-row"><b>GPS Telemetry:</b> ${{evt.lat.toFixed(6)}}° N, ${{evt.lon.toFixed(6)}}° E</div>
              <div class="tactical-popup-row"><b>Telemetry Age:</b> ${{Math.round(evt.age_s)}}s / TTL: ${{evt.ttl_seconds}}s</div>
              ${{evt.drift_corrected ? '<div class="tactical-popup-status" style="color:#10b981;">✓ Drift Compensated at ONA</div>' : ''}}
              ${{evt.stale ? '<div class="tactical-popup-status" style="color:#f59e0b;">⚠ STALE BEACON TELEMETRY</div>' : ''}}
            </div>
          `, {{ className: `beacon-event-popup${{isOriginalMeshDemo ? ' mesh-transient-popup' : ''}}` }});
        markers[evt.beacon_id] = m;
        // The normal dashboard may spotlight each incoming beacon.  In the
        // original RF Mesh Relay Demo, keep that map message to one compact
        // introduction so later packets do not cover the relay animation.
        if (!isOriginalMeshDemo || !meshRelayIntroPopupShown) {{
          m.openPopup();
          if (isOriginalMeshDemo) {{
            meshRelayIntroPopupShown = true;
            window.setTimeout(() => m.closePopup(), 1100);
          }}
        }}
      }}
      if (window._continuityDemoHiding && markers[evt.beacon_id]) {{
        markers[evt.beacon_id].setOpacity(0.05);
      }}
    }});

    // Auto-fit bounds so BOTH Command Post and newly arrived events remain clearly visible
    if (newEventsAdded && events.length > 0) {{
      const allPoints = [
        [CP_LAT, CP_LON],
        [BUILDING_LAT, BUILDING_LON],
        [WRITER_LAT, WRITER_LON],
        [EXECUTOR_LAT, EXECUTOR_LON],
        ...events.map(e => [e.lat, e.lon])
      ];
      map.fitBounds(L.latLngBounds(allPoints), {{ padding: [90, 90], maxZoom: 14 }});
    }}

    // Event list
    const listEl = document.getElementById('event-list');
    if (events.length === 0) {{
      listEl.innerHTML = '<div style="color:var(--ops-text-muted);font-size:11px;text-align:center;padding:16px 0">Waiting for beacon data via Outside Network Area…</div>';
    }} else {{
      listEl.innerHTML = events.map(evt => {{
        const cls = {{ victim_detected:'victim', fire_detected:'fire' }}[evt.event_type] || '';
        return `<div class="event-card ${{cls}}" onclick="map.setView([${{evt.lat}},${{evt.lon}}],16); markers['${{evt.beacon_id}}'].openPopup();">
          <div class="event-header">
            <span class="event-type" style="color:${{evt.color}}">${{evt.event_type.replace(/_/g,' ')}}</span>
            <span class="event-sev">${{(evt.severity*100).toFixed(0)}}%</span>
          </div>
          <div class="event-id">${{evt.beacon_id}} | ${{evt.writer_id}}</div>
          <div class="event-coords">${{evt.lat.toFixed(5)}}° N, ${{evt.lon.toFixed(5)}}° E</div>
          <div class="event-ttl">
            Age: ${{Math.round(evt.age_s)}}s / TTL: ${{evt.ttl_seconds}}s
            ${{evt.stale ? '<span class="stale-badge">STALE</span>' : ''}}
            ${{evt.drift_corrected ? ' | ✓ drift' : ''}}
          </div>
        </div>`;
      }}).join('');
    }}

    document.getElementById('last-update').textContent = new Date().toLocaleTimeString();
  }} catch(e) {{
    console.error('Tactical map update failed:', e);
  }}
}}

async function triggerFailure(type, btn) {{
  btn.disabled = true;
  try {{
    const resp = await fetch(`/api/failure/${{type}}`, {{method: 'POST'}});
    if (!resp.ok) throw new Error('Failed');
  }} catch (e) {{
    console.error('Failure injection error:', e);
  }} finally {{
    setTimeout(() => {{ btn.disabled = false; }}, 5000);
  }}
}}

// ── Before / After Mission Continuity Demo Sequence (Part 2) ───────────────
let _continuityActive = false;
let _continuityTimer = null;

async function triggerContinuityDemo(btn) {{
  if (_continuityActive) return;
  _continuityActive = true;
  if (btn) btn.disabled = true;

  const overlay = document.getElementById('continuity-overlay');
  const banner = document.getElementById('continuity-banner');
  const textEl = document.getElementById('continuity-banner-text');

  // Trigger Writer kill failure via backend API
  try {{
    await fetch('/api/failure/writer', {{ method: 'POST' }});
  }} catch(e) {{}}

  // Phase 1: Without Living Map (Data Lost Baseline)
  banner.className = 'continuity-banner before';
  textEl.innerHTML = `
    <div><b>❌ WITHOUT SPATIAL MEMORY</b> (Baseline Failure Mode)</div>
    <div style="font-weight:400;font-size:11px;margin-top:2px;">
      Writer Robot destroyed — all internal sensor data lost on scene with the robot! Map state cleared.
    </div>`;
  overlay.classList.add('visible');
  window._continuityDemoHiding = true;

  // Temporarily hide map event markers & confidence halos
  Object.values(markers).forEach(m => m.setOpacity(0.05));
  Object.values(halos).forEach(h => h.setStyle({{ opacity: 0, fillOpacity: 0 }}));

  _continuityTimer = setTimeout(() => {{
    // Phase 2: With Living Map (Mission Continuity Preserved)
    window._continuityDemoHiding = false;
    banner.className = 'continuity-banner after';
    textEl.innerHTML = `
                    <div><b>✅ WITH THE LIVING MAP</b> (Resilient mission continuity)</div>
      <div style="font-weight:400;font-size:11px;margin-top:2px;">
        Mission Continuity Preserved — 100% of spatial memory salvaged from ONA store! Real relayed beacons restored.
      </div>`;

    // Restore real markers & halos from actual ONA telemetry
    Object.values(markers).forEach(m => m.setOpacity(1.0));
    _latestEvents.forEach(evt => updateBeaconHalo(evt));

    _continuityTimer = setTimeout(() => {{
      dismissContinuityDemo();
      if (btn) btn.disabled = false;
    }}, 4500);
  }}, 3500);
}}

function dismissContinuityDemo() {{
  if (_continuityTimer) {{
    clearTimeout(_continuityTimer);
    _continuityTimer = null;
  }}
  window._continuityDemoHiding = false;
  const overlay = document.getElementById('continuity-overlay');
  if (overlay) overlay.classList.remove('visible');
  Object.values(markers).forEach(m => m.setOpacity(1.0));
  _latestEvents.forEach(evt => updateBeaconHalo(evt));
  _continuityActive = false;
  const btn = document.getElementById('continuity-demo-btn');
  if (btn) btn.disabled = false;
}}

async function generateMission() {{
  const btn = document.getElementById('gen-mission-btn');
  btn.disabled = true;
  btn.textContent = '⏳ Generating…';
  try {{
    const resp = await fetch('/api/generate_mission', {{ method: 'POST' }});
    const data = await resp.json();
    if (data.briefing) {{
      const panel = document.getElementById('mission-panel');
      panel.classList.add('visible');
      const briefing = data.briefing;
      document.getElementById('mission-content').textContent =
        `${{briefing.briefing_id}}\\n${{briefing.summary}}\\n\\n` +
        briefing.waypoints.map(w =>
          `${{w.waypoint_id}}: ${{w.event_type}} [${{(w.priority*100).toFixed(0)}}% priority]\\n  → ${{w.lat.toFixed(5)}}, ${{w.lon.toFixed(5)}}`
        ).join('\\n');
    }} else {{
      alert(data.error || 'No events available');
    }}
  }} catch(e) {{
    alert('Mission generation failed: ' + e.message);
  }} finally {{
    btn.disabled = false;
    btn.innerHTML = '<span>⚡</span> Generate & Send Mission';
  }}
}}

// ── Executor Marker Live Animation ──────────────────────────────────────────
// Polls /api/executor/position every 100ms and advances executorMarker step-by-step.
// The ONA→Executor relay line endpoint updates in sync.
let _execTargetLat = EXECUTOR_LAT;
let _execTargetLon = EXECUTOR_LON;
let _execCurLat = EXECUTOR_LAT;
let _execCurLon = EXECUTOR_LON;

async function pollExecutorPosition() {{
  try {{
    const r = await fetch('/api/executor/position');
    const d = await r.json();
    if (d.lat && d.lon) {{
      _execTargetLat = d.lat;
      _execTargetLon = d.lon;
    }}
  }} catch(e) {{ /* ignore transient network errors */ }}
}}

function animateExecutor() {{
  const LERP = 0.40; // smooth tracking factor per animation frame
  const dlat = _execTargetLat - _execCurLat;
  const dlon = _execTargetLon - _execCurLon;
  if (Math.abs(dlat) > 1e-9 || Math.abs(dlon) > 1e-9) {{
    _execCurLat += dlat * LERP;
    _execCurLon += dlon * LERP;
    // Invoke Leaflet's native position update method on the rendered marker
    executorMarker?.setLatLng([_execCurLat, _execCurLon]);
    // Keep an active ONA Role 4 downlink anchored to its selected Executor.
    if (executorDispatchLink) {{
      executorDispatchLink.setLatLngs([[CP_LAT, CP_LON], [_execCurLat, _execCurLon]]);
    }}
  }}
  requestAnimationFrame(animateExecutor);
}}

pollExecutorPosition();
setInterval(pollExecutorPosition, 100);
requestAnimationFrame(animateExecutor);

// ── Mission Briefing Sync ────────────────────────────────────────────────────
let _lastBriefingId = null;

function setOnaBriefProgress(message, color = 'var(--ops-text-muted)') {{
  const progress = document.getElementById('ona-brief-progress');
  if (!progress) return;
  progress.textContent = message;
  progress.style.color = color;
}}

function renderMissionBriefing(briefing) {{
  const container = document.getElementById('briefing-details');
  const statusEl = document.getElementById('briefing-status');
  if (!briefing || !briefing.waypoints) return;

  statusEl.textContent = `READY — AWAITING AUTHORIZATION ($${{briefing.briefing_id}})`;
  statusEl.style.color = '#f59e0b';
  onaBriefInFlight = false;
  setOnaBriefProgress('STEP 1/2: SELECT A RESPONSE UNIT, THEN AUTHORIZE ONA BRIEF', '#fbbf24');

  let html = `<div style="font-size:11px;color:var(--text-muted);margin-bottom:8px;font-family:'JetBrains Mono',monospace;">
    <div><b>Briefing ID:</b> ${{briefing.briefing_id}}</div>
    <div><b>Delivery:</b> Held at Command Post until operator authorization</div>
  </div>`;

  briefing.waypoints.forEach(w => {{
    html += `<div class="briefing-item">
      <div>
        <span class="briefing-item-id">${{w.waypoint_id}}</span>
        <span class="briefing-item-type">${{w.event_type.replace(/_/g, ' ')}}</span>
      </div>
      <div class="briefing-item-meta">
        Severity: ${{(w.severity * 100).toFixed(0)}}% | Priority: ${{(w.priority * 100).toFixed(0)}}% | GPS: ${{w.lat.toFixed(5)}}° N, ${{w.lon.toFixed(5)}}° E
      </div>
    </div>`;
  }});

  container.innerHTML = html;
}}

let missionAuthorizationStatus = 'IDLE';

function renderExecutorRoster(units) {{
  const container = document.getElementById('unit-roster');
  if (!container) return;
  if (!units.length) {{
    container.innerHTML = '<div style="color:var(--ops-text-muted);font-size:11px">No response units available.</div>';
    return;
  }}
  container.innerHTML = units.map(unit => {{
    const selectedClass = unit.selected ? ' selected' : '';
    const enabled = missionAuthorizationStatus === 'AWAITING_AUTHORIZATION' && !unit.selected;
    const score = unit.compatibility_score ?? 0;
    return `<div class="unit-card${{selectedClass}}" style="border-left-color:${{unit.icon_tint}}">
      <img class="unit-card-icon" src="${{unit.icon_url}}" alt="" style="filter:drop-shadow(0 0 4px ${{unit.icon_tint}})">
      <div><div class="unit-card-name">${{unit.display_name}}</div><div class="unit-card-meta">${{unit.unit_id}} · ${{unit.job_type}} · ${{Number(unit.battery).toFixed(0)}}% · ${{unit.status}}</div></div>
      <div class="unit-score">${{score}}%</div>
      <button class="unit-select-btn" data-unit-id="${{unit.unit_id}}" ${{enabled ? '' : 'disabled'}}>${{unit.selected ? 'SELECTED' : 'Select for mission'}}</button>
    </div>`;
  }}).join('');
  initLucideIcons();
  container.querySelectorAll('[data-unit-id]').forEach(button => {{
    button.addEventListener('click', () => selectExecutorUnit(button.dataset.unitId));
  }});
}}

async function refreshExecutorUnits() {{
  try {{
    const response = await fetch('/api/executor/units', {{ cache: 'no-store' }});
    const data = await response.json();
    const units = data.units || [];
    renderExecutorMarkers(units);
    const displayedUnit = units.find(unit => unit.selected) || units[0];
    if (displayedUnit) executorBatteryBar.setIcon(batteryMapBar(displayedUnit.unit_id));
    renderExecutorRoster(units);
    renderSelectedExecutorRoute();
  }} catch (error) {{
    console.warn('Executor roster update failed', error);
  }}
}}

async function selectExecutorUnit(unitId) {{
  try {{
    const response = await fetch('/api/executor/select', {{
      method: 'POST',
      headers: {{ 'Content-Type': 'application/json' }},
      body: JSON.stringify({{ unit_id: unitId }}),
    }});
    const data = await response.json();
    if (!response.ok) throw new Error(data.error || response.statusText);
    await refreshExecutorUnits();
    updateExecStatusUI(missionAuthorizationStatus);
    setOnaBriefProgress(`STEP 2/2: ${{unitId}} SELECTED — AUTHORIZE ONA BRIEF`, '#fbbf24');
  }} catch (error) {{
    alert('Unit selection failed: ' + error.message);
  }}
}}

// ── Authorization Gate: Send Mission button logic ─────────────────────────
function updateExecStatusUI(execStatus) {{
  const btn = document.getElementById('send-mission-btn');
  const lbl = document.getElementById('exec-status-label');
  if (!btn || !lbl) return;
  missionAuthorizationStatus = execStatus;
  if (execStatus === 'AWAITING_AUTHORIZATION') {{
    btn.disabled = !selectedExecutorId;
    lbl.textContent = selectedExecutorId ? `⚠ ${{selectedExecutorId}}: AWAITING AUTHORIZATION` : '⚠ SELECT A RESPONSE UNIT';
    lbl.className = 'exec-status-label awaiting';
    if (!selectedExecutorId) setOnaBriefProgress('STEP 1/2: SELECT A RESPONSE UNIT, THEN AUTHORIZE ONA BRIEF', '#fbbf24');
  }} else if (execStatus === 'EN_ROUTE') {{
    btn.disabled = true;
    lbl.textContent = `▶ ${{selectedExecutorId || 'SELECTED UNIT'}}: EN ROUTE`;
    lbl.className = 'exec-status-label en-route';
    const statusEl = document.getElementById('briefing-status');
    if (statusEl) {{
      statusEl.textContent = 'ONA BRIEF DELIVERED — EXECUTOR EN ROUTE';
      statusEl.style.color = '#34d399';
    }}
    setOnaBriefProgress(`ONA ROLE 4 DELIVERED — ${{selectedExecutorId || 'SELECTED UNIT'}} IS EN ROUTE`, '#34d399');
  }} else if (execStatus === 'COMPLETE') {{
    btn.disabled = true;
    lbl.textContent = `✓ ${{selectedExecutorId || 'SELECTED UNIT'}}: MISSION COMPLETE`;
    lbl.className = 'exec-status-label en-route';
  }} else {{
    btn.disabled = true;
    lbl.textContent = 'RESPONSE UNITS: IDLE';
    lbl.className = 'exec-status-label idle';
  }}
  refreshExecutorUnits();
}}

async function sendMission() {{
  const btn = document.getElementById('send-mission-btn');
  btn.disabled = true;
  btn.textContent = '⏳ Sending authorization to ONA…';
  try {{
    const resp = await fetch('/api/mission/authorize', {{ method: 'POST' }});
    const data = await resp.json();
    if (resp.ok) {{
      const statusEl = document.getElementById('briefing-status');
      if (statusEl) {{
        statusEl.textContent = `ONA BRIEF IN FLIGHT ($${{data.relay_id}})`;
        statusEl.style.color = '#c084fc';
      }}
      onaBriefInFlight = true;
      setOnaBriefProgress(`ONA ROLE 4: BRIEFING IN FLIGHT TO ${{data.unit_id}} — DO NOT REAUTHORIZE`, '#c084fc');
      btn.textContent = '⏳ ONA BRIEF IN FLIGHT…';
    }} else {{
      alert('Authorization failed: ' + (data.error || resp.statusText));
      btn.disabled = false;
      btn.textContent = 'Send Mission';
    }}
  }} catch(e) {{
    alert('Network error: ' + e.message);
    btn.disabled = false;
    btn.textContent = 'Send Mission';
  }}
}}

async function checkMissionBriefing() {{
  try {{
    const resp = await fetch('/api/mission/latest');
    const data = await resp.json();
    // Update button/label based on executor_status from server
    if (data.executor_status && !(onaBriefInFlight && data.executor_status === 'EN_ROUTE')) {{
      updateExecStatusUI(data.executor_status);
    }}
    if (data.briefing && data.briefing.briefing_id !== _lastBriefingId) {{
      _lastBriefingId = data.briefing.briefing_id;
      renderMissionBriefing(data.briefing);
    }}
  }} catch(e) {{}}
}}

// Start event update loop
function batteryTone(value) {{ return value < 20 ? ['red', '#d8645d', 'LOW BATTERY'] : value <= 50 ? ['yellow', '#e3a13d', 'RESERVE'] : ['green', '#54b889', 'NOMINAL']; }}
function updateBatteryGauge(prefix, value) {{
  const [tone, color, label] = batteryTone(value);
  const fill = document.getElementById(prefix + '-battery-fill');
  const val = document.getElementById(prefix + '-battery-value');
  const state = document.getElementById(prefix + '-battery-state');
  fill.style.width = value + '%'; fill.className = 'conn-gauge-fill ' + tone;
  val.textContent = value.toFixed(1) + '%'; val.className = 'conn-gauge-value ' + tone;
  state.textContent = label; state.className = 'conn-gauge-state ' + (tone === 'green' ? 'nominal' : tone);
}}
async function updateRobotEnergy() {{
  try {{
    const response = await fetch('/api/rovers', {{ cache: 'no-store' }});
    const data = await response.json();
    const writer = data.writer, executor = data.executor;
    updateBatteryGauge('writer', writer.battery); updateBatteryGauge('executor', executor.battery);
    writerBatteryBar.setLatLng([writer.lat, writer.lon]); executorBatteryBar.setLatLng([executor.lat, executor.lon]);
    for (const [marker, robot] of [[writerBatteryBar, writer], [executorBatteryBar, executor]]) {{
      if (!marker || typeof marker.getElement !== 'function') continue;
      const el = marker.getElement(); if (!el) continue;
      const fill = el.querySelector('.map-battery-fill');
      if (fill) fill.style.cssText = `width:${{robot.battery}}%;background:${{batteryTone(robot.battery)[1]}}`;
      el.classList.toggle('battery-critical', robot.battery < 20);
    }}
    writerMarker?.getElement?.()?.classList.toggle('battery-critical', writer.battery < 20);
    executorMarker?.getElement?.()?.classList.toggle('battery-critical', executor.battery < 20);
  }} catch (e) {{
    console.warn('Battery update failed', e);
  }} finally {{
    // Poll again only after this request finishes so updates stay responsive
    // without allowing overlapping telemetry requests.
    setTimeout(updateRobotEnergy, 100);
  }}
}}
fetchAndUpdate();
updateRobotEnergy();
setInterval(fetchAndUpdate, 3000);
checkMissionBriefing();
setInterval(checkMissionBriefing, 1000);
refreshExecutorUnits();
setInterval(refreshExecutorUnits, 1000);

// ── Mission Summary Modal ──────────────────────────────────────────────────
let _summaryVisible = false;
let _summaryCountdown = null;
let _summarySeenCompletionKey = null;  // mission IDs restart each loop cycle

function _failureClass(type) {{
  if (type === 'satellite_disrupted') return 'satellite';
  if (type === 'beacon_expired') return 'beacon';
  if (type === 'stacked') return 'stacked';
  return '';  // writer_killed gets default red
}}

function renderSummaryModal(data) {{
  if (!data || !data.mission_id) return;
  // Deduplicate one completion payload, while allowing BRIEF-001 in later cycles.
  const completionKey = `${{data.cycle ?? 'unknown'}}:${{data.mission_id}}:${{data.timestamp ?? 'unknown'}}`;
  if (completionKey === _summarySeenCompletionKey) return;
  _summarySeenCompletionKey = completionKey;
  _lastSummaryMissionId = data.mission_id || null;

  const visited = data.targets_visited ?? 0;
  const total = data.targets_total ?? 0;
  const pct = data.completion_pct ?? (total > 0 ? Math.round(100 * visited / total) : 0);
  const drift = data.drift_corrections ?? 0;
  const skipped = data.beacons_skipped_ttl ?? 0;
  const missionId = data.mission_id || '—';
  const ts = data.timestamp ? new Date(data.timestamp * 1000).toLocaleTimeString() : '—';
  const beacons = data.beacons_by_priority || [];
  const failures = data.failures || [];
  const cycle = data.cycle || '—';

  document.getElementById('summary-title').textContent = `Mission Cycle ${{cycle}} — Summary`;
  document.getElementById('summary-meta').textContent = `${{missionId}} | Completed: ${{ts}}`;

  // ── Outcome card
  const outcomeColor = pct >= 80 ? 'green' : pct >= 50 ? 'yellow' : 'red';
  const outcomeHtml = `
    <div class="summary-card">
      <div class="summary-card-title"><span class="summary-card-title-icon"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><circle cx="12" cy="12" r="10"/><line x1="22" y1="12" x2="18" y2="12"/><line x1="6" y1="12" x2="2" y2="12"/><line x1="12" y1="6" x2="12" y2="2"/><line x1="12" y1="22" x2="12" y2="18"/></svg></span> Overall Outcome</div>
      <div class="summary-metric">
        <div class="summary-metric-val ${{outcomeColor}}">${{pct}}%</div>
        <div class="summary-metric-unit">completion</div>
      </div>
      <div class="summary-progress"><div class="summary-progress-fill" style="width:${{pct}}%"></div></div>
      <div class="summary-row">
        <span class="summary-row-label">Targets visited</span>
        <span class="summary-row-val">${{visited}} / ${{total}}</span>
      </div>
      <div class="summary-row">
        <span class="summary-row-label">Beacons skipped (TTL)</span>
        <span class="summary-row-val">${{skipped}}</span>
      </div>
    </div>`;

  // ── Beacon priority card
  const beaconRows = beacons.map((b, i) => `
    <div class="beacon-priority-item">
      <div class="beacon-priority-rank">${{i+1}}</div>
      <div style="flex:1;min-width:0">
        <div style="font-size:11px;font-weight:600;color:#e2e8f0;white-space:nowrap;overflow:hidden;text-overflow:ellipsis">${{b.type ? b.type.replace(/_/g,' ') : b.id}}</div>
        <div style="font-size:10px;color:var(--text-muted);font-family:'JetBrains Mono',monospace">${{b.id}}</div>
      </div>
      <div class="beacon-priority-bar"><div class="beacon-priority-fill" style="width:${{b.priority_pct}}%"></div></div>
      <div class="beacon-priority-pct">${{b.priority_pct}}%</div>
    </div>`).join('');
  const beaconHtml = `
    <div class="summary-card">
      <div class="summary-card-title"><span class="summary-card-title-icon"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M4.9 19.1C1 15.2 1 8.8 4.9 4.9"/><path d="M7.8 16.2c-2.3-2.3-2.3-6.1 0-8.5"/><circle cx="12" cy="12" r="2"/><path d="M16.2 7.8c2.3 2.3 2.3 6.1 0 8.5"/><path d="M19.1 4.9C23 8.8 23 15.1 19.1 19"/></svg></span> Beacon Handling (Priority Order)</div>
      ${{beacons.length > 0
        ? `<div class="beacon-priority-list">${{beaconRows}}</div>`
        : '<div style="font-size:11px;color:var(--text-muted)">No beacons briefed this cycle.</div>'
      }}
    </div>`;

  // ── Failure recap card
  let failureHtml;
  if (failures.length === 0) {{
    failureHtml = `
      <div class="summary-card full-width">
        <div class="summary-card-title"><span class="summary-card-title-icon"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M12 22s8-4 8-10V5l-8-3-8 3v7c0 6 8 10 8 10z"/><polyline points="9 12 11 14 15 10"/></svg></span> Failure Injection Recap</div>
        <div class="nominal-badge">✓ No failures injected this cycle — nominal run</div>
      </div>`;
  }} else {{
    const items = failures.map(f => `
      <div class="failure-recap-item ${{_failureClass(f.type)}}">
        <div class="failure-recap-header">
          <span class="failure-recap-label">${{f.label}}</span>
          <span class="failure-recap-ts">${{f.ts_str || ''}}</span>
        </div>
        <div class="failure-recap-detail">${{f.detail}}</div>
      </div>`).join('');
    failureHtml = `
      <div class="summary-card full-width">
        <div class="summary-card-title"><span class="summary-card-title-icon"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polygon points="13 2 3 14 12 14 11 22 21 10 12 10 13 2"/></svg></span> Failure Injection Recap (${{failures.length}} event${{failures.length !== 1 ? 's' : ''}})</div>
        <div class="failure-recap-list">${{items}}</div>
      </div>`;
  }}

  // ── Drift summary card
  const driftHtml = `
    <div class="summary-card">
      <div class="summary-card-title"><span class="summary-card-title-icon"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="10"/><polygon points="16.24 7.76 14.12 14.12 7.76 16.24 9.88 9.88 16.24 7.76"/></svg></span> Drift Correction</div>
      <div class="summary-metric">
        <div class="summary-metric-val ${{drift > 0 ? 'green' : ''}}">${{drift}}</div>
        <div class="summary-metric-unit">correction${{drift !== 1 ? 's' : ''}} applied</div>
      </div>
      <div class="summary-row">
        <span class="summary-row-label">Before/After logged</span>
        <span class="summary-row-val">${{drift > 0 ? 'Yes — ONA log' : 'N/A'}}</span>
      </div>
    </div>`;

  document.getElementById('summary-sections').innerHTML = outcomeHtml + beaconHtml + failureHtml + driftHtml;
  initLucideIcons();

  // Show modal
  const backdrop = document.getElementById('summary-backdrop');
  backdrop.classList.add('visible');
  _summaryVisible = true;

  // Start countdown
  let remaining = 15;
  const countdownEl = document.getElementById('summary-countdown');
  countdownEl.textContent = `Auto-closing in ${{remaining}}s…`;
  _summaryCountdown = setInterval(() => {{
    remaining -= 1;
    countdownEl.textContent = `Auto-closing in ${{remaining}}s…`;
    if (remaining <= 0) {{
      dismissSummary();
    }}
  }}, 1000);
}}

function dismissSummary() {{
  const backdrop = document.getElementById('summary-backdrop');
  backdrop.classList.remove('visible');
  _summaryVisible = false;
  if (_summaryCountdown) {{
    clearInterval(_summaryCountdown);
    _summaryCountdown = null;
  }}
}}

let _lastSummaryMissionId = null;

function exportMissionPDF() {{
  const url = _lastSummaryMissionId ? `/api/export_mission_pdf?mission_id=${{encodeURIComponent(_lastSummaryMissionId)}}` : '/api/export_mission_pdf';
  window.open(url, '_blank');
}}

async function checkCycleSummary() {{
  try {{
    const resp = await fetch('/api/cycle_summary');
    const data = await resp.json();
    if (data && data.mission_id) {{
      renderSummaryModal(data);
    }}
  }} catch(e) {{ /* ignore */ }}
}}

// Poll for cycle summary every 2s — starts up immediately after page load
setInterval(checkCycleSummary, 2000);

</script>
</body>
</html>
"""


def _build_webots_vision_html() -> str:
    return """<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Pioneer Robot Camera — Living Map</title>
<style>
  :root { color-scheme: dark; --bg:#05080c; --panel:#0d141f; --line:#25394f; --cyan:#38bdf8; --green:#10b981; --amber:#f59e0b; }
  * { box-sizing:border-box } body { margin:0; min-height:100vh; background:var(--bg); color:#e5edf6; font-family:Arial,sans-serif; padding:18px; }
  header { display:flex; justify-content:space-between; gap:12px; align-items:center; border-bottom:1px solid var(--line); padding-bottom:12px; margin-bottom:14px; }
  h1 { margin:0; font-size:18px; letter-spacing:.08em; color:var(--cyan); } .sub { color:#94a3b8; font:12px monospace; margin-top:5px; }
  .live { color:var(--amber); font:12px monospace; } .frame { position:relative; background:#020406; border:1px solid var(--line); min-height:360px; display:flex; align-items:center; justify-content:center; overflow:hidden; }
  #camera { width:100%; max-height:480px; object-fit:contain; display:none; } #waiting { color:#94a3b8; font:14px monospace; text-align:center; line-height:1.6; }
  .camera-labels { position:absolute; top:14px; left:14px; display:grid; gap:8px; pointer-events:none; } .camera-label { min-width:188px; padding:8px 10px; border:1px solid rgba(245,158,11,.72); border-left:4px solid var(--amber); background:rgba(5,8,12,.86); color:#fde68a; font:700 12px monospace; letter-spacing:.05em; box-shadow:0 3px 14px rgba(0,0,0,.32); } .camera-label.confirmed { border-color:rgba(16,185,129,.82); border-left-color:var(--green); color:#a7f3d0; } .camera-label small { display:block; margin-top:3px; color:#94a3b8; font:10px monospace; letter-spacing:0; }
  #cv-boxes { position:absolute; pointer-events:none; overflow:visible; } .cv-box { position:absolute; border:3px solid #10b981; box-shadow:0 0 0 1px rgba(2,6,23,.82),0 0 16px rgba(16,185,129,.5); } .cv-box.fire { border-color:#fb923c; box-shadow:0 0 0 1px rgba(2,6,23,.82),0 0 16px rgba(249,115,22,.55); } .cv-tag { position:absolute; top:-29px; left:-3px; padding:5px 8px; background:#10b981; color:#02120c; font:800 11px monospace; letter-spacing:.04em; white-space:nowrap; box-shadow:0 2px 8px rgba(0,0,0,.4); } .cv-box.fire .cv-tag { background:#fb923c; color:#1b0a02; }
  .grid { display:grid; grid-template-columns:1fr 1fr; gap:12px; margin-top:14px; } .card { background:var(--panel); border:1px solid var(--line); border-left:4px solid var(--amber); padding:12px; }
  .card.detected { border-left-color:var(--green); box-shadow:0 0 14px rgba(16,185,129,.16); } .label { font:700 13px monospace; letter-spacing:.08em; } .state { color:#fbbf24; font:12px monospace; margin-top:8px; } .detected .state { color:#6ee7b7; }
  .detail { color:#94a3b8; font:12px monospace; margin-top:5px; } .note { margin-top:14px; color:#94a3b8; font-size:12px; line-height:1.45; }
</style></head><body>
<header><div><h1>PIONEER FRONT CAMERA</h1><div class="sub">LIVE SENSOR FEED · WEBOTS VISION + DETECTION</div></div><div id="connection" class="live">CONNECTING…</div></header>
<section class="frame"><img id="camera" alt="Live Pioneer front-camera feed"><div id="cv-boxes"></div><div class="camera-labels"><div class="camera-label" id="fire-feed-label">FIRE · SEARCHING<small>Awaiting visual confirmation</small></div><div class="camera-label" id="victim-feed-label">VICTIM · SEARCHING<small>Awaiting visual confirmation</small></div></div><div id="waiting">Start the Webots visual companion.<br>The live robot camera will appear here automatically.</div></section>
<section class="grid"><article class="card" id="fire-card"><div class="label">FIRE DETECTION</div><div class="state" id="fire-state">SEARCHING</div><div class="detail" id="fire-detail">Awaiting camera confirmation</div></article><article class="card" id="victim-card"><div class="label">VICTIM DETECTION</div><div class="state" id="victim-state">SEARCHING</div><div class="detail" id="victim-detail">Awaiting camera confirmation</div></article></section>
<div class="note">Green means the robot's front camera has confirmed the target. The displayed distance and source are emitted by the running Webots controller.</div>
<script>
  const camera = document.getElementById('camera'), waiting = document.getElementById('waiting'), connection = document.getElementById('connection'), cvBoxes = document.getElementById('cv-boxes'); let latestVisionBoxes = [], latestImageSize = [640, 480];
  function renderVisionBoxes() { const frame = camera.closest('.frame'), imageRect = camera.getBoundingClientRect(), frameRect = frame.getBoundingClientRect(); cvBoxes.style.left=(imageRect.left-frameRect.left)+'px'; cvBoxes.style.top=(imageRect.top-frameRect.top)+'px'; cvBoxes.style.width=imageRect.width+'px'; cvBoxes.style.height=imageRect.height+'px'; cvBoxes.replaceChildren(); const [sourceWidth, sourceHeight] = latestImageSize; latestVisionBoxes.forEach(box => { if (!box.center || !box.size || !sourceWidth || !sourceHeight) return; const node=document.createElement('div'), tag=document.createElement('div'), isFire=box.label === 'FIRE'; node.className='cv-box'+(isFire ? ' fire' : ''); const left=100*box.center[0]/sourceWidth, top=100*box.center[1]/sourceHeight, width=100*box.size[0]/sourceWidth, height=100*box.size[1]/sourceHeight; node.style.left=(left-width/2)+'%'; node.style.top=(top-height/2)+'%'; node.style.width=width+'%'; node.style.height=height+'%'; tag.className='cv-tag'; const distance=Number.isFinite(box.distance_m) ? ` · ${box.distance_m.toFixed(2)} m` : ''; tag.textContent=`${box.label} DETECTED · ${Math.round((box.confidence || 1)*100)}%${distance}`; node.appendChild(tag); cvBoxes.appendChild(node); }); }
  function updateCard(item) { const key=item.label.toLowerCase(), card=document.getElementById(key+'-card'); document.getElementById(key+'-state').textContent='CONFIRMED'; document.getElementById(key+'-detail').textContent=`${item.source.toUpperCase()} · ${item.distance_m.toFixed(2)} m`; card.classList.add('detected'); const feed=document.getElementById(key+'-feed-label'); feed.classList.add('confirmed'); feed.innerHTML=`${item.label.toUpperCase()} · CONFIRMED<small>${item.source.toUpperCase()} · ${item.distance_m.toFixed(2)} m</small>`; }
  camera.addEventListener('load', renderVisionBoxes); window.addEventListener('resize', renderVisionBoxes);
  function connect() { const ws=new WebSocket('ws://127.0.0.1:8765'); ws.onopen=()=>{ connection.textContent='LIVE · CONNECTED · 8 FPS'; connection.style.color='#10b981'; }; ws.onclose=()=>{ connection.textContent='WAITING FOR WEBOTS…'; connection.style.color='#f59e0b'; setTimeout(connect,1500); }; ws.onmessage=(event)=>{ if(typeof event.data!=='string') return; try { const packet=JSON.parse(event.data); if(packet.type!=='camera_frame') return; latestVisionBoxes=packet.vision_boxes || []; latestImageSize=packet.image_size || [640,480]; camera.src=packet.image; camera.style.display='block'; waiting.style.display='none'; requestAnimationFrame(renderVisionBoxes); (packet.detections||[]).forEach(updateCard); } catch (_) {} }; }
  connect();
</script></body></html>"""


def _build_communications_console_html() -> str:
    """Render the detailed communications view using the terminal's table layout."""
    return """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <meta http-equiv="Cache-Control" content="no-cache, no-store, must-revalidate">
  <title>Living Map — Communication Console</title>
  <link rel="preconnect" href="https://fonts.googleapis.com">
  <link rel="preconnect" href="https://fonts.googleapis.com">
  <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
  <link href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:ital,wght@0,400;0,500;0,600;0,700;1,400&family=IBM+Plex+Sans:wght@400;500;600;700&family=Rajdhani:wght@500;600;700&display=swap" rel="stylesheet">
  <style>
    :root {
      --bg: #0b1318; --panel: #101d25; --panel-strong: #172730;
      --line: #314955; --line-soft: rgba(73, 102, 114, .55);
      --text: #e6eef0; --muted: #9db3ba; --dim: #6f858d;
      --cyan: #66c4d4; --green: #54b889; --yellow: #e8c56a; --red: #e2756e;
    }
    * { box-sizing: border-box; }
    body { margin: 0; min-width: 900px; background: var(--bg); color: var(--text); font-family: 'IBM Plex Sans', sans-serif; }
    header { min-height: 86px; display: flex; justify-content: space-between; align-items: center; gap: 24px; padding: 18px 28px; background: var(--panel-strong); border-bottom: 2px solid var(--cyan); }
    .eyebrow { color: var(--cyan); font: 600 10px 'IBM Plex Mono', monospace; letter-spacing: .1em; text-transform: uppercase; }
    h1 { margin: 3px 0 0; font-size: 20px; font-weight: 600; }
    .sub { margin-top: 4px; color: var(--muted); font: 11px 'IBM Plex Mono', monospace; }
    .header-actions { display: flex; align-items: center; gap: 10px; }
    .live { color: var(--green); font: 600 11px 'IBM Plex Mono', monospace; letter-spacing: .06em; }
    .return { padding: 8px 11px; border: 1px solid var(--line); color: var(--text); background: transparent; text-decoration: none; font-size: 12px; }
    .return:hover { border-color: var(--cyan); color: var(--cyan); }
    main { max-width: 1600px; margin: 0 auto; padding: 28px 30px 50px; }
    .architecture { display: flex; flex-wrap: wrap; align-items: center; gap: 10px; padding: 16px 18px; margin-bottom: 24px; background: var(--panel); border: 1px solid var(--line); color: var(--muted); font: 12px/1.6 'IBM Plex Mono', monospace; }
    .architecture .node { color: var(--text); font-weight: 600; }
    .architecture .arrow { color: var(--cyan); }
    .stats { display: grid; grid-template-columns: repeat(5, minmax(0, 1fr)); border: 1px solid var(--line); margin-bottom: 26px; background: var(--panel); }
    .stat { min-height: 92px; padding: 17px 20px; border-right: 1px solid var(--line); }
    .stat:last-child { border-right: 0; }
    .stat-label { color: var(--muted); font: 10px 'IBM Plex Mono', monospace; text-transform: uppercase; }
    .stat-value { margin-top: 6px; color: var(--cyan); font: 600 23px 'IBM Plex Mono', monospace; }
    .section { margin-top: 28px; border: 1px solid var(--line); background: var(--panel); }
    .section-head { display: flex; align-items: baseline; justify-content: space-between; gap: 18px; padding: 16px 18px; background: var(--panel-strong); border-bottom: 1px solid var(--line); }
    .section-title { font-size: 16px; font-weight: 600; }
    .section-note { color: var(--muted); font: 10px 'IBM Plex Mono', monospace; text-align: right; }
    .table-wrap { overflow: auto; max-height: 52vh; padding-bottom: 10px; }
    table { width: 100%; min-width: 1060px; border-collapse: separate; border-spacing: 0; font: 12px/1.65 'IBM Plex Mono', monospace; }
    thead { position: sticky; top: 0; z-index: 1; background: #14252d; }
    th { padding: 13px 16px; color: var(--muted); border-bottom: 1px solid var(--line); text-align: left; font-size: 10px; font-weight: 600; letter-spacing: .06em; text-transform: uppercase; }
    td { padding: 14px 16px; border-bottom: 1px solid var(--line-soft); color: var(--text); vertical-align: top; }
    tbody tr:hover td { background: rgba(102, 196, 212, .055); }
    .group-row td { padding: 14px 16px 11px; color: var(--cyan); background: #0d181e; border-top: 14px solid var(--bg); border-bottom: 2px solid var(--line); font-size: 11px; font-weight: 600; letter-spacing: .08em; }
    .ref { color: var(--yellow); font-weight: 600; white-space: nowrap; }
    .stage { color: var(--cyan); white-space: nowrap; }
    .detail { color: var(--muted); min-width: 600px; overflow-wrap: anywhere; }
    .warn { color: var(--yellow); } .error { color: var(--red); }
    .event-type { color: var(--cyan); } .severity { color: var(--yellow); text-align: right; } .ttl { color: var(--green); text-align: right; }
    .drift-corrected { color: var(--green); } .drift-nominal { color: var(--muted); }
    .empty { padding: 34px; color: var(--muted); text-align: center; font: 11px 'IBM Plex Mono', monospace; }
    @media (max-width: 1100px) { .stats { grid-template-columns: repeat(3, 1fr); } .stat:nth-child(3) { border-right: 0; } }
  </style>
</head>
<body>
  <header>
    <div>
      <div class="eyebrow">Command Post / Observability</div>
      <h1>Live Communication Console</h1>
      <div class="sub">Structured packet flow and in-situ beacon register</div>
    </div>
    <div class="header-actions"><span id="live-state" class="live">● LIVE · WAITING</span><a class="return" href="/">Return to Tactical Map</a></div>
  </header>
  <main>
    <div class="architecture"><span class="node">WRITER</span><span class="arrow">→</span><span class="node">BEACON NETWORK</span><span class="arrow">→</span><span class="node">ONA RECEIVE / TRANSLATE / CARRY</span><span class="arrow">→</span><span class="node">COMMAND POST</span><span class="arrow">→</span><span class="node">ONA BRIEF</span><span class="arrow">→</span><span class="node">EXECUTOR</span></div>
    <section class="stats">
      <div class="stat"><div class="stat-label">Packet records</div><div id="count-total" class="stat-value">0</div></div>
      <div class="stat"><div class="stat-label">ONA receive</div><div id="count-receive" class="stat-value">0</div></div>
      <div class="stat"><div class="stat-label">ONA translate</div><div id="count-translate" class="stat-value">0</div></div>
      <div class="stat"><div class="stat-label">ONA carry</div><div id="count-carry" class="stat-value">0</div></div>
      <div class="stat"><div class="stat-label">Beacon register</div><div id="count-beacons" class="stat-value">0</div></div>
    </section>
    <section class="section">
      <div class="section-head"><div class="section-title">Communication Packet Log</div><div class="section-note">Terminal-style chronological protocol record</div></div>
      <div class="table-wrap"><table><thead><tr><th>Time</th><th>Pipeline Stage</th><th>Packet / Beacon</th><th>Operational Detail</th></tr></thead><tbody id="packet-body"><tr><td class="empty" colspan="4">Waiting for communication traffic…</td></tr></tbody></table></div>
    </section>
    <section class="section">
      <div class="section-head"><div class="section-title">Command Post Beacon Register</div><div class="section-note">Same fields as the terminal Command Post Event Feed</div></div>
      <div class="table-wrap"><table><thead><tr><th>Beacon ID</th><th>CRC-32</th><th>Event Type</th><th>Severity</th><th>GPS Coordinates</th><th>TTL</th><th>Drift</th></tr></thead><tbody id="beacon-body"><tr><td class="empty" colspan="7">Waiting for a relayed beacon…</td></tr></tbody></table></div>
    </section>
  </main>
  <script>
    let sequence = 0;
    const MAX_PACKET_ROWS = 500;
    const counts = { total: 0, receive: 0, translate: 0, carry: 0 };
    let lastGroup = '';

    function cell(row, text, className = '') {
      const item = document.createElement('td');
      item.textContent = text || '—';
      if (className) item.className = className;
      row.appendChild(item);
    }
    function groupFor(source) {
      if (source === 'WRITER') return 'SOURCE ROBOT AND BEACON NETWORK';
      if (source === 'ONA RECEIVE' || source === 'ONA TRANSLATE' || source === 'ONA CARRY' || source === 'ONA HEARTBEAT') return 'OUTSIDE NETWORK AREA — RECEIVE / TRANSLATE / CARRY / HEARTBEAT';
      if (source === 'COMMAND POST') return 'COMMAND POST';
      return 'MISSION RELAY AND EXECUTOR';
    }
    function referenceFor(event) {
      const match = String(event.message || '').match(/\\b(?:PKT\\d+|MISSION-\\d+|B[A-F0-9]{6})\\b/i);
      return match ? match[0].toUpperCase() : 'SYSTEM';
    }
    function updateCounts() {
      document.getElementById('count-total').textContent = counts.total;
      document.getElementById('count-receive').textContent = counts.receive;
      document.getElementById('count-translate').textContent = counts.translate;
      document.getElementById('count-carry').textContent = counts.carry;
    }
    function appendPacket(event) {
      const body = document.getElementById('packet-body');
      const empty = body.querySelector('.empty');
      if (empty) empty.closest('tr').remove();
      const group = groupFor(event.source || 'SYSTEM');
      if (group !== lastGroup) {
        const divider = document.createElement('tr'); divider.className = 'group-row';
        const dividerCell = document.createElement('td'); dividerCell.colSpan = 4; dividerCell.textContent = group;
        divider.appendChild(dividerCell); body.appendChild(divider); lastGroup = group;
      }
      const row = document.createElement('tr');
      const level = String(event.level || 'INFO').toLowerCase();
      cell(row, event.timestamp || '--:--:--');
      cell(row, event.source || 'SYSTEM', 'stage ' + (level === 'warning' ? 'warn' : level === 'error' ? 'error' : ''));
      cell(row, referenceFor(event), 'ref');
      cell(row, event.message || '', 'detail ' + (level === 'warning' ? 'warn' : level === 'error' ? 'error' : ''));
      body.appendChild(row);
      while (body.querySelectorAll('tr:not(.group-row)').length > MAX_PACKET_ROWS) body.removeChild(body.firstElementChild);
      counts.total += 1;
      if (event.source === 'ONA RECEIVE') counts.receive += 1;
      if (event.source === 'ONA TRANSLATE') counts.translate += 1;
      if (event.source === 'ONA CARRY') counts.carry += 1;
      updateCounts();
    }
    async function refreshBeaconRegister() {
      try {
        const response = await fetch('/api/events', { cache: 'no-store' });
        if (!response.ok) return;
        const data = await response.json(); const events = data.events || [];
        document.getElementById('count-beacons').textContent = events.length;
        const body = document.getElementById('beacon-body'); body.replaceChildren();
        if (!events.length) { const row = document.createElement('tr'); const placeholder = document.createElement('td'); placeholder.colSpan = 7; placeholder.className = 'empty'; placeholder.textContent = 'Waiting for a relayed beacon…'; row.appendChild(placeholder); body.appendChild(row); return; }
        events.forEach(event => {
          const row = document.createElement('tr');
          cell(row, event.beacon_id, 'ref');
          cell(row, event.crc32 || '—', 'ref');
          cell(row, String(event.event_type || 'unknown').replace(/_/g, ' '), 'event-type');
          cell(row, ((Number(event.severity) || 0) * 100).toFixed(0) + '%', 'severity');
          const lat = Number(event.lat); const lon = Number(event.lon);
          cell(row, Number.isFinite(lat) && Number.isFinite(lon) ? lat.toFixed(6) + ' N, ' + lon.toFixed(6) + ' E' : '—');
          cell(row, String(event.ttl_seconds || 0) + 's', 'ttl');
          cell(row, event.drift_corrected ? 'Corrected' : 'Nominal', event.drift_corrected ? 'drift-corrected' : 'drift-nominal');
          body.appendChild(row);
        });
      } catch (_) { /* The packet log remains available while map data reconnects. */ }
    }
    async function pollTraffic() {
      try {
        const response = await fetch('/api/communications?after=' + sequence + '&limit=500', { cache: 'no-store' });
        if (!response.ok) throw new Error('Communication API unavailable');
        const data = await response.json();
        (data.events || []).forEach(event => { sequence = Math.max(sequence, Number(event.sequence) || 0); appendPacket(event); });
        sequence = Math.max(sequence, Number(data.latest_sequence) || 0);
        document.getElementById('live-state').textContent = '● LIVE · ' + sequence + ' RECORDS';
      } catch (_) { document.getElementById('live-state').textContent = '● RECONNECTING'; }
    }
    pollTraffic(); refreshBeaconRegister();
    setInterval(pollTraffic, 350);
    setInterval(refreshBeaconRegister, 1000);
  </script>
</body>
</html>"""
