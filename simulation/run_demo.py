"""Run the emergency-response communications simulation."""

from __future__ import annotations

import ast
import datetime
import logging
import os
import queue
import sys
import threading
import time
import argparse
from pathlib import Path
from typing import List, Tuple

from rich.console import Console
from rich.logging import RichHandler
from rich.panel import Panel
from rich.table import Table

# Reconfigure stdout/stderr for UTF-8 on Windows
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

# Add project root to sys.path
PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from beacon.store import BeaconStore
from beacon.schema import BeaconMessage, EVENT_TYPES
from command_post.dashboard import app as flask_app, init_dashboard, set_latest_briefing, set_rover_coords, set_awaiting_authorization, wait_for_authorization, register_failure_targets, push_cycle_summary, set_executor_unit_state, get_selected_executor_unit_id, set_executor_mission_status
from command_post.map_state import LiveMap
from command_post.mission_generator import MissionGenerator
from executor_robot.navigator import ExecutorRobot
from executor_robot.roster import EXECUTOR_UNITS
from outside_network.frame_transform import GPSAnchor, DriftModel, LocalToGPS
from outside_network.lora_uplink import BaseSatelliteDriver, SimulatedSatelliteDriver, RetryQueue, SatelliteUplink
from outside_network.mission_relay import MissionRelay
from outside_network.receiver import BeaconReceiver
from simulation.config import (
    AnchorConfig,
    DriftConfig,
    SatelliteLinkConfig,
    GridConfig,
    WriterConfig,
    DashboardConfig,
    LOGS_DIR,
    ENVIRONMENT_NAME,
)
from writer_robot.explorer import WriterRobot, RobotState
from writer_robot.grid_world import GridWorld


logger = logging.getLogger("simulation")
console = Console(legacy_windows=False)
_flask_started = False  # Guard: only start the Flask thread once across loop cycles


def _configure_console_logging(root_logger: logging.Logger) -> None:
    """Add one concise console handler for operational simulation logs."""
    if any(getattr(handler, "name", None) == "simulation-console" for handler in root_logger.handlers):
        return
    console_handler = RichHandler(
        console=console,
        show_time=True,
        show_level=True,
        show_path=False,
        markup=False,
    )
    console_handler.name = "simulation-console"
    console_handler.setLevel(logging.INFO)
    console_handler.setFormatter(logging.Formatter("%(message)s"))
    root_logger.addHandler(console_handler)


def _wait_for_completion_beacon(
    beacon_store: BeaconStore,
    beacon_receiver: BeaconReceiver,
    live_map: LiveMap,
    executor_id: str,
    timeout_s: float = 5.0,
) -> bool:
    """Keep the existing ONA pipeline alive until completion reaches Command Post."""
    deadline = time.monotonic() + timeout_s
    completion_id = None
    while time.monotonic() < deadline:
        completion = next(
            (
                beacon for beacon in beacon_store.get_active()
                if beacon.writer_id == executor_id and beacon.event.type == "mission_completed"
            ),
            None,
        )
        if completion is not None:
            completion_id = completion.beacon_id
            if any(event.beacon_id == completion_id for event in live_map.get_active_events()):
                logger.info(
                    "Executor completion beacon delivered; beacon_id=%s forwarded=%s",
                    completion_id, beacon_receiver.beacons_forwarded,
                )
                return True
        time.sleep(0.05)

    logger.error(
        "Executor completion beacon was not delivered before timeout; beacon_id=%s forwarded=%s",
        completion_id or "not-created", beacon_receiver.beacons_forwarded,
    )
    return False


# ── Step 1: Architectural Constraint Verification ───────────────────────────

def verify_architectural_isolation() -> List[Tuple[str, bool, str]]:
    """
    Enforces the HARD CONSTRAINT:
    Zero direct code connections allowed between robots and Command Post.
    Uses Python AST to inspect all imports in writer_robot/, executor_robot/, and command_post/.
    """
    violations = []
    checks = []

    def check_file(py_file: Path, forbidden_packages: List[str]):
        with open(py_file, "r", encoding="utf-8") as f:
            tree = ast.parse(f.read(), filename=str(py_file))

        rel_path = py_file.relative_to(PROJECT_ROOT).as_posix()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    for forbidden in forbidden_packages:
                        if alias.name == forbidden or alias.name.startswith(f"{forbidden}."):
                            violations.append((rel_path, f"Direct import of '{alias.name}'"))
            elif isinstance(node, ast.ImportFrom):
                if node.module:
                    for forbidden in forbidden_packages:
                        if node.module == forbidden or node.module.startswith(f"{forbidden}."):
                            violations.append((rel_path, f"Direct import from '{node.module}'"))

    # 1. Writer robot must NOT import command_post or executor_robot
    writer_files = list((PROJECT_ROOT / "writer_robot").glob("*.py"))
    for f in writer_files:
        check_file(f, ["command_post", "executor_robot", "outside_network.mission_relay"])

    # 2. Executor robot must NOT import command_post or writer_robot
    executor_files = list((PROJECT_ROOT / "executor_robot").glob("*.py"))
    for f in executor_files:
        check_file(f, ["command_post", "writer_robot"])

    # 3. Command Post must NOT import writer_robot or executor_robot
    cp_files = list((PROJECT_ROOT / "command_post").glob("*.py"))
    for f in cp_files:
        check_file(f, ["writer_robot", "executor_robot"])

    checks.append((
        "Writer Robot Isolation (writer_robot/ -> command_post/)",
        not any("writer_robot" in v[0] for v in violations),
        "Zero direct imports detected" if not any("writer_robot" in v[0] for v in violations) else "Violation detected"
    ))
    checks.append((
        "Executor Robot Isolation (executor_robot/ -> command_post/)",
        not any("executor_robot" in v[0] for v in violations),
        "Zero direct imports detected" if not any("executor_robot" in v[0] for v in violations) else "Violation detected"
    ))
    checks.append((
        "Command Post Isolation (command_post/ -> robots/)",
        not any("command_post" in v[0] for v in violations),
        "Zero direct imports detected" if not any("command_post" in v[0] for v in violations) else "Violation detected"
    ))
    checks.append((
        "Outside Network Area (ONA) Relay Boundary",
        len(violations) == 0,
        "All communications strictly routed through ONA queues & satellite transport"
    ))

    return checks


# ── Step 2: Main Simulation Orchestration ────────────────────────────────────

def run_simulation(
    cycle_num: int = 1,
    headless: bool = False,
    max_beacons_target: int = 4,
    *,
    dashboard_port: int | None = None,
    heartbeat_monitor=None,
    heartbeat_dashboard_visible: bool = True,
    satellite_failure_after_command_post_events: int | None = None,
    baseline_alert=None,
    webots_visualization_enabled: bool = False,
    webots_fire_response_enabled: bool = False,
) -> bool:
    timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file_path = LOGS_DIR / f"demo_{timestamp_str}.log"

    # Setup file logging
    file_handler = logging.FileHandler(log_file_path, encoding="utf-8")
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(logging.Formatter("[%(asctime)s] [%(levelname)s] [%(name)s] %(message)s"))
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.INFO)
    root_logger.addHandler(file_handler)
    _configure_console_logging(root_logger)

    # Silence Flask/Werkzeug console noise
    logging.getLogger("werkzeug").setLevel(logging.ERROR)

    # Clear any previous cycle summary so the dashboard doesn't show stale data
    push_cycle_summary({})

    console.print(Panel.fit(
        "[bold cyan]SYSTEM OPERATION[/bold cyan]\n"
        f"Cycle {cycle_num}  •  Environment: {ENVIRONMENT_NAME}  •  Satellite uplink enabled",
        border_style="cyan",
    ))
    console.rule(style="dim")
    checks = verify_architectural_isolation()
    all_passed = True
    for name, passed, details in checks:
        if not passed:
            all_passed = False
            logger.error("Architecture validation failed: %s (%s)", name, details)

    if not all_passed:
        logger.error("Cycle %s aborted by architecture validation", cycle_num)
        return False
    logger.info("Architecture validation passed")
    
    # 1. Communications Queues (in-process representation of physical/radio boundaries)
    uplink_q: queue.Queue = queue.Queue()      # ONA satellite TX -> CP RX
    downlink_q: queue.Queue = queue.Queue()    # CP satellite TX -> ONA RX
    executor_inboxes: dict[str, queue.Queue] = {unit.unit_id: queue.Queue() for unit in EXECUTOR_UNITS}
    webots_firefighter_id = "WEBOTS-FIREFIGHTER-1"
    webots_firefighter_unit = {
        "unit_id": webots_firefighter_id,
        "display_name": "Webots Firefighter TurtleBot",
        "job_type": "fire",
        "icon_url": "https://cdn.jsdelivr.net/npm/@mdi/svg@7.4.47/svg/fire-truck.svg",
        "icon_tint": "#ef4444",
        "lat": 36.846000,
        "lon": 10.158000,
        "battery": 100.0,
        "status": "AVAILABLE",
    }
    if webots_fire_response_enabled:
        # This is a real ONA Role 4 target inbox, not a Command Post shortcut.
        executor_inboxes[webots_firefighter_id] = queue.Queue()
    status_q: queue.Queue = queue.Queue()      # Executor Status -> ONA

    # 2. Shared In-Memory Beacon Store (Physical beacons dropped in space)
    beacon_store = BeaconStore()
    # The Webots companion demo is intentionally event-led: its dashboard stays
    # empty until the real Pioneer controller confirms its first fire target.
    webots_first_fire_received = threading.Event()
    webots_first_fire_arrived = threading.Event()
    webots_victim_arrived = threading.Event()
    webots_completion_received = threading.Event()
    webots_beacon_world_positions: dict[str, tuple[float, float]] = {}
    webots_delivered_mission: dict | None = None
    webots_bridge_lock = threading.RLock()

    def _accept_webots_beacon(payload: dict) -> dict:
        """Convert a Webots detection into a clearly-labelled demo physical beacon."""
        event_type = str(payload.get("event_type", "")).strip().lower()
        if event_type not in {"fire_detected", "victim_detected"}:
            raise ValueError(
                "Webots event_type must be fire_detected or victim_detected"
            )
        if (
            webots_visualization_enabled
            and event_type != "fire_detected"
            and not webots_first_fire_received.is_set()
        ):
            raise ValueError(
                "Webots must confirm fire_detected before any other dashboard beacon"
            )
        try:
            world_x = float(payload["world_x"])
            world_y = float(payload["world_y"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("Webots beacon requires numeric world_x and world_y") from exc

        # The Webots house uses its own metre frame. This fixed, explicit
        # adapter maps it into the existing Writer/Beacon local-frame bounds;
        # ONA remains responsible for the only local→GPS transformation.
        local_x = round(max(0.0, min(18.0, (world_x - 9.0) * 0.8)), 2)
        local_y = round(max(0.0, min(18.0, (world_y - 9.0) * 0.8)), 2)
        # A confirmed victim is always a life-critical target. Enforce the
        # highest severity at the trusted Webots-to-Beacon boundary.
        severity = 1.0 if event_type == "victim_detected" else float(payload.get("severity", 0.95))
        beacon = BeaconMessage.create(
            writer_id="WEBOTS-PIONEER-SIM",
            event_type=event_type,
            severity=severity,
            x=local_x,
            y=local_y,
            z=0.0,
            heading_deg=float(payload.get("heading_deg", 0.0)),
        )
        if not beacon_store.add(beacon):
            raise ValueError("Webots beacon identifier collision")
        with webots_bridge_lock:
            webots_beacon_world_positions[beacon.beacon_id] = (world_x, world_y)
        logger.info(
            "[WEBOTS → BEACON NETWORK] %s | %s | Webots(%.2f, %.2f) → local(%.2f, %.2f)",
            beacon.beacon_id, event_type, world_x, world_y, local_x, local_y,
        )
        if event_type == "fire_detected":
            webots_first_fire_received.set()
        return {"beacon_id": beacon.beacon_id, "event_type": event_type}

    def _webots_executor_mission() -> dict:
        """Read only a briefing delivered by ONA Role 4 to the dedicated inbox."""
        nonlocal webots_delivered_mission
        if not webots_fire_response_enabled:
            return {"status": "inactive"}
        with webots_bridge_lock:
            if webots_delivered_mission is None:
                try:
                    payload = executor_inboxes[webots_firefighter_id].get_nowait()
                except queue.Empty:
                    return {"status": "waiting_for_ona_role_4"}
                if payload.get("target_unit_id") != webots_firefighter_id:
                    raise ValueError("ONA Role 4 payload target mismatch")
                briefing = payload.get("briefing", {})
                fire_waypoint = next(
                    (waypoint for waypoint in briefing.get("waypoints", []) if waypoint.get("event_type") == "fire_detected"),
                    None,
                )
                if fire_waypoint is None:
                    raise ValueError("Authorized briefing has no fire_detected target")
                world_target = webots_beacon_world_positions.get(fire_waypoint.get("beacon_id"))
                if world_target is None:
                    raise ValueError("Webots source position unavailable for authorized fire beacon")
                webots_delivered_mission = {
                    "relay_id": payload.get("relay_id"),
                    "briefing_id": briefing.get("briefing_id"),
                    "target": {"world_x": world_target[0], "world_y": world_target[1]},
                }
                logger.info(
                    "[ONA Role 4 → Webots Executor] relay=%s target=%s", payload.get("relay_id"), webots_firefighter_id
                )
            return {"status": "delivered", "mission": dict(webots_delivered_mission)}

    def _accept_webots_executor_completion(payload: dict) -> dict:
        """Adapt the physical Webots completion report into a normal BeaconStore write."""
        if not webots_fire_response_enabled:
            raise ValueError("Webots fire-response mode is inactive")
        if payload.get("unit_id") != webots_firefighter_id:
            raise ValueError("Completion unit is not the ONA-authorized Webots Firefighter")
        with webots_bridge_lock:
            mission = dict(webots_delivered_mission or {})
        if not mission or payload.get("briefing_id") != mission.get("briefing_id"):
            raise ValueError("Completion does not match an ONA-delivered mission")
        world_x, world_y = mission["target"]["world_x"], mission["target"]["world_y"]
        beacon = BeaconMessage.create(
            writer_id=webots_firefighter_id,
            event_type="mission_completed",
            severity=1.0,
            x=round(max(0.0, min(18.0, (world_x - 9.0) * 0.8)), 2),
            y=round(max(0.0, min(18.0, (world_y - 9.0) * 0.8)), 2),
        )
        if not beacon_store.add(beacon):
            raise ValueError("Webots completion beacon identifier collision")
        webots_completion_received.set()
        logger.info("[WEBOTS EXECUTOR → BEACON NETWORK] %s | mission_completed", beacon.beacon_id)
        return {"beacon_id": beacon.beacon_id, "event_type": "mission_completed"}

    # 3. Environment Grid World
    grid_cfg = GridConfig()
    grid = GridWorld(width=grid_cfg.width, height=grid_cfg.height)

    # 4. Outside Network Area (ONA) Georeferencing & satellite CARRY link
    anchor_cfg = AnchorConfig()
    drift_cfg = DriftConfig()
    anchor = GPSAnchor(
        origin_lat=anchor_cfg.origin_lat,
        origin_lon=anchor_cfg.origin_lon,
        origin_alt=anchor_cfg.origin_alt,
        bearing_deg=anchor_cfg.bearing_deg,
    )
    drift_model = DriftModel(
        drift_rate_x=drift_cfg.drift_rate_x,
        drift_rate_y=drift_cfg.drift_rate_y,
        correction_interval=drift_cfg.correction_interval,
    )
    transformer = LocalToGPS(anchor=anchor, drift_model=drift_model)

    satellite_cfg = SatelliteLinkConfig()
    satellite_driver = SimulatedSatelliteDriver(
        uplink_queue=uplink_q,
        downlink_queue=downlink_q,
        packet_loss_prob=satellite_cfg.nominal_packet_loss,
        latency_ms=satellite_cfg.one_way_latency_ms,
    )
    retry_queue = RetryQueue(satellite_driver)
    satellite_uplink = SatelliteUplink(satellite_driver, retry_queue)

    beacon_receiver = BeaconReceiver(beacon_store, transformer, satellite_uplink)
    mission_relay = MissionRelay(satellite_uplink, executor_inboxes)

    # 5. Command Post
    command_post_arrivals = 0

    def _on_event_ingested(event) -> None:
        """Optional isolated-demo observers; normal simulation leaves these off."""
        nonlocal command_post_arrivals
        command_post_arrivals += 1
        if (
            webots_visualization_enabled
            and event.writer_id == "WEBOTS-PIONEER-SIM"
            and event.event_type == "fire_detected"
        ):
            webots_first_fire_arrived.set()
        if (
            webots_visualization_enabled
            and event.writer_id == "WEBOTS-PIONEER-SIM"
            and event.event_type == "victim_detected"
        ):
            webots_victim_arrived.set()
        if heartbeat_monitor is not None and hasattr(heartbeat_monitor, "note_command_post_beacon"):
            heartbeat_monitor.note_command_post_beacon(event)
        if (
            satellite_failure_after_command_post_events is not None
            and command_post_arrivals == satellite_failure_after_command_post_events
        ):
            # Baseline comparison only: deliberately do not instantiate a mesh
            # transport, so later packets remain in the existing RetryQueue.
            satellite_driver.link_disrupted = True
            if baseline_alert is not None and hasattr(baseline_alert, "trigger"):
                baseline_alert.trigger()
            logger.warning(
                "[BaselineNoFailover] Satellite failed after Command Post beacon %s; RF mesh solution is NOT applied",
                event.beacon_id,
            )

    live_map = LiveMap(
        uplink_q,
        on_event_ingested=_on_event_ingested,
    )
    mission_generator = MissionGenerator(live_map, mission_relay)

    # Start Flask Web Dashboard in daemon thread (only once across loop cycles)
    dash_cfg = DashboardConfig(port=dashboard_port) if dashboard_port is not None else DashboardConfig()
    init_dashboard(
        live_map,
        mission_generator,
        satellite_driver=satellite_driver,
        satellite_uplink=satellite_uplink,
        beacon_store=beacon_store,
        executor_units=([webots_firefighter_unit] if webots_fire_response_enabled else [unit.to_dashboard_dict() for unit in EXECUTOR_UNITS]),
        heartbeat_monitor=heartbeat_monitor if heartbeat_dashboard_visible else None,
        baseline_alert=baseline_alert,
        webots_visualization_enabled=webots_visualization_enabled,
        webots_beacon_source=_accept_webots_beacon if webots_visualization_enabled else None,
        webots_mission_source=_webots_executor_mission if webots_fire_response_enabled else None,
        webots_completion_source=_accept_webots_executor_completion if webots_fire_response_enabled else None,
        webots_world=(PROJECT_ROOT / "webots_fire_response_demo" / "worlds" / "fire_response_chain.wbt") if webots_fire_response_enabled else None,
        webots_launch_label="Writer-to-Firefighter response chain" if webots_fire_response_enabled else "Pioneer LiDAR visualization",
    )
    global _flask_started
    if not _flask_started:
        dash_thread = threading.Thread(
            target=lambda: flask_app.run(
                host=dash_cfg.host,
                port=dash_cfg.port,
                debug=False,
                use_reloader=False,
            ),
            name="DashboardThread",
            daemon=True,
        )
        dash_thread.start()
        _flask_started = True

    # Start ONA and Command Post Workers
    beacon_receiver.start()
    live_map.start()
    if heartbeat_monitor is not None and hasattr(heartbeat_monitor, "start"):
        heartbeat_monitor.start()
    retry_thread = threading.Thread(target=retry_queue.drain, name="RetryQueueWorker", daemon=True)
    retry_thread.start()

    logger.info(
        "Pipeline ready; satellite_latency_ms=%s dashboard=http://%s:%s",
        satellite_cfg.one_way_latency_ms, dash_cfg.host, dash_cfg.port,
    )
    writer_cfg = WriterConfig()
    writer = WriterRobot(
        robot_id=writer_cfg.robot_id,
        grid=grid,
        beacon_store=beacon_store,
        start_row=writer_cfg.start_row,
        start_col=writer_cfg.start_col,
        battery_pct=writer_cfg.battery_pct,
        step_callback=lambda robot: set_rover_coords(writer_battery=robot.battery_pct),
        rng_seed=42,
    )
    register_failure_targets(writer=writer, satellite_driver=satellite_driver, beacon_store=beacon_store)

    logger.info("Writer %s exploration started", writer.robot_id)

    # In the Webots companion demo, dashboard traffic begins only after the
    # Pioneer confirms fire. Normal and resilience demos retain the simulated
    # Writer Robot loop below without any change.
    max_steps = 250
    if webots_visualization_enabled:
        logger.info(
            "Webots mode ready: waiting for the Pioneer to detect fire before the first dashboard beacon"
        )
        if not webots_first_fire_received.wait(timeout=300.0):
            logger.error("Webots fire detection timed out; no dashboard beacon was released")
            return False
        if not webots_first_fire_arrived.wait(timeout=8.0):
            logger.error("Webots fire beacon did not reach the Command Post in time")
            return False
        logger.info("Webots fire beacon confirmed at Command Post; waiting for victim confirmation before briefing")
        if not webots_victim_arrived.wait(timeout=120.0):
            logger.error("Webots victim beacon did not reach the Command Post in time; briefing not generated")
            return False
        logger.info("Webots victim beacon confirmed at Command Post; generating combined fire and victim briefing")
    else:
        while writer.is_alive() and len(writer.beacons_dropped) < max_beacons_target and writer._step < max_steps:
            frontiers = writer.grid.get_frontiers()
            if not frontiers:
                break
            target = writer._nearest_frontier(frontiers)
            if target is None:
                break
            if not writer._step_toward(target):
                writer.grid.grid[target[0], target[1]] = 2  # visited
                continue

            writer._step += 1
            writer.battery_pct = max(0.0, writer.battery_pct - 0.2)
            writer._scan_and_maybe_drop()
            time.sleep(writer_cfg.step_delay_s)

    # Let background receiver and satellite store-and-forward finish relaying
    time.sleep(1.5)

    telemetry = writer.get_telemetry()
    logger.info(
        "Writer %s exploration complete; steps=%s area_pct=%s battery_pct=%s beacons=%s",
        writer.robot_id, telemetry.current_step, telemetry.exploration_pct,
        telemetry.battery_pct, telemetry.beacons_dropped,
    )
    writer_table = Table(title="Writer Robot Status", show_header=True, header_style="bold yellow")
    writer_table.add_column("Metric", style="white")
    writer_table.add_column("Value", style="bold cyan", justify="right")
    writer_table.add_row("Steps", str(telemetry.current_step))
    writer_table.add_row("Coverage", f"{telemetry.exploration_pct}%")
    writer_table.add_row("Battery", f"{telemetry.battery_pct}%")
    writer_table.add_row("Beacons", str(telemetry.beacons_dropped))
    console.print(writer_table)

    writer_beacon_table = Table(title="Writer Beacon Register", show_header=True, header_style="bold yellow")
    writer_beacon_table.add_column("Beacon ID", style="bold white")
    writer_beacon_table.add_column("Event Type", style="cyan")
    writer_beacon_table.add_column("Severity", justify="right", style="yellow")
    writer_beacon_table.add_column("Local Position", style="dim")
    writer_beacon_table.add_column("TTL", justify="right", style="green")
    for beacon in writer.beacons_dropped:
        writer_beacon_table.add_row(
            beacon.beacon_id,
            beacon.event.type,
            f"{beacon.event.severity:.2f}",
            f"({beacon.position_local.x:.1f}, {beacon.position_local.y:.1f}, {beacon.position_local.z:.1f})",
            f"{beacon.ttl_seconds}s",
        )
    console.print(writer_beacon_table)
    console.rule(style="dim")

    event_types_found = set(b.event.type for b in writer.beacons_dropped)
    if len(event_types_found) < 2:
        logger.warning("Writer produced only %s event type(s)", len(event_types_found))
    else:
        logger.info("Writer event types: %s", ", ".join(sorted(event_types_found)))

    stats = retry_queue.stats()
    logger.info(
        "Uplink status; forwarded=%s drift_corrections=%s sent=%s pending=%s",
        beacon_receiver.beacons_forwarded, drift_model.corrections_applied(),
        stats["sent"], stats["pending"],
    )
    uplink_table = Table(title="Uplink Status", show_header=True, header_style="bold magenta")
    uplink_table.add_column("Forwarded", justify="right", style="cyan")
    uplink_table.add_column("Drift Corrections", justify="right", style="yellow")
    uplink_table.add_column("Sent", justify="right", style="green")
    uplink_table.add_column("Pending", justify="right", style="white")
    uplink_table.add_column("Failed", justify="right", style="red")
    uplink_table.add_row(
        str(beacon_receiver.beacons_forwarded),
        str(drift_model.corrections_applied()),
        str(stats["sent"]),
        str(stats["pending"]),
        str(stats["failed"]),
    )
    console.print(uplink_table)
    console.rule(style="dim")
    
    # Check LiveMap state
    active_events = live_map.get_active_events()
    logger.info("Command Post received %s active beacon event(s)", len(active_events))
    event_table = Table(title="Command Post Event Feed", show_header=True, header_style="bold cyan")
    event_table.add_column("Beacon ID", style="bold white")
    event_table.add_column("Event Type", style="cyan")
    event_table.add_column("Severity", justify="right", style="yellow")
    event_table.add_column("Coordinates", style="dim")
    event_table.add_column("TTL", justify="right", style="green")
    event_table.add_column("Drift", style="magenta")
    for event in active_events:
        event_table.add_row(
            event.beacon_id,
            event.event_type,
            f"{event.severity:.2f}",
            f"{event.lat:.6f}, {event.lon:.6f}",
            f"{event.ttl_seconds}s",
            "Corrected" if event.drift_corrected else "Nominal",
        )
    console.print(event_table)
    console.rule(style="dim")
    briefing = mission_generator.generate()
    if briefing is None:
        logger.error("Mission briefing generation failed: no active events")
        return False

    set_latest_briefing(briefing.to_dict())
    # Set Executor status to AWAITING_AUTHORIZATION — do NOT start navigation yet
    set_awaiting_authorization()
    logger.info(
        "Mission %s prepared; waypoints=%s; awaiting operator authorization",
        briefing.briefing_id, len(briefing.waypoints),
    )
    briefing_table = Table(title="Mission Briefing", show_header=True, header_style="bold cyan")
    briefing_table.add_column("Mission ID", style="bold white")
    briefing_table.add_column("Waypoints", justify="right", style="cyan")
    briefing_table.add_column("Status", style="yellow")
    briefing_table.add_row(briefing.briefing_id, str(len(briefing.waypoints)), "Awaiting operator authorization")
    console.print(briefing_table)
    waypoint_table = Table(title="Mission Waypoints", show_header=True, header_style="bold cyan")
    waypoint_table.add_column("Waypoint", style="bold white")
    waypoint_table.add_column("Beacon", style="white")
    waypoint_table.add_column("Event Type", style="cyan")
    waypoint_table.add_column("Priority", justify="right", style="yellow")
    waypoint_table.add_column("Coordinates", style="dim")
    for waypoint in briefing.waypoints:
        waypoint_table.add_row(
            waypoint.waypoint_id,
            waypoint.beacon_id,
            waypoint.event_type,
            f"{waypoint.priority * 100:.0f}%",
            f"{waypoint.lat:.6f}, {waypoint.lon:.6f}",
        )
    console.print(waypoint_table)
    console.rule(style="dim")
    def _unit_step_callback(unit_id: str):
        def update_unit(**kwargs) -> None:
            set_executor_unit_state(
                unit_id,
                executor_coords=kwargs.get("executor_coords"),
                battery=kwargs.get("executor_battery"),
            )
        return update_unit

    executors = {
        unit.unit_id: ExecutorRobot(
            robot_id=unit.unit_id,
            executor_inbox=executor_inboxes[unit.unit_id],
            status_queue=status_q,
            beacon_store=beacon_store,
            start_lat=unit.start_lat,
            start_lon=unit.start_lon,
            battery_pct=unit.initial_battery,
            step_callback=_unit_step_callback(unit.unit_id),
        )
        for unit in EXECUTOR_UNITS
    }

    logger.info("Awaiting operator unit selection and mission authorization")
    authorized = wait_for_authorization(timeout_s=300.0)
    if not authorized:
        logger.error("Mission authorization timed out")
        return False
    selected_unit_id = get_selected_executor_unit_id()
    if webots_fire_response_enabled and selected_unit_id == webots_firefighter_id:
        logger.info("Mission authorized; ONA Role 4 is dispatching to the Webots Firefighter")
        success = webots_completion_received.wait(timeout=240.0)
        if not success:
            logger.error("Webots Firefighter did not report mission completion before timeout")
        completion_beacon_delivered = (
            _wait_for_completion_beacon(beacon_store, beacon_receiver, live_map, webots_firefighter_id)
            if success else False
        )
        set_executor_unit_state(
            selected_unit_id,
            battery=86.0 if success else 100.0,
            status="COMPLETE" if success else "FAILED",
        )
        executor = None
    elif selected_unit_id not in executors:
        logger.error("Mission authorization completed without a valid selected unit")
        return False
    else:
        executor = executors[selected_unit_id]
        logger.info("Mission authorized; targeted Executor dispatch started for %s", selected_unit_id)

        # Run executor mission (navigation starts only now)
        success = executor.await_and_run(timeout_s=5.0)
        completion_beacon_delivered = (
            _wait_for_completion_beacon(beacon_store, beacon_receiver, live_map, executor.robot_id)
            if success else False
        )
        set_executor_unit_state(
            selected_unit_id,
            executor_coords=(executor.lat, executor.lon),
            battery=executor.battery_pct,
            status="COMPLETE" if success else "FAILED",
        )
    set_executor_mission_status("COMPLETE" if success else "FAILED")

    # Collect status reports from ONA status queue
    reports = []
    while not status_q.empty():
        reports.append(status_q.get_nowait())
    if webots_fire_response_enabled and executor is None and success:
        # The physical response unit reports completion through its beacon, not
        # the simulated navigator status queue. Keep the summary truthful.
        reports.append({
            "report_type": "arrival",
            "waypoint_id": briefing.waypoints[0].waypoint_id if briefing.waypoints else None,
            "beacon_id": briefing.waypoints[0].beacon_id if briefing.waypoints else None,
            "message": "Webots Firefighter reached the ONA-authorized fire target and extinguished it.",
        })

    arrivals = [r for r in reports if r['report_type'] == 'arrival']
    skipped = [r for r in reports if r.get('report_type') == 'skipped']

    # ── Push cycle summary to dashboard (triggers summary modal) ──────────────
    # Collect any failures that were triggered via dashboard buttons this cycle
    import copy as _copy
    import command_post.dashboard as _dash_mod
    with _dash_mod._failure_log_lock:
        dash_failures = _copy.deepcopy(_dash_mod._failure_log)

    push_cycle_summary({
        "cycle": cycle_num,
        "mission_id": briefing.briefing_id,
        "timestamp": time.time(),
        "targets_visited": len(arrivals),
        "targets_total": len(briefing.waypoints),
        "completion_pct": round(100 * len(arrivals) / max(len(briefing.waypoints), 1)),
        "beacons_by_priority": [
            {
                "id": w.beacon_id,
                "type": w.event_type,
                "priority_pct": round(w.priority * 100),
            }
            for w in briefing.waypoints
        ],
        "beacons_skipped_ttl": len(skipped),
        "drift_corrections": drift_model.corrections_applied(),
        "drift_total_adjustment_m": drift_model.total_adjustment_m(),
        "failures": dash_failures,
    })

    logger.info(
        "Executor mission complete; visited=%s/%s direct_beacon_reads=%s status_reports=%s completion_beacon_delivered=%s",
        len(arrivals), len(briefing.waypoints), executor._direct_beacons_read if executor else "Webots LiDAR", len(reports), completion_beacon_delivered,
    )
    executor_table = Table(title="Executor Mission Status", show_header=True, header_style="bold green")
    executor_table.add_column("Visited Targets", justify="right", style="white")
    executor_table.add_column("Direct Beacon Reads", justify="right", style="cyan")
    executor_table.add_column("Completion Beacon", style="green" if completion_beacon_delivered else "red")
    executor_table.add_row(
        f"{len(arrivals)}/{len(briefing.waypoints)}",
        str(executor._direct_beacons_read) if executor else "Webots LiDAR response",
        "Delivered to Command Post" if completion_beacon_delivered else "Delivery timed out",
    )
    console.print(executor_table)
    report_table = Table(title="Executor Status Reports", show_header=True, header_style="bold green")
    report_table.add_column("Type", style="bold white")
    report_table.add_column("Waypoint", style="cyan")
    report_table.add_column("Beacon", style="white")
    report_table.add_column("Details", style="dim")
    for report in reports:
        report_table.add_row(
            report["report_type"].upper(),
            report.get("waypoint_id") or "--",
            report.get("beacon_id") or "--",
            report["message"],
        )
    console.print(report_table)
    console.rule(style="dim")

    # Clean shutdown of background threads
    beacon_receiver.stop()
    live_map.stop()
    if heartbeat_monitor is not None and hasattr(heartbeat_monitor, "stop"):
        heartbeat_monitor.stop()

    overall_success = success and completion_beacon_delivered
    if overall_success:
        console.print(Panel.fit(
            "[bold green]MISSION COMPLETED[/bold green]\n"
            f"Operational log: {log_file_path}",
            border_style="green",
        ))
        console.rule(style="dim")
        logger.info("Cycle %s completed successfully; log=%s", cycle_num, log_file_path)
    else:
        logger.error("Cycle %s completed with errors; log=%s", cycle_num, log_file_path)
    return overall_success


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the emergency-response simulation")
    parser.add_argument(
        "--serve",
        action="store_true",
        help="Keep the Flask dashboard alive after the demo completes",
    )
    parser.add_argument(
        "--keep-alive",
        dest="serve",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="Run only one cycle (no loop)",
    )
    parser.add_argument(
        "--dashboard-port",
        type=int,
        default=None,
        help="Command Post dashboard port (default: 5000)",
    )
    parser.add_argument(
        "--webots-visualization",
        action="store_true",
        help="Enable the independent Pioneer LiDAR Webots launch button for this live mission only",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    _stop_event = threading.Event()

    def _shutdown(sig=None, frame=None):
        logger.info("Shutdown requested; stopping after the current cycle")
        _stop_event.set()

    import signal
    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    cycle = 0
    exit_code = 0

    try:
        while not _stop_event.is_set():
            cycle += 1
            if cycle > 1:
                logger.info("Cycle %s starting; state reset complete", cycle)

            success = run_simulation(
                cycle_num=cycle,
                dashboard_port=args.dashboard_port,
                webots_visualization_enabled=args.webots_visualization,
            )

            logger.info("Cycle %s %s", cycle, "completed" if success else "failed")

            if not success:
                exit_code = 1

            if args.once or _stop_event.is_set():
                break

            # Short pause between cycles so the judge can see the banner
            for _ in range(30):
                if _stop_event.is_set():
                    break
                time.sleep(0.1)

    except KeyboardInterrupt:
        _shutdown()

    # Keep Flask alive after loop ends
    if args.serve and not _stop_event.is_set():
        active_port = args.dashboard_port or DashboardConfig().port
        logger.info("Dashboard remains available at http://127.0.0.1:%s; press Ctrl+C to stop", active_port)
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass

    logger.info("Simulation process exiting")
    sys.exit(exit_code)
