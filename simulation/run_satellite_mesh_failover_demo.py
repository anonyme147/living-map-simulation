"""Standalone satellite-failure to automatic RF-mesh failover demonstration.

This scenario starts with the normal satellite CARRY path.  Immediately after
the second satellite-carried Writer beacon reaches Command Post, it invokes the
existing dashboard satellite-failure endpoint; every later ONA Role 3 beacon
packet is detected and carried over the RF mesh.  No existing demo transport
is changed.
"""

from __future__ import annotations

import argparse
import json
import logging
import queue
import sys
import threading
import time
from pathlib import Path
from typing import Optional
from urllib.request import Request, urlopen

from rich.console import Console
from rich.logging import RichHandler
from rich.panel import Panel
from rich.table import Table


PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass


from beacon.store import BeaconStore
from command_post.dashboard import (
    app as flask_app,
    get_selected_executor_unit_id,
    init_dashboard,
    push_cycle_summary,
    register_failure_targets,
    set_awaiting_authorization,
    set_executor_mission_status,
    set_executor_unit_state,
    set_latest_briefing,
    set_rover_coords,
    wait_for_authorization,
)
from command_post.map_state import LiveMap
from command_post.mission_generator import MissionGenerator
from executor_robot.navigator import ExecutorRobot
from executor_robot.roster import EXECUTOR_UNITS
from outside_network.frame_transform import DriftModel, GPSAnchor, LocalToGPS
from outside_network.lora_uplink import RetryQueue, SimulatedSatelliteDriver
from outside_network.mission_relay import MissionRelay
from outside_network.receiver import BeaconReceiver
from outside_network.rf_mesh import DEFAULT_MESH_NODE_COUNT, MeshRelayDriver, RFMeshTopology
from outside_network.satellite_mesh_failover import FailoverSatelliteUplink, SatelliteMeshFailoverDriver
from simulation.config import AnchorConfig, DashboardConfig, DriftConfig, GridConfig, WriterConfig
from writer_robot.explorer import WriterRobot
from writer_robot.grid_world import GridWorld


console = Console(legacy_windows=False)
logger = logging.getLogger("satellite_mesh_failover_demo")
_flask_started = False
SECOND_ARRIVAL_FAILURE_DELAY_S = 1.0


class _FailoverRFTraceFilter(logging.Filter):
    """Keep PowerShell focused on the failover evidence, not dashboard polling."""

    _TRACE_LOGGERS = (
        "satellite_mesh_failover_demo",
        "outside_network.satellite_mesh_failover",
        "outside_network.rf_mesh",
        "command_post.map_state",
    )

    def filter(self, record: logging.LogRecord) -> bool:
        return record.name.startswith(self._TRACE_LOGGERS)


def _configure_logging() -> None:
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.INFO)
    # The dashboard polls frequently.  Keep those HTTP 200 access records out
    # of the terminal so its narrative remains the actual RF packet flow.
    logging.getLogger("werkzeug").setLevel(logging.ERROR)
    handler = next(
        (item for item in root_logger.handlers if getattr(item, "name", None) == "satellite-mesh-failover-console"),
        None,
    )
    if handler is None:
        handler = RichHandler(console=console, show_time=True, show_level=True, show_path=False, markup=False)
        handler.name = "satellite-mesh-failover-console"
        handler.setFormatter(logging.Formatter("%(message)s"))
        root_logger.addHandler(handler)

    # Show the actual handoff and every RF hop after satellite loss.  Browser
    # polling and unrelated Writer exploration remain hidden, so this is a
    # recording-friendly packet trace rather than a wall of HTTP log lines.
    handler.setLevel(logging.INFO)
    handler.filters.clear()
    handler.addFilter(_FailoverRFTraceFilter())


def _wait_until(predicate, timeout_s: float, *, interval_s: float = 0.1) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval_s)
    return bool(predicate())


def _unit_step_callback(unit_id: str):
    def update_unit(**kwargs) -> None:
        set_executor_unit_state(
            unit_id,
            executor_coords=kwargs.get("executor_coords"),
            battery=kwargs.get("executor_battery"),
        )
    return update_unit


def _render_writer_beacon_register(beacons: list) -> None:
    """Print a compact, human-readable Writer-to-ONA beacon register."""
    table = Table(title="WRITER → ONA | BEACON REGISTER", header_style="bold yellow")
    table.add_column("#", justify="right", style="dim")
    table.add_column("Beacon", style="bold white")
    table.add_column("Event", style="cyan")
    table.add_column("Severity", justify="right", style="yellow")
    table.add_column("Local position", style="white")
    table.add_column("TTL", justify="right", style="green")
    for number, beacon in enumerate(beacons, start=1):
        table.add_row(
            str(number),
            beacon.beacon_id,
            beacon.event.type.replace("_", " ").upper(),
            f"{beacon.event.severity:.2f}",
            f"({beacon.position_local.x:.1f}, {beacon.position_local.y:.1f})",
            f"{beacon.ttl_seconds}s",
        )
    console.print(table)


def _render_carry_ledger(beacons: list, live_map: LiveMap, transport_stats: dict) -> None:
    """Print the verified carrier path for each Writer beacon."""
    events = {event.beacon_id: event for event in live_map.get_active_events()}
    table = Table(title="ONA ROLE 3 | VERIFIED CARRY LEDGER", header_style="bold magenta")
    table.add_column("Packet", style="bold white")
    table.add_column("Beacon", style="white")
    table.add_column("Carrier", style="cyan")
    table.add_column("Command Post receipt", style="green")
    table.add_column("Result", style="bold green")
    for number, beacon in enumerate(beacons, start=1):
        event = events.get(beacon.beacon_id)
        carrier = "SATELLITE" if number <= 2 else "RF MESH"
        receipt = time.strftime("%H:%M:%S", time.localtime(event.received_at)) if event else "not received"
        table.add_row(
            event.packet_id if event else f"PKT{number:04d}",
            beacon.beacon_id,
            carrier,
            receipt,
            "DELIVERED" if event else "MISSING",
        )
    console.print(table)
    console.print(
        Panel.fit(
            f"[bold red]SATELLITE FAILURE[/bold red]  1 second after P2 receipt\n"
            f"[bold magenta]RF MESH ACTIVE[/bold magenta]  Satellite packets: {transport_stats['satellite_packets']}  |  "
            f"Mesh packets: {transport_stats['mesh_packets']}",
            title="FAILOVER HANDOFF",
            border_style="magenta",
        )
    )
    console.rule(style="dim")


def _trigger_existing_satellite_failure(port: int) -> dict:
    """Use the real dashboard route—not a private mutation—to inject failure."""
    request = Request(
        f"http://127.0.0.1:{port}/api/failure/satellite",
        data=b"{}",
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urlopen(request, timeout=5) as response:
        payload = json.loads(response.read().decode("utf-8"))
        if response.status != 200:
            raise RuntimeError(f"Satellite-failure endpoint returned HTTP {response.status}: {payload}")
    return payload


def _wait_for_completion_beacon(
    beacon_store: BeaconStore,
    receiver: BeaconReceiver,
    live_map: LiveMap,
    executor_id: str,
    timeout_s: float = 25.0,
) -> bool:
    completion_id: Optional[str] = None

    def delivered() -> bool:
        nonlocal completion_id
        if completion_id is None:
            completion = next(
                (
                    beacon for beacon in beacon_store.get_active()
                    if beacon.writer_id == executor_id and beacon.event.type == "mission_completed"
                ),
                None,
            )
            if completion is not None:
                completion_id = completion.beacon_id
        return completion_id is not None and any(
            event.beacon_id == completion_id for event in live_map.get_active_events()
        )

    result = _wait_until(delivered, timeout_s)
    logger.info(
        "[FailoverDemo] Completion beacon via automatic mesh failover=%s beacon=%s forwarded=%s",
        result,
        completion_id or "not-created",
        receiver.beacons_forwarded,
    )
    return result


def run_satellite_mesh_failover_demo(
    *,
    dashboard_port: int = 5004,
    node_count: int = DEFAULT_MESH_NODE_COUNT,
    seed: Optional[int] = None,
) -> bool:
    """Run a separate mission proving satellite loss triggers mesh continuity."""
    global _flask_started
    _configure_logging()
    receiver: Optional[BeaconReceiver] = None
    live_map: Optional[LiveMap] = None
    try:
        console.print(Panel.fit(
            "[bold cyan]SATELLITE FAILURE → AUTOMATIC RF MESH FAILOVER[/bold cyan]\n"
            "Normal ONA CARRY starts on satellite; only a real satellite outage activates mesh routing.",
            border_style="cyan",
        ))

        uplink_queue: queue.Queue = queue.Queue()
        downlink_queue: queue.Queue = queue.Queue()
        status_queue: queue.Queue = queue.Queue()
        executor_inboxes: dict[str, queue.Queue] = {unit.unit_id: queue.Queue() for unit in EXECUTOR_UNITS}
        beacon_store = BeaconStore()

        grid_config = GridConfig()
        writer_config = WriterConfig()
        grid = GridWorld(width=grid_config.width, height=grid_config.height)
        anchor_config, drift_config = AnchorConfig(), DriftConfig()
        transformer = LocalToGPS(
            anchor=GPSAnchor(
                origin_lat=anchor_config.origin_lat,
                origin_lon=anchor_config.origin_lon,
                origin_alt=anchor_config.origin_alt,
                bearing_deg=anchor_config.bearing_deg,
            ),
            drift_model=DriftModel(
                drift_rate_x=drift_config.drift_rate_x,
                drift_rate_y=drift_config.drift_rate_y,
                correction_interval=drift_config.correction_interval,
            ),
        )

        dashboard_config = DashboardConfig(port=dashboard_port)
        satellite_driver = SimulatedSatelliteDriver(
            uplink_queue=uplink_queue,
            downlink_queue=downlink_queue,
            packet_loss_prob=0.0,
            # Presentation pacing only: each real pre-failure satellite frame
            # remains visible long enough for the dashboard's active-CARRY
            # visual to establish Phase 1 before the scripted outage.
            latency_ms=3000,
        )
        topology = RFMeshTopology.generate(
            building_lat=dashboard_config.building_site_lat,
            building_lon=dashboard_config.building_site_lon,
            command_post_lat=dashboard_config.command_post_lat,
            command_post_lon=dashboard_config.command_post_lon,
            node_count=node_count,
            seed=seed,
        )
        mesh_driver = MeshRelayDriver(uplink_queue, downlink_queue, topology)
        second_command_post_arrival = threading.Event()
        failover_driver = SatelliteMeshFailoverDriver(
            satellite_driver,
            mesh_driver,
            second_command_post_arrival=second_command_post_arrival,
        )
        retry_queue = RetryQueue(failover_driver)
        failover_uplink = FailoverSatelliteUplink(failover_driver, retry_queue)
        receiver = BeaconReceiver(beacon_store, transformer, failover_uplink)
        failure_injected = threading.Event()
        failure_result: dict = {}
        confirmed_arrivals: list[dict] = []

        def trigger_failure_after_second_command_post_arrival(event) -> None:
            """Fail only after two satellite-carried Writer beacons are in CP state."""
            if event.writer_id != writer_config.robot_id or failure_injected.is_set():
                return
            confirmed_arrivals.append({
                "beacon_id": event.beacon_id,
                "packet_id": event.packet_id,
                "received_at": event.received_at,
            })
            if len(confirmed_arrivals) < 2:
                logger.info(
                    "[FailoverDemo] Command Post confirmed satellite beacon %s (packet=%s); satellite remains active",
                    event.beacon_id,
                    event.packet_id or "unknown",
                )
                return
            logger.warning(
                "[FailoverDemo] Command Post confirmed second beacon %s (packet=%s); "
                "holding packet three and triggering /api/failure/satellite in %.1fs",
                event.beacon_id,
                event.packet_id or "unknown",
                SECOND_ARRIVAL_FAILURE_DELAY_S,
            )
            try:
                # Packet three remains blocked in SatelliteMeshFailoverDriver
                # until this callback signals the completed failover action.
                time.sleep(SECOND_ARRIVAL_FAILURE_DELAY_S)
                failure_result["response"] = _trigger_existing_satellite_failure(dashboard_port)
                failure_injected.set()
                second_command_post_arrival.set()
                logger.warning(
                    "[FailoverDemo] Satellite failure injected %.1fs after Command Post arrival "
                    "of %s",
                    SECOND_ARRIVAL_FAILURE_DELAY_S,
                    event.beacon_id,
                )
            except Exception as exc:
                failure_result["error"] = str(exc)
                logger.exception("[FailoverDemo] Failed to invoke the existing satellite-failure endpoint")

        live_map = LiveMap(uplink_queue, on_event_ingested=trigger_failure_after_second_command_post_arrival)
        mission_relay = MissionRelay(failover_uplink, executor_inboxes)
        mission_generator = MissionGenerator(live_map, mission_relay)

        init_dashboard(
            live_map,
            mission_generator,
            satellite_driver=failover_driver,
            satellite_uplink=failover_uplink,
            beacon_store=beacon_store,
            executor_units=[unit.to_dashboard_dict() for unit in EXECUTOR_UNITS],
        )
        if not _flask_started:
            threading.Thread(
                target=lambda: flask_app.run(
                    host=dashboard_config.host,
                    port=dashboard_config.port,
                    debug=False,
                    use_reloader=False,
                ),
                name="SatelliteMeshFailoverDashboard",
                daemon=True,
            ).start()
            _flask_started = True

        receiver.start()
        live_map.start()
        threading.Thread(target=retry_queue.drain, name="SatelliteMeshFailoverRetryQueue", daemon=True).start()
        logger.info("[FailoverDemo] Satellite primary ready; RF mesh is intentionally hidden until failure")

        writer = WriterRobot(
            robot_id=writer_config.robot_id,
            grid=grid,
            beacon_store=beacon_store,
            start_row=writer_config.start_row,
            start_col=writer_config.start_col,
            battery_pct=writer_config.battery_pct,
            step_callback=lambda robot: set_rover_coords(writer_battery=robot.battery_pct),
            rng_seed=42,
        )
        register_failure_targets(writer=writer, satellite_driver=failover_driver, beacon_store=beacon_store)

        while writer.is_alive() and len(writer.beacons_dropped) < 3 and writer._step < 220:
            frontier = writer._nearest_frontier(writer.grid.get_frontiers())
            if frontier is None or not writer._step_toward(frontier):
                break
            writer._step += 1
            writer.battery_pct = max(0.0, writer.battery_pct - 0.2)
            writer._scan_and_maybe_drop()
            time.sleep(writer_config.step_delay_s)

        expected_beacons = len(writer.beacons_dropped)
        if expected_beacons < 3:
            logger.error("[FailoverDemo] Writer produced only %s beacon(s); cannot prove third-beacon mesh failover", expected_beacons)
            return False
        _render_writer_beacon_register(writer.beacons_dropped)
        console.rule(style="dim")
        if not _wait_until(lambda: len(live_map.get_active_events()) >= expected_beacons, timeout_s=25.0):
            logger.error("[FailoverDemo] Beacon traffic did not fully reach Command Post after failover")
            return False
        initial_transport_stats = failover_driver.stats()
        if not failure_injected.is_set() or len(confirmed_arrivals) != 2:
            logger.error("[FailoverDemo] Second Command Post arrival did not trigger satellite failure: %s", failure_result)
            return False
        if initial_transport_stats["satellite_packets"] != 2 or initial_transport_stats["mesh_packets"] < expected_beacons - 2:
            logger.error(
                "[FailoverDemo] Expected exactly two satellite packets then mesh-only Writer traffic: %s",
                initial_transport_stats,
            )
            return False
        logger.info(
            "[FailoverDemo] Arrival-triggered failover verified: second=%s satellite_packets=2 mesh_packets=%s",
            confirmed_arrivals[1]["beacon_id"],
            initial_transport_stats["mesh_packets"],
        )
        _render_carry_ledger(writer.beacons_dropped, live_map, initial_transport_stats)

        briefing = mission_generator.generate()
        if briefing is None:
            logger.error("[FailoverDemo] No mission briefing could be generated")
            return False
        set_latest_briefing(briefing.to_dict())
        set_awaiting_authorization()
        logger.info("[FailoverDemo] Mission %s ready; select a unit and authorize from dashboard", briefing.briefing_id)

        executors = {
            unit.unit_id: ExecutorRobot(
                robot_id=unit.unit_id,
                executor_inbox=executor_inboxes[unit.unit_id],
                status_queue=status_queue,
                beacon_store=beacon_store,
                start_lat=unit.start_lat,
                start_lon=unit.start_lon,
                battery_pct=unit.initial_battery,
                step_callback=_unit_step_callback(unit.unit_id),
            )
            for unit in EXECUTOR_UNITS
        }
        if not wait_for_authorization(timeout_s=300.0):
            logger.error("[FailoverDemo] Mission authorization timed out")
            return False
        selected_unit_id = get_selected_executor_unit_id()
        if selected_unit_id not in executors:
            logger.error("[FailoverDemo] Invalid selected Executor unit: %s", selected_unit_id)
            return False

        executor = executors[selected_unit_id]
        set_executor_unit_state(selected_unit_id, status="EN_ROUTE")
        success = executor.await_and_run(timeout_s=8.0)
        completion_delivered = (
            _wait_for_completion_beacon(beacon_store, receiver, live_map, executor.robot_id)
            if success else False
        )
        transport_stats = failover_driver.stats()
        failover_verified = (
            failure_injected.is_set()
            and len(confirmed_arrivals) == 2
            and confirmed_arrivals[0].get("packet_id") == "PKT0001"
            and confirmed_arrivals[1].get("packet_id") == "PKT0002"
            and initial_transport_stats["satellite_packets"] == 2
            and initial_transport_stats["mesh_packets"] >= expected_beacons - 2
            and transport_stats["failover_active"]
        )

        set_executor_unit_state(
            selected_unit_id,
            executor_coords=(executor.lat, executor.lon),
            battery=executor.battery_pct,
            status="COMPLETE" if success else "FAILED",
        )
        set_executor_mission_status("COMPLETE" if success else "FAILED")

        reports = []
        while not status_queue.empty():
            reports.append(status_queue.get_nowait())
        arrivals = [report for report in reports if report.get("report_type") == "arrival"]
        push_cycle_summary({
            "cycle": 1,
            "mission_id": briefing.briefing_id,
            "timestamp": time.time(),
            "targets_visited": len(arrivals),
            "targets_total": len(briefing.waypoints),
            "completion_pct": round(100 * len(arrivals) / max(len(briefing.waypoints), 1)),
            "beacons_by_priority": [
                {"id": waypoint.beacon_id, "type": waypoint.event_type, "priority_pct": round(waypoint.priority * 100)}
                for waypoint in briefing.waypoints
            ],
            "beacons_skipped_ttl": 0,
            "drift_corrections": transformer.drift.corrections_applied(),
            "drift_total_adjustment_m": transformer.drift.total_adjustment_m(),
            "failures": [
                {
                    "type": "satellite_disrupted",
                    "label": "Satellite Link Disrupted",
                    "timestamp": time.time(),
                    "ts_str": "1S AFTER SECOND COMMAND POST ARRIVAL",
                    "detail": "Injected via /api/failure/satellite 1 second after PKT0002 reached Command Post",
                },
                {
                    "type": "automatic_rf_mesh_failover",
                    "label": "Automatic RF Mesh Failover",
                    "timestamp": time.time(),
                    "ts_str": "ONA ROLE 3",
                    "detail": "Satellite loss detected; subsequent completion beacon carried through RF mesh",
                },
            ],
        })

        proof = Table(title="Satellite Failure → RF Mesh Continuity Proof", header_style="bold cyan")
        proof.add_column("Check", style="bold white")
        proof.add_column("Observed result", style="cyan")
        proof.add_row(
            "Confirmed satellite arrivals",
            " / ".join(
                f"{arrival.get('beacon_id', 'unknown')} ({arrival.get('packet_id', 'unknown')}) → Command Post"
                for arrival in confirmed_arrivals
            ) or "not received",
        )
        proof.add_row(
            "Scheduled failure injection",
            "POST /api/failure/satellite 1 second after second Command Post arrival"
            if failure_injected.is_set() else failure_result.get("error", "not injected"),
        )
        proof.add_row(
            "Writer traffic after failure",
            f"Satellite packets: {initial_transport_stats['satellite_packets']} | mesh packets: {initial_transport_stats['mesh_packets']}",
        )
        proof.add_row("Automatic reroute", f"Total mesh packets: {transport_stats['mesh_packets']} | active={transport_stats['failover_active']}")
        proof.add_row("Mission continuity", f"Executor success={success}; completion beacon delivered={completion_delivered}")
        console.print(proof)
        logger.info("[FailoverDemo] Complete: success=%s completion=%s failover=%s", success, completion_delivered, failover_verified)
        return success and completion_delivered and failover_verified
    finally:
        if receiver is not None:
            receiver.stop()
        if live_map is not None:
            live_map.stop()


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the isolated satellite-to-RF-mesh failover demo")
    parser.add_argument("--port", type=int, default=5004, help="Dashboard port (default: 5004)")
    parser.add_argument("--nodes", type=int, default=DEFAULT_MESH_NODE_COUNT, help="RF relay node count (default: 6)")
    parser.add_argument("--seed", type=int, default=None, help="Optional reproducible RF node placement seed")
    parser.add_argument("--no-serve", action="store_true", help="Exit after the scenario instead of retaining the dashboard")
    return parser.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    completed = run_satellite_mesh_failover_demo(
        dashboard_port=args.port,
        node_count=args.nodes,
        seed=args.seed,
    )
    if completed and not args.no_serve:
        logger.info("Failover dashboard remains available at http://127.0.0.1:%s — press Ctrl+C to stop", args.port)
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
    sys.exit(0 if completed else 1)
