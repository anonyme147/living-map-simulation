"""Standalone RF-mesh relay demonstration for ONA Role 3 CARRY.

This scenario is intentionally separate from the normal, resilience, and
Writer-loss demonstrations. It leaves their single-hop satellite CARRY code
unchanged and substitutes the mesh transport only within this process.
"""

from __future__ import annotations

import argparse
import logging
import queue
import sys
import threading
import time
from pathlib import Path
from typing import Optional

from rich.console import Console
from rich.logging import RichHandler
from rich.panel import Panel
from rich.table import Table


PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

# Existing pipeline logs include operational symbols such as drift deltas and
# arrows. Match the normal demo's Windows-safe UTF-8 terminal configuration.
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
from outside_network.lora_uplink import RetryQueue
from outside_network.receiver import BeaconReceiver
from outside_network.rf_mesh import (
    DEFAULT_MESH_NODE_COUNT,
    MeshMissionRelay,
    MeshRelayDriver,
    MeshSatelliteUplink,
    RFMeshTopology,
)
from simulation.config import (
    AnchorConfig,
    DashboardConfig,
    DriftConfig,
    GridConfig,
    WriterConfig,
)
from writer_robot.explorer import WriterRobot
from writer_robot.grid_world import GridWorld


console = Console(legacy_windows=False)
logger = logging.getLogger("rf_mesh_demo")
_flask_started = False


def _configure_logging() -> None:
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.INFO)
    # Browser polling is dashboard plumbing, not RF evidence. Keep it out of
    # the recording terminal so the active hop, failure, and reroute messages
    # remain readable like the normal mission terminal.
    logging.getLogger("werkzeug").setLevel(logging.ERROR)
    if any(getattr(handler, "name", None) == "rf-mesh-console" for handler in root_logger.handlers):
        return
    handler = RichHandler(console=console, show_time=True, show_level=True, show_path=False, markup=False)
    handler.name = "rf-mesh-console"
    handler.setFormatter(logging.Formatter("%(message)s"))
    root_logger.addHandler(handler)


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


def _render_mesh_table(topology: RFMeshTopology) -> None:
    snapshot = topology.snapshot()
    table = Table(title="ONA Role 3 — RF Mesh Topology", header_style="bold cyan")
    table.add_column("Node", style="bold white")
    table.add_column("Role", style="magenta")
    table.add_column("Latitude", style="cyan")
    table.add_column("Longitude", style="cyan")
    for node in snapshot["nodes"]:
        table.add_row(
            node["node_id"],
            "CP Gateway" if node["gateway"] else "RF Relay",
            f"{node['lat']:.6f}",
            f"{node['lon']:.6f}",
        )
    console.print(table)
    console.print(f"[dim]Static mesh links: {len(snapshot['links'])} | Routing: distance-weighted Dijkstra[/dim]")


def _wait_for_completion_beacon(
    beacon_store: BeaconStore,
    receiver: BeaconReceiver,
    live_map: LiveMap,
    executor_id: str,
    timeout_s: float = 20.0,
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
        "RF mesh completion-beacon delivery=%s beacon=%s forwarded=%s",
        result,
        completion_id or "not-created",
        receiver.beacons_forwarded,
    )
    return result


def run_rf_mesh_demo(
    *,
    dashboard_port: int = 5003,
    node_count: int = DEFAULT_MESH_NODE_COUNT,
    seed: Optional[int] = None,
    resilient_node_failure: bool = False,
    dual_node_failure: bool = False,
) -> bool:
    """Run one isolated mesh-transport mission cycle."""
    global _flask_started
    _configure_logging()
    receiver: Optional[BeaconReceiver] = None
    live_map: Optional[LiveMap] = None
    try:
        console.print(Panel.fit(
            "[bold cyan]RF MESH DUAL-NODE FAILURE[/bold cyan]\n"
            "Two relays fail simultaneously; the in-flight packet reroutes from its current node."
            if dual_node_failure else
            "A live relay failure reroutes an in-flight packet from its current node."
            if resilient_node_failure else
            "[bold cyan]RF MESH RELAY DEMO[/bold cyan]\n"
            "ONA Role 3 carries every beacon hop-by-hop through the RF mesh.",
            border_style="cyan",
        ))

        uplink_queue: queue.Queue = queue.Queue()
        downlink_queue: queue.Queue = queue.Queue()
        executor_inboxes: dict[str, queue.Queue] = {unit.unit_id: queue.Queue() for unit in EXECUTOR_UNITS}
        status_queue: queue.Queue = queue.Queue()
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
        topology = RFMeshTopology.generate(
            building_lat=dashboard_config.building_site_lat,
            building_lon=dashboard_config.building_site_lon,
            command_post_lat=dashboard_config.command_post_lat,
            command_post_lon=dashboard_config.command_post_lon,
            node_count=node_count,
            seed=seed,
            resilient=resilient_node_failure,
            resilience_profile="dual" if dual_node_failure else "none",
        )
        if dual_node_failure:
            # Refuse to start a misleading demo if the exact simultaneous
            # outage cannot reach Command Post from the current relay.
            topology.shortest_path(
                source_id="RF-02",
                excluded_node_ids={"RF-03", "RF-05"},
            )
        mesh_driver = MeshRelayDriver(
            uplink_queue,
            downlink_queue,
            topology,
            failure_after_hop=1 if (resilient_node_failure or dual_node_failure) else None,
            failure_node_id="RF-03" if resilient_node_failure else None,
            failure_node_ids=("RF-03", "RF-05") if dual_node_failure else None,
            # The dual-node demo proves a Writer beacon rerouting in flight.
            # Its later ONA Role 4 briefing still uses the mesh, but the reverse
            # gateway-to-entry animation is hidden so it cannot be read as an
            # extra ONA-to-Writer beacon transmission.
            show_downlink_activity=not dual_node_failure,
        )
        retry_queue = RetryQueue(mesh_driver)
        mesh_uplink = MeshSatelliteUplink(
            mesh_driver,
            retry_queue,
            visual_mode=("mesh_dual_node_failure" if dual_node_failure else "mesh_node_failure" if resilient_node_failure else "mesh"),
            # RF mesh scenarios show the relay topology and active RF hops
            # only; do not draw the separate blue direct-CARRY overlay.
            show_initial_blue_link=False,
        )
        receiver = BeaconReceiver(beacon_store, transformer, mesh_uplink)
        live_map = LiveMap(uplink_queue)
        mission_relay = MeshMissionRelay(mesh_uplink, executor_inboxes, mesh_driver)
        mission_generator = MissionGenerator(live_map, mission_relay)

        init_dashboard(
            live_map,
            mission_generator,
            satellite_driver=mesh_driver,
            satellite_uplink=mesh_uplink,
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
                name="RFMeshDashboard",
                daemon=True,
            ).start()
            _flask_started = True

        receiver.start()
        live_map.start()
        threading.Thread(target=retry_queue.drain, name="RFMeshRetryQueue", daemon=True).start()
        _render_mesh_table(topology)
        logger.info(
            "RF mesh pipeline ready; nodes=%s seed=%s dashboard=http://%s:%s",
            node_count,
            seed if seed is not None else "random",
            dashboard_config.host,
            dashboard_config.port,
        )

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
        register_failure_targets(writer=writer, satellite_driver=mesh_driver, beacon_store=beacon_store)

        while writer.is_alive() and len(writer.beacons_dropped) < 3 and writer._step < 220:
            frontier = writer._nearest_frontier(writer.grid.get_frontiers())
            if frontier is None or not writer._step_toward(frontier):
                break
            writer._step += 1
            writer.battery_pct = max(0.0, writer.battery_pct - 0.2)
            writer._scan_and_maybe_drop()
            time.sleep(writer_config.step_delay_s)

        expected_beacons = len(writer.beacons_dropped)
        if expected_beacons < 2:
            logger.error("Writer produced only %s beacon(s); cannot demonstrate a mesh route", expected_beacons)
            return False
        if not _wait_until(lambda: len(live_map.get_active_events()) >= expected_beacons, 30.0):
            logger.error("Mesh CARRY timed out before all beacons reached Command Post")
            return False

        events = live_map.get_active_events()
        event_table = Table(title="Command Post Event Feed — Via RF Mesh", header_style="bold cyan")
        event_table.add_column("Beacon", style="bold white")
        event_table.add_column("Event", style="cyan")
        event_table.add_column("Severity", style="yellow")
        event_table.add_column("Route", style="magenta")
        path_text = " → ".join(node.node_id for node in topology.shortest_path())
        for event in events:
            event_table.add_row(event.beacon_id, event.event_type, f"{event.severity:.2f}", path_text)
        console.print(event_table)

        briefing = mission_generator.generate()
        if briefing is None:
            logger.error("Mesh demo cannot generate a briefing from Command Post events")
            return False
        set_latest_briefing(briefing.to_dict())
        set_awaiting_authorization()
        logger.info("Mission %s prepared; select a unit and authorize from dashboard", briefing.briefing_id)

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
            logger.error("Mesh demo authorization timed out")
            return False
        selected_unit_id = get_selected_executor_unit_id()
        if selected_unit_id not in executors:
            logger.error("No valid Executor unit was selected for mesh demo")
            return False

        executor = executors[selected_unit_id]
        logger.info("ONA Role 4 dispatch started for selected unit %s", selected_unit_id)
        success = executor.await_and_run(timeout_s=8.0)
        completion_delivered = _wait_for_completion_beacon(
            beacon_store, receiver, live_map, executor.robot_id
        ) if success else False
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
            "failures": ([
                {
                    "type": "rf_mesh_dual_node_failure",
                    "label": "Two RF Mesh Relays Offline",
                    "timestamp": time.time(),
                    "ts_str": "SIMULTANEOUS MID-RELAY",
                    "detail": "RF-03 and RF-05 failed together after hop 1; the in-flight packet rerouted around both backup branches.",
                },
            ] if dual_node_failure else [
                {
                    "type": "rf_mesh_node_failed",
                    "label": "RF Mesh Relay Node Offline",
                    "timestamp": time.time(),
                    "ts_str": "MID-RELAY",
                    "detail": "RF-03 failed after hop 1; the in-flight packet rerouted from RF-02 through the backup relay.",
                },
            ] if resilient_node_failure else []),
        })
        logger.info(
            "RF mesh demo complete; selected=%s visited=%s/%s completion_beacon=%s",
            selected_unit_id,
            len(arrivals),
            len(briefing.waypoints),
            completion_delivered,
        )
        return success and completion_delivered
    finally:
        if receiver is not None:
            receiver.stop()
        if live_map is not None:
            live_map.stop()


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the isolated ONA RF mesh relay demo")
    parser.add_argument("--port", type=int, default=5003, help="Dashboard port (default: 5003)")
    parser.add_argument("--nodes", type=int, default=DEFAULT_MESH_NODE_COUNT, help="RF relay node count (default: 6)")
    parser.add_argument("--seed", type=int, default=None, help="Optional placement seed for reproducible topology")
    parser.add_argument("--no-serve", action="store_true", help="Exit after the scenario rather than keeping dashboard open")
    return parser.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    completed = run_rf_mesh_demo(dashboard_port=args.port, node_count=args.nodes, seed=args.seed)
    if completed and not args.no_serve:
        logger.info("RF mesh dashboard remains available at http://127.0.0.1:%s — press Ctrl+C to stop", args.port)
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
    sys.exit(0 if completed else 1)
