"""Standalone Writer-loss mission-continuity demonstration.

This scenario is intentionally separate from ``run_demo.py`` and
``run_failure_demo.py``.  It demonstrates one failure case only:

1. The Writer Robot discovers and deposits at least two different event types.
2. The Writer is catastrophically lost and produces no further updates.
3. The deposited beacons persist, traverse the normal ONA uplink, and reach
   the Command Post.
4. The Command Post prepares a briefing and authorizes one selected field unit
   through ONA Role 4.
5. That Executor completes the beacon-derived mission and deposits its normal
   ``mission_completed`` beacon through the same beacon pipeline.

Run from the project root:
    python simulation/run_writer_loss_continuity_demo.py
"""

from __future__ import annotations

import argparse
import logging
import queue
import sys
import threading
import time
from pathlib import Path
from typing import Callable


PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from beacon.job_compatibility import mission_compatibility_score
from beacon.store import BeaconStore
from command_post.dashboard import (
    app as flask_app,
    init_dashboard,
    register_failure_targets,
    set_awaiting_authorization,
    set_executor_mission_status,
    set_executor_unit_state,
    set_latest_briefing,
    set_rover_coords,
)
from command_post.map_state import LiveMap
from command_post.mission_generator import MissionGenerator
from executor_robot.navigator import ExecutorRobot
from executor_robot.roster import EXECUTOR_UNITS, ExecutorUnitDefinition
from outside_network.frame_transform import DriftModel, GPSAnchor, LocalToGPS
from outside_network.lora_uplink import RetryQueue, SatelliteUplink, SimulatedSatelliteDriver
from outside_network.mission_relay import MissionRelay
from outside_network.receiver import BeaconReceiver
from simulation.config import AnchorConfig, DriftConfig, GridConfig, LOGS_DIR, WriterConfig
from writer_robot.explorer import WriterRobot
from writer_robot.grid_world import GridWorld


logger = logging.getLogger("writer_loss_continuity_demo")
WAIT_TIMEOUT_S = 12.0


def _wait_until(condition: Callable[[], bool], description: str, timeout_s: float = WAIT_TIMEOUT_S) -> None:
    """Wait on real component state rather than a fixed arbitrary delay."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if condition():
            return
        time.sleep(0.05)
    raise RuntimeError(f"Timed out waiting for {description}")


def _best_unit(event_types: list[str]) -> ExecutorUnitDefinition:
    """Select the best matching available unit using the existing explicit mapping."""
    return max(
        EXECUTOR_UNITS,
        key=lambda unit: mission_compatibility_score(event_types, unit.job_type),
    )


def _configure_logging() -> Path:
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    log_path = LOGS_DIR / f"writer_loss_continuity_{timestamp}.log"
    logger.setLevel(logging.INFO)
    logger.handlers.clear()
    formatter = logging.Formatter("%(asctime)s | %(levelname)s | %(message)s")

    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)

    file_handler = logging.FileHandler(log_path, encoding="utf-8")
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)
    return log_path


def run_writer_loss_continuity_demo(dashboard_port: int = 5002) -> bool:
    """Run the isolated Writer-loss continuity scenario and return its outcome."""
    log_path = _configure_logging()
    beacon_receiver: BeaconReceiver | None = None
    live_map: LiveMap | None = None

    try:
        logger.info("WRITER-LOSS CONTINUITY DEMO STARTED")

        uplink_queue: queue.Queue = queue.Queue()
        downlink_queue: queue.Queue = queue.Queue()
        executor_inboxes = {unit.unit_id: queue.Queue() for unit in EXECUTOR_UNITS}
        status_queue: queue.Queue = queue.Queue()

        beacon_store = BeaconStore()
        grid_cfg = GridConfig()
        grid = GridWorld(width=grid_cfg.width, height=grid_cfg.height)

        anchor_cfg = AnchorConfig()
        drift_cfg = DriftConfig()
        transformer = LocalToGPS(
            anchor=GPSAnchor(
                origin_lat=anchor_cfg.origin_lat,
                origin_lon=anchor_cfg.origin_lon,
                origin_alt=anchor_cfg.origin_alt,
                bearing_deg=anchor_cfg.bearing_deg,
            ),
            drift_model=DriftModel(
                drift_rate_x=drift_cfg.drift_rate_x,
                drift_rate_y=drift_cfg.drift_rate_y,
                correction_interval=drift_cfg.correction_interval,
            ),
        )

        # This scenario isolates Writer loss; it intentionally injects no
        # satellite fault or packet loss.
        satellite_driver = SimulatedSatelliteDriver(
            uplink_queue=uplink_queue,
            downlink_queue=downlink_queue,
            packet_loss_prob=0.0,
            latency_ms=0,
        )
        retry_queue = RetryQueue(satellite_driver)
        satellite_uplink = SatelliteUplink(satellite_driver, retry_queue)
        beacon_receiver = BeaconReceiver(beacon_store, transformer, satellite_uplink)
        live_map = LiveMap(uplink_queue)
        mission_relay = MissionRelay(satellite_uplink, executor_inboxes)
        mission_generator = MissionGenerator(live_map, mission_relay)

        # This standalone scenario has its own dashboard process and port. It
        # therefore never replaces or competes with the main demo dashboard.
        init_dashboard(
            live_map,
            mission_generator,
            satellite_driver=satellite_driver,
            satellite_uplink=satellite_uplink,
            beacon_store=beacon_store,
            executor_units=[unit.to_dashboard_dict() for unit in EXECUTOR_UNITS],
        )
        threading.Thread(
            target=lambda: flask_app.run(
                host="127.0.0.1",
                port=dashboard_port,
                debug=False,
                use_reloader=False,
            ),
            name="WriterLossDashboard",
            daemon=True,
        ).start()
        logger.info("DASHBOARD STARTED | http://127.0.0.1:%s", dashboard_port)

        beacon_receiver.start()
        live_map.start()
        threading.Thread(target=retry_queue.drain, name="WriterLossRetryQueue", daemon=True).start()

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
        writer_thread = threading.Thread(
            target=writer.run,
            kwargs={"max_steps": 250},
            name="WriterLossWriter",
        )
        writer_thread.start()

        _wait_until(
            lambda: len(writer.beacons_dropped) >= 2
            and len({beacon.event.type for beacon in writer.beacons_dropped}) >= 2,
            "two Writer beacons with distinct event types",
        )
        pre_failure_beacons = list(writer.beacons_dropped)
        pre_failure_ids = {beacon.beacon_id for beacon in pre_failure_beacons}
        pre_failure_snapshot = (writer._step, len(writer.beacons_dropped))
        logger.info(
            "WRITER UPDATE | steps=%s | beacons=%s | event_types=%s",
            pre_failure_snapshot[0],
            pre_failure_snapshot[1],
            ", ".join(beacon.event.type for beacon in pre_failure_beacons),
        )

        writer.kill()
        writer_thread.join(timeout=2.0)
        if writer_thread.is_alive() or writer.is_alive():
            raise RuntimeError("Writer did not stop after catastrophic-loss injection")

        time.sleep(0.30)
        post_failure_snapshot = (writer._step, len(writer.beacons_dropped))
        if post_failure_snapshot != pre_failure_snapshot:
            raise RuntimeError("Writer produced a position or beacon update after failure")
        logger.info(
            "FAILURE INJECTED | Writer state=%s | no post-failure updates confirmed",
            writer.state.name,
        )

        active_store_ids = {beacon.beacon_id for beacon in beacon_store.get_active()}
        if not pre_failure_ids.issubset(active_store_ids):
            raise RuntimeError("A beacon deposited before Writer loss did not persist in BeaconStore")
        logger.info("BEACON STORE PRESERVED | surviving_beacons=%s", ", ".join(sorted(pre_failure_ids)))

        _wait_until(
            lambda: pre_failure_ids.issubset(
                {event.beacon_id for event in live_map.get_active_events()}
            ),
            "surviving beacons to reach the Command Post through ONA",
        )
        logger.info(
            "COMMAND POST RECEIVED | map_events=%s | ona_forwarded=%s",
            len(live_map.get_active_events()),
            beacon_receiver.beacons_forwarded,
        )

        briefing = mission_generator.generate()
        if briefing is None or not pre_failure_ids.issubset({wp.beacon_id for wp in briefing.waypoints}):
            raise RuntimeError("Command Post could not generate a mission from the surviving beacons")

        event_types = [waypoint.event_type for waypoint in briefing.waypoints]
        selected_unit = _best_unit(event_types)
        compatibility = mission_compatibility_score(event_types, selected_unit.job_type)
        logger.info(
            "MISSION PREPARED | briefing=%s | waypoints=%s | selected_unit=%s | compatibility=%s",
            briefing.briefing_id,
            len(briefing.waypoints),
            selected_unit.unit_id,
            compatibility,
        )

        executor = ExecutorRobot(
            robot_id=selected_unit.unit_id,
            executor_inbox=executor_inboxes[selected_unit.unit_id],
            status_queue=status_queue,
            beacon_store=beacon_store,
            start_lat=selected_unit.start_lat,
            start_lon=selected_unit.start_lon,
            battery_pct=selected_unit.initial_battery,
            step_callback=lambda **update: set_executor_unit_state(
                selected_unit.unit_id,
                executor_coords=update.get("executor_coords"),
                battery=update.get("executor_battery"),
            ),
        )

        # Prepare the same dashboard state an operator sees, then submit the
        # scripted authorization through the actual dashboard routes. This keeps
        # the only dispatch path CP -> ONA Role 4 -> selected Executor inbox.
        set_latest_briefing(briefing.to_dict())
        set_awaiting_authorization()
        with flask_app.test_client() as client:
            selection = client.post("/api/executor/select", json={"unit_id": selected_unit.unit_id})
            if selection.status_code != 200:
                raise RuntimeError(f"Dashboard could not select {selected_unit.unit_id}: {selection.get_json()}")
            authorization = client.post("/api/mission/authorize")
            if authorization.status_code != 200:
                raise RuntimeError(f"Dashboard could not authorize the mission: {authorization.get_json()}")
            relay_id = authorization.get_json()["relay_id"]
        logger.info("MISSION AUTHORIZED | relay=%s | target=%s", relay_id, selected_unit.unit_id)
        if not executor.await_and_run(timeout_s=5.0):
            raise RuntimeError("Selected Executor did not complete the surviving-beacon mission")

        _wait_until(
            lambda: any(
                event.writer_id == selected_unit.unit_id and event.event_type == "mission_completed"
                for event in live_map.get_active_events()
            ),
            "Executor completion beacon to reach Command Post through ONA",
        )

        arrival_reports = []
        while not status_queue.empty():
            report = status_queue.get_nowait()
            if report.get("report_type") == "arrival":
                arrival_reports.append(report)
        if len(arrival_reports) < 2:
            raise RuntimeError("Executor did not confirm both beacon-derived targets")

        logger.info(
            "EXECUTOR COMPLETE | unit=%s | targets=%s | completion_beacon=received",
            selected_unit.unit_id,
            len(arrival_reports),
        )
        set_executor_unit_state(
            selected_unit.unit_id,
            executor_coords=(executor.lat, executor.lon),
            battery=executor.battery_pct,
            status="COMPLETE",
        )
        set_executor_mission_status("COMPLETE")
        logger.info("RESULT | PASS | Writer knowledge survived loss and enabled mission continuity")
        logger.info("Evidence log: %s", log_path)
        return True
    except Exception as error:
        logger.exception("RESULT | FAIL | %s", error)
        return False
    finally:
        if beacon_receiver is not None:
            beacon_receiver.stop()
        if live_map is not None:
            live_map.stop()


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the isolated Writer-loss continuity demo")
    parser.add_argument("--port", type=int, default=5002, help="Dashboard port (default: 5002)")
    parser.add_argument(
        "--no-serve",
        action="store_true",
        help="Exit when the demonstration finishes instead of keeping its dashboard open",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    success = run_writer_loss_continuity_demo(dashboard_port=args.port)
    if success and not args.no_serve:
        logger.info("Dashboard remains available at http://127.0.0.1:%s — press Ctrl+C to stop", args.port)
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
    sys.exit(0 if success else 1)
