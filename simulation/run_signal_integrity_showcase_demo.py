"""Flagship continuous proof that ONA preserves beacon payload integrity."""

from __future__ import annotations

import argparse
import json
import logging
import queue
import sys
import threading
import time
from pathlib import Path
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
import executor_robot.navigator as navigator_module
from executor_robot.roster import EXECUTOR_UNITS
from outside_network.frame_transform import DriftModel, GPSAnchor, LocalToGPS
from outside_network.heartbeat import HeartbeatMonitor
from outside_network.lora_uplink import RetryQueue, SimulatedSatelliteDriver
from outside_network.mission_relay import MissionRelay
from outside_network.receiver import BeaconReceiver
from outside_network.rf_mesh import MeshRelayDriver, RFMeshTopology
from outside_network.satellite_mesh_failover import FailoverSatelliteUplink, SatelliteMeshFailoverDriver
from simulation.config import AnchorConfig, DashboardConfig, DriftConfig, GridConfig, WriterConfig
from simulation.signal_integrity import SignalIntegrityLedger
from writer_robot.explorer import WriterRobot
from writer_robot.grid_world import GridWorld


console = Console(legacy_windows=False)
logger = logging.getLogger("signal_integrity_showcase")

SUBTITLES = {
    "opening": "The Writer Robot is depositing emergency beacons inside the disaster zone. ONA carries each message to Command Post, where its exact payload is checked for loss or corruption.",
    "first_satellite": "The first beacon has reached Command Post through the satellite link. Its payload is intact, establishing the baseline for the resilience tests ahead.",
    "two_satellite": "Two beacon packets have reached Command Post through the satellite link. This establishes a healthy communication baseline before resilience tests begin.",
    "satellite_failed": "The satellite link has failed. ONA automatically switches new traffic to the RF mesh, preserving the same beacon data without creating a direct robot-to-command link.",
    "mesh_active": "The RF relay mesh is now carrying mission traffic hop by hop. Communication continues even though the primary satellite path is unavailable.",
    "heartbeat_started": "The mesh is carrying beacon data normally. This phase tests link trust separately: heartbeat messages confirm that ONA and Command Post can still verify each other.",
    "heartbeat_degraded": "Heartbeat signals have stopped arriving, so the link is marked unconfirmed. Beacon traffic is not discarded: the system distinguishes missing trust signals from actual data loss.",
    "data_while_degraded": "A preserved beacon packet has arrived while heartbeats are absent. This proves the data channel is still functioning, even though operators are warned that link confirmation is missing.",
    "heartbeat_recovered": "Heartbeat confirmation has returned. The link is healthy again, and the earlier beacon payloads remained intact throughout the uncertainty.",
    "relay_armed": "The RF mesh now faces an internal relay failure. If a relay becomes unavailable during transmission, the packet must continue from its current position rather than restart from the Writer Robot.",
    "relay_failed": "Relay RF-03 has failed during an active transmission. The mesh is recalculating a route from the packet's current relay instead of discarding the beacon.",
    "rerouted": "A new shortest route is active around the failed relay. The packet continues through live RF nodes and reaches Command Post without being lost or altered.",
    "mission_ready": "All surviving beacon data is now available at Command Post. Select the most suitable response unit and authorize delivery through ONA Role 4.",
    "authorized": "The selected response unit has been authorized through ONA Role 4. It is navigating from preserved beacon knowledge, not from a direct Command Post link.",
    "complete": "The selected Executor completed the assigned response mission. Its completion beacon returned through the surviving communication path, confirming end-to-end mission continuity.",
}

# Showcase-only pacing.  These values preserve the real event order while
# keeping the recording concise: satellite baseline, mesh failover, heartbeat
# degradation/recovery, relay reroute, then operator authorization.
# They never alter shared physics, routing decisions, or other demo scenarios.
SHOWCASE_WRITER_STEP_DELAY_S = 0.16
SHOWCASE_SATELLITE_LATENCY_MS = 900
SHOWCASE_MESH_HOP_DURATION_S = 0.42
SHOWCASE_HEARTBEAT_INTERVAL_S = 0.45
SHOWCASE_HEARTBEAT_THRESHOLD_S = 2.0
SHOWCASE_HEARTBEAT_RECOVERY_S = 0.90
SHOWCASE_EXECUTOR_NAV_STEP_DELAY_S = 0.16


class _DelayedHeartbeatSource:
    """Keeps the heartbeat panel hidden until the mesh heartbeat phase begins."""

    def __init__(self, monitor: HeartbeatMonitor) -> None:
        self.monitor = monitor
        self.enabled = False

    def status(self) -> dict:
        return self.monitor.status() if self.enabled else {"enabled": False}


class _IntegrityFailoverUplink(FailoverSatelliteUplink):
    def __init__(self, driver, retry_queue, ledger: SignalIntegrityLedger) -> None:
        super().__init__(driver, retry_queue)
        self._ledger = ledger

    def send_beacon(self, enriched_beacon: dict) -> str:
        self._ledger.record_sent(enriched_beacon)
        return super().send_beacon(enriched_beacon)


class _IntegrityLiveMap(LiveMap):
    def __init__(self, uplink_queue: queue.Queue, ledger: SignalIntegrityLedger, **kwargs) -> None:
        super().__init__(uplink_queue, **kwargs)
        self._ledger = ledger

    def _ingest(self, raw_bytes_or_str) -> None:
        self._ledger.record_delivered_packet(raw_bytes_or_str)
        super()._ingest(raw_bytes_or_str)


def _wait_until(predicate, timeout_s: float, interval_s: float = 0.05) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval_s)
    return bool(predicate())


def _trigger_satellite_failure(port: int) -> None:
    request = Request(
        f"http://127.0.0.1:{port}/api/failure/satellite",
        data=b"{}",
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urlopen(request, timeout=5) as response:
        if response.status != 200:
            raise RuntimeError(f"Satellite failure endpoint returned HTTP {response.status}")


def _advance_to_next_beacon(writer: WriterRobot, config: WriterConfig) -> bool:
    """Advance Writer only until one new physical beacon exists."""
    prior_count = len(writer.beacons_dropped)
    while writer.is_alive() and writer._step < 250:
        frontier = writer._nearest_frontier(writer.grid.get_frontiers())
        if frontier is None or not writer._step_toward(frontier):
            return False
        writer._step += 1
        writer.battery_pct = max(0.0, writer.battery_pct - 0.2)
        writer._scan_and_maybe_drop()
        if len(writer.beacons_dropped) > prior_count:
            return True
        time.sleep(config.step_delay_s)
    return False


def _replay_preserved_beacon(receiver: BeaconReceiver, beacon) -> None:
    """Re-carry a preserved packet through the same ONA receive/translate/carry path.

    The fixed Writer scenario provides three independently confirmed discovery
    beacons.  The later Showcase phases test communication resilience, not new
    discovery, so they deliberately replay a retained beacon instead of waiting
    forever for a fourth or fifth physical event.  ``BeaconReceiver._process``
    is the existing ONA Role 1 -> Role 2 -> Role 3 path; this helper introduces
    no Command Post shortcut or new communication channel.
    """
    logger.info(
        "[Showcase] Replaying preserved beacon %s through ONA for the next communication phase",
        beacon.beacon_id,
    )
    receiver._process(beacon)


def _configure_logging() -> None:
    root = logging.getLogger()
    root.setLevel(logging.INFO)
    logging.getLogger("werkzeug").setLevel(logging.ERROR)
    if any(getattr(item, "name", None) == "signal-integrity-showcase-console" for item in root.handlers):
        return
    handler = RichHandler(console=console, show_time=True, show_level=True, show_path=False, markup=False)
    handler.name = "signal-integrity-showcase-console"
    handler.setLevel(logging.INFO)
    handler.setFormatter(logging.Formatter("%(message)s"))
    root.addHandler(handler)


def run_signal_integrity_showcase_demo(*, dashboard_port: int = 5010) -> bool:
    """Run all three resilience proofs inside one continuous ONA mission."""
    _configure_logging()
    receiver = None
    live_map = None
    heartbeat = None
    try:
        console.print(Panel.fit(
            "[bold cyan]SIGNAL INTEGRITY SHOWCASE[/bold cyan]\n"
            "One continuous mission: satellite failover → zombie link → live mesh reroute.\n"
            "[bold green]Target proof: zero lost payloads · zero corrupted payloads[/bold green]",
            border_style="cyan",
        ))
        dashboard = DashboardConfig(port=dashboard_port)
        ledger = SignalIntegrityLedger()
        ledger.set_subtitle(SUBTITLES["opening"])
        uplink_queue: queue.Queue = queue.Queue()
        downlink_queue: queue.Queue = queue.Queue()
        status_queue: queue.Queue = queue.Queue()
        inboxes = {unit.unit_id: queue.Queue() for unit in EXECUTOR_UNITS}
        store = BeaconStore()
        anchor, drift = AnchorConfig(), DriftConfig()
        transformer = LocalToGPS(
            anchor=GPSAnchor(anchor.origin_lat, anchor.origin_lon, anchor.origin_alt, anchor.bearing_deg),
            drift_model=DriftModel(drift.drift_rate_x, drift.drift_rate_y, drift.correction_interval),
        )
        grid_cfg, writer_cfg = GridConfig(), WriterConfig()
        writer_cfg.step_delay_s = SHOWCASE_WRITER_STEP_DELAY_S
        grid = GridWorld(width=grid_cfg.width, height=grid_cfg.height)
        satellite = SimulatedSatelliteDriver(
            uplink_queue, downlink_queue, packet_loss_prob=0.0, latency_ms=SHOWCASE_SATELLITE_LATENCY_MS,
        )
        topology = RFMeshTopology.generate(
            building_lat=dashboard.building_site_lat,
            building_lon=dashboard.building_site_lon,
            command_post_lat=dashboard.command_post_lat,
            command_post_lon=dashboard.command_post_lon,
            node_count=6,
            seed=42,
            resilient=True,
        )
        mesh = MeshRelayDriver(
            uplink_queue,
            downlink_queue,
            topology,
            hop_duration_s=SHOWCASE_MESH_HOP_DURATION_S,
            on_node_failure=lambda **_: ledger.set_subtitle(SUBTITLES["relay_failed"]),
            on_reroute=lambda **_: ledger.set_subtitle(SUBTITLES["rerouted"]),
        )
        second_arrival = threading.Event()
        driver = SatelliteMeshFailoverDriver(satellite, mesh, second_command_post_arrival=second_arrival)
        retry_queue = RetryQueue(driver)
        uplink = _IntegrityFailoverUplink(driver, retry_queue, ledger)
        heartbeat = HeartbeatMonitor(
            interval_s=SHOWCASE_HEARTBEAT_INTERVAL_S,
            threshold_s=SHOWCASE_HEARTBEAT_THRESHOLD_S,
            normal_after_first_data_s=1.0,
            recovery_after_degraded_data_s=SHOWCASE_HEARTBEAT_RECOVERY_S,
            maximum_loss_s=8.0,
            on_degraded=lambda: ledger.set_subtitle(SUBTITLES["heartbeat_degraded"]),
            on_recovered=lambda: ledger.set_subtitle(SUBTITLES["heartbeat_recovered"]),
        )
        heartbeat_source = _DelayedHeartbeatSource(heartbeat)
        failure_injected = threading.Event()
        command_post_arrivals: list[str] = []

        def on_command_post_event(event) -> None:
            command_post_arrivals.append(event.beacon_id)
            if heartbeat_source.enabled:
                heartbeat.note_command_post_beacon(event)
                if heartbeat.status()["state"] == "degraded_unconfirmed":
                    ledger.set_subtitle(SUBTITLES["data_while_degraded"])
            if event.event_type == "mission_completed":
                ledger.set_subtitle(SUBTITLES["complete"])
            elif len(command_post_arrivals) == 1:
                ledger.set_subtitle(SUBTITLES["first_satellite"])
            elif len(command_post_arrivals) == 3 and failure_injected.is_set():
                ledger.set_subtitle(SUBTITLES["mesh_active"])
            if len(command_post_arrivals) == 2 and not failure_injected.is_set():
                logger.warning("[Showcase] Phase 1 complete: two satellite beacons reached Command Post")
                ledger.set_subtitle(SUBTITLES["two_satellite"])
                time.sleep(1.0)
                _trigger_satellite_failure(dashboard_port)
                ledger.set_phase("SATELLITE FAILED — RF MESH ACTIVE")
                ledger.set_subtitle(SUBTITLES["satellite_failed"])
                failure_injected.set()
                second_arrival.set()
                logger.warning("[Showcase] Satellite failure detected; ONA Role 3 automatically switched to RF mesh")

        live_map = _IntegrityLiveMap(uplink_queue, ledger, on_event_ingested=on_command_post_event)
        relay = MissionRelay(uplink, inboxes)
        mission_generator = MissionGenerator(live_map, relay)
        init_dashboard(
            live_map,
            mission_generator,
            satellite_driver=driver,
            satellite_uplink=uplink,
            beacon_store=store,
            executor_units=[unit.to_dashboard_dict() for unit in EXECUTOR_UNITS],
            heartbeat_monitor=heartbeat_source,
            integrity_ledger=ledger,
        )
        threading.Thread(
            target=lambda: flask_app.run(host=dashboard.host, port=dashboard.port, debug=False, use_reloader=False),
            name="SignalIntegrityDashboard",
            daemon=True,
        ).start()
        receiver = BeaconReceiver(store, transformer, uplink)
        receiver.start()
        live_map.start()
        threading.Thread(target=retry_queue.drain, name="SignalIntegrityRetryQueue", daemon=True).start()

        writer = WriterRobot(
            robot_id=writer_cfg.robot_id,
            grid=grid,
            beacon_store=store,
            start_row=writer_cfg.start_row,
            start_col=writer_cfg.start_col,
            battery_pct=writer_cfg.battery_pct,
            step_callback=lambda robot: set_rover_coords(writer_battery=robot.battery_pct),
            rng_seed=42,
        )
        register_failure_targets(writer=writer, satellite_driver=driver, beacon_store=store)

        console.rule("[bold cyan]PHASE 1 · SATELLITE FAILURE → RF MESH FAILOVER[/bold cyan]")
        ledger.set_phase("SATELLITE PRIMARY — 2 CONFIRMED BEACONS")
        for expected in (1, 2):
            if not _advance_to_next_beacon(writer, writer_cfg):
                return False
            if not _wait_until(lambda: len(live_map.get_active_events()) >= expected, 15.0):
                return False
        if not _wait_until(failure_injected.is_set, 8.0):
            return False

        console.rule("[bold yellow]PHASE 2 · ZOMBIE LINK: HEARTBEAT LOSS, DATA CONTINUES[/bold yellow]")
        ledger.set_phase("RF MESH — HEARTBEAT TRUST TEST")
        ledger.set_subtitle(SUBTITLES["heartbeat_started"])
        heartbeat_source.enabled = True
        heartbeat.start()
        if not _advance_to_next_beacon(writer, writer_cfg) or not _wait_until(lambda: len(live_map.get_active_events()) >= 3, 12.0):
            return False
        # Let the actual gap cross the configured threshold before another
        # real mesh-carried beacon proves data continues while unconfirmed.
        if not _wait_until(lambda: heartbeat.status()["state"] == "degraded_unconfirmed", 8.0):
            return False
        # The fixed Writer scene contains three real discoveries.  Re-carry its
        # last preserved beacon to prove live data delivery during the heartbeat
        # outage, rather than waiting forever for a nonexistent fourth event.
        _replay_preserved_beacon(receiver, writer.beacons_dropped[-1])
        if not _wait_until(lambda: len(command_post_arrivals) >= 4, 8.0):
            return False
        if not _wait_until(lambda: heartbeat.status()["state"] == "healthy", 8.0):
            return False

        console.rule("[bold magenta]PHASE 3 · LIVE RF RELAY FAILURE AND REROUTE[/bold magenta]")
        ledger.set_phase("RF MESH — RELAY FAILURE / LIVE REROUTE")
        ledger.set_subtitle(SUBTITLES["relay_armed"])
        mesh.arm_next_uplink_node_failure(failure_after_hop=1, failure_node_ids=("RF-03",))
        # Replay a retained packet for the relay-failure transport test.  The
        # packet still travels through ONA and the live mesh; only discovery is
        # not repeated.
        _replay_preserved_beacon(receiver, writer.beacons_dropped[0])
        if not _wait_until(lambda: len(command_post_arrivals) >= 5, 10.0):
            return False
        if not any(node["node_id"] == "RF-03" and node["state"] == "failed" for node in topology.snapshot()["nodes"]):
            return False

        console.rule("[bold green]PHASE 4 · ONA-ONLY MISSION CONTINUITY[/bold green]")
        ledger.set_phase("MISSION CONTINUITY — SURVIVING RF MESH")
        briefing = mission_generator.generate()
        if briefing is None:
            return False
        set_latest_briefing(briefing.to_dict())
        set_awaiting_authorization()
        ledger.set_subtitle(SUBTITLES["mission_ready"])
        logger.info("[Showcase] Select a response unit and authorize mission %s from the dashboard", briefing.briefing_id)
        if not wait_for_authorization(timeout_s=300.0):
            return False
        selected_unit_id = get_selected_executor_unit_id()
        unit = next((candidate for candidate in EXECUTOR_UNITS if candidate.unit_id == selected_unit_id), None)
        if unit is None:
            return False
        ledger.set_subtitle(SUBTITLES["authorized"])
        executor = ExecutorRobot(
            robot_id=unit.unit_id,
            executor_inbox=inboxes[unit.unit_id],
            status_queue=status_queue,
            beacon_store=store,
            start_lat=unit.start_lat,
            start_lon=unit.start_lon,
            battery_pct=unit.initial_battery,
            step_callback=lambda **values: set_executor_unit_state(
                unit.unit_id,
                executor_coords=values.get("executor_coords"),
                battery=values.get("executor_battery"),
            ),
        )
        set_executor_unit_state(unit.unit_id, status="EN_ROUTE")
        original_nav_step_delay = navigator_module.NAV_STEP_DELAY
        navigator_module.NAV_STEP_DELAY = SHOWCASE_EXECUTOR_NAV_STEP_DELAY_S
        try:
            navigation_success = executor.await_and_run(timeout_s=12.0)
        finally:
            navigator_module.NAV_STEP_DELAY = original_nav_step_delay
        completion_ok = _wait_until(
            lambda: any(
                event.writer_id == executor.robot_id and event.event_type == "mission_completed"
                for event in live_map.get_active_events()
            ),
            20.0,
        ) if navigation_success else False
        set_executor_unit_state(unit.unit_id, status="COMPLETE" if navigation_success else "FAILED")
        set_executor_mission_status("COMPLETE" if navigation_success else "FAILED")
        ledger.set_phase("MISSION COMPLETE — SIGNAL INTEGRITY VERIFIED")
        integrity = ledger.snapshot()
        success = navigation_success and completion_ok and integrity["lost"] == 0 and integrity["corrupted"] == 0

        proof = Table(title="Signal Integrity Showcase — End-to-End Proof", header_style="bold green")
        proof.add_column("Metric", style="bold white")
        proof.add_column("Observed value", style="cyan")
        for key, label in (("sent", "Payloads sent"), ("delivered", "Payloads delivered"), ("lost", "Payloads lost"), ("corrupted", "Payloads corrupted")):
            proof.add_row(label, str(integrity[key]))
        proof.add_row("Heartbeat phase", heartbeat.status()["state"])
        proof.add_row("RF-03", "FAILED — reroute completed")
        proof.add_row("Completion beacon", "DELIVERED" if completion_ok else "MISSING")
        console.print(proof)
        push_cycle_summary({
            "cycle": 1,
            "mission_id": briefing.briefing_id,
            "timestamp": time.time(),
            "targets_visited": len([item for item in list(status_queue.queue) if item.get("report_type") == "arrival"]),
            "targets_total": len(briefing.waypoints),
            "completion_pct": 100 if navigation_success else 0,
            "beacons_by_priority": [],
            "beacons_skipped_ttl": 0,
            "drift_corrections": transformer.drift.corrections_applied(),
            "drift_total_adjustment_m": transformer.drift.total_adjustment_m(),
            "failures": [
                {"type": "satellite_disrupted", "label": "Satellite Failure → RF Mesh Failover", "timestamp": time.time(), "ts_str": "PHASE 1", "detail": "Two satellite deliveries confirmed, then ONA automatically switched to RF mesh."},
                {"type": "heartbeat_loss", "label": "Zombie Link — Heartbeat Loss", "timestamp": time.time(), "ts_str": "PHASE 2", "detail": "Heartbeat trust degraded while beacon CARRY continued, then recovered."},
                {"type": "mesh_node_failure", "label": "RF Relay Node Failure", "timestamp": time.time(), "ts_str": "PHASE 3", "detail": "RF-03 failed mid-relay; the packet rerouted from RF-02."},
            ],
        })
        return success
    finally:
        if heartbeat is not None:
            heartbeat.stop()
        if receiver is not None:
            receiver.stop()
        if live_map is not None:
            live_map.stop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run the continuous Signal Integrity Showcase")
    parser.add_argument("--port", type=int, default=5010)
    args = parser.parse_args()
    raise SystemExit(0 if run_signal_integrity_showcase_demo(dashboard_port=args.port) else 1)
