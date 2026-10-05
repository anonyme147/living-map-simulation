"""
simulation/run_failure_demo.py
Failure-Injection & Resilience Demonstration for "The Living Map".
Addresses the failure-cases demonstration, Goal G1, Goal G6, and NF-3:

  1. Architectural Constraint Check (AST verification).
  2. Failure Case A: Catastrophic Writer Robot Loss
     - Writer robot explores, drops 2-3 beacons, then suffers catastrophic physical damage / power loss.
     - Writer robot is killed (state = KILLED). Exploration halts prematurely.
  3. Failure Case B: Satellite Link Disruption (Store-and-Forward Buffering)
     - Satellite uplink encounters a 100% weather/pass-handoff outage.
     - Store-and-Forward RetryQueue buffers packets, attempting retransmission with exponential backoff.
     - Channel link is restored; buffered packets drain without loss to Command Post.
  4. Mission Continuity & Second Responder Handoff
     - Command Post compiles partial surviving telemetry from in-situ beacons.
     - Generates mission briefing and dispatches through Outside Network Area only.
     - Executor Robot receives briefing, enters the zone, and reaches targets.
"""

from __future__ import annotations

import datetime
import logging
import os
import queue
import sys
import threading
import time
from pathlib import Path
from typing import List, Tuple

# Reconfigure stdout/stderr for UTF-8 on Windows
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

from rich.console import Console
from rich.panel import Panel
from rich.table import Table
from rich.text import Text
from rich.progress import Progress, SpinnerColumn, BarColumn, TextColumn
from rich.rule import Rule

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from beacon.store import BeaconStore
from command_post.dashboard import app as flask_app, init_dashboard, set_latest_briefing, set_awaiting_authorization, wait_for_authorization, register_failure_targets, push_cycle_summary, get_selected_executor_unit_id
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
from simulation.run_demo import _wait_for_completion_beacon, verify_architectural_isolation
from writer_robot.explorer import WriterRobot, RobotState
from writer_robot.grid_world import GridWorld


console = Console(legacy_windows=False)


def _wait_for_command_post_events(live_map: LiveMap, expected_count: int, timeout_s: float = 10.0) -> bool:
    """Wait for the actual ONA → CARRY → Command Post deliveries, not a fixed delay."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if len(live_map.get_active_events()) >= expected_count:
            return True
        time.sleep(0.05)
    return False


def run_failure_demo(dashboard_port: int = 5001) -> bool:
    timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file_path = LOGS_DIR / f"failure_demo_{timestamp_str}.log"

    # Setup file logging
    file_handler = logging.FileHandler(log_file_path, encoding="utf-8")
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(logging.Formatter("[%(asctime)s] [%(levelname)s] [%(name)s] %(message)s"))
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.INFO)
    root_logger.addHandler(file_handler)
    logging.getLogger("werkzeug").setLevel(logging.ERROR)

    # Clear any previous cycle summary so the dashboard starts fresh
    push_cycle_summary({})

    console.print()
    console.print(Panel.fit(
        "[bold red]THE LIVING MAP -- FAILURE INJECTION & RESILIENCE DEMO[/bold red]\n"
        "[dim]Anonymized resilience simulation[/dim]\n"
        "[yellow]Demonstrating Fault Tolerance, Store-and-Forward Buffering, and Mission Continuity[/yellow]\n"
        f"[green]Environment:[/green] [bold white]{ENVIRONMENT_NAME}[/bold white] | "
        f"[green]Uplink:[/green] [bold cyan]Satellite link (650 ms model)[/bold cyan]",
        border_style="red"
    ))
    console.print()

    # ─────────────────────────────────────────────────────────────────────────
    # Phase 0: Architectural Validation
    # ─────────────────────────────────────────────────────────────────────────
    console.print(Rule("[bold yellow]PHASE 0: ARCHITECTURAL HARD-CONSTRAINT VALIDATION[/bold yellow]"))
    checks = verify_architectural_isolation()
    for name, passed, details in checks:
        if not passed:
            console.print(f"[red]FAIL: {name}[/red]")
            return False
    console.print("[bold green][OK] Hard constraint verified: Zero direct robot <-> Command Post connections.[/bold green]\n")

    # ─────────────────────────────────────────────────────────────────────────
    # Phase 1: Pipeline Setup with Failure Hooks
    # ─────────────────────────────────────────────────────────────────────────
    console.print(Rule("[bold yellow]PHASE 1: SUBSYSTEM INITIALIZATION & FAULT HOOKS[/bold yellow]"))

    uplink_q: queue.Queue = queue.Queue()
    downlink_q: queue.Queue = queue.Queue()
    executor_inboxes: dict[str, queue.Queue] = {unit.unit_id: queue.Queue() for unit in EXECUTOR_UNITS}
    status_q: queue.Queue = queue.Queue()

    beacon_store = BeaconStore()
    grid_cfg = GridConfig()
    grid = GridWorld(width=grid_cfg.width, height=grid_cfg.height)

    anchor_cfg = AnchorConfig()
    drift_cfg = DriftConfig()
    anchor = GPSAnchor(origin_lat=anchor_cfg.origin_lat, origin_lon=anchor_cfg.origin_lon, origin_alt=anchor_cfg.origin_alt)
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

    live_map = LiveMap(uplink_q)
    mission_generator = MissionGenerator(live_map, mission_relay)

    dash_cfg = DashboardConfig(port=dashboard_port)  # Default keeps the registered Full Resilience port.
    init_dashboard(
        live_map,
        mission_generator,
        satellite_driver=satellite_driver,
        satellite_uplink=satellite_uplink,
        beacon_store=beacon_store,
        executor_units=[unit.to_dashboard_dict() for unit in EXECUTOR_UNITS],
    )
    dash_thread = threading.Thread(
        target=lambda: flask_app.run(host=dash_cfg.host, port=dash_cfg.port, debug=False, use_reloader=False),
        daemon=True,
    )
    dash_thread.start()

    beacon_receiver.start()
    live_map.start()
    retry_thread = threading.Thread(target=retry_queue.drain, daemon=True)
    retry_thread.start()

    console.print(f"[green][OK][/green] Pipeline active. Web dashboard on http://{dash_cfg.host}:{dash_cfg.port}")
    console.print(f"[green][OK][/green] In-situ Beacon Store and Outside Network Area ready.\n")

    # ─────────────────────────────────────────────────────────────────────────
    # Phase 2: Failure Case A — Mid-Mission Catastrophic Writer Loss
    # ─────────────────────────────────────────────────────────────────────────
    console.print(Rule("[bold red]PHASE 2: INJECTED FAILURE A -- WRITER ROBOT LOSS[/bold red]"))
    writer_cfg = WriterConfig()
    writer = WriterRobot(
        robot_id=writer_cfg.robot_id,
        grid=grid,
        beacon_store=beacon_store,
        start_row=writer_cfg.start_row,
        start_col=writer_cfg.start_col,
        rng_seed=42,
    )
    register_failure_targets(writer=writer, satellite_driver=satellite_driver, beacon_store=beacon_store)

    console.print(f"[{writer.robot_id}] Exploring USAR structure... Targets to drop before injected failure: 2 beacons.")
    # Arm the blackout before deposits so the packet queue visibly contains
    # real preserved data; no race can let the first packet escape beforehand.
    satellite_driver.link_disrupted = True
    console.print("[bold red]Satellite blackout armed: incoming beacons will be buffered by ONA store-and-forward.[/bold red]")

    # Explore until 2 beacons are dropped
    while writer.is_alive() and len(writer.beacons_dropped) < 2 and writer._step < 120:
        frontiers = writer.grid.get_frontiers()
        if not frontiers:
            break
        target = writer._nearest_frontier(frontiers)
        if target is None or not writer._step_toward(target):
            if target:
                writer.grid.grid[target[0], target[1]] = 2
            continue
        writer._step += 1
        writer._scan_and_maybe_drop()
        time.sleep(0.005)

    console.print(f"[yellow]Writer Robot placed {len(writer.beacons_dropped)} in-situ beacons:[/yellow]")
    for b in writer.beacons_dropped:
        console.print(f"  * [bold yellow]{b.beacon_id}[/bold yellow] ({b.event.type}, severity={b.event.severity:.2f}) at local ({b.position_local.x:.1f}m, {b.position_local.y:.1f}m)")

    # INJECT CATASTROPHIC FAILURE
    console.print("\n[bold red]!!! INJECTING CATASTROPHIC FAILURE: STRUCTURAL COLLAPSE / TOTAL POWER LOSS !!![/bold red]")
    writer.kill()
    console.print(f"[{writer.robot_id}] State: [bold red]{writer.state.name}[/bold red]. Explorer is permanently dead.")
    console.print(f"[{writer.robot_id}] Cells mapped before demise: {writer.get_telemetry().cells_visited} cells.")
    console.print("[bold green][OK] In-situ beacons remain deposited physically in the disaster zone (survived robot destruction).[/bold green]\n")

    # ─────────────────────────────────────────────────────────────────────────
    # Phase 3: Failure Case B — satellite disruption & store-and-forward buffering
    # ─────────────────────────────────────────────────────────────────────────
    console.print(Rule("[bold red]PHASE 3: INJECTED FAILURE B -- SATELLITE LINK DISRUPTION & RETRY QUEUE[/bold red]"))
    console.print("[bold red]Satellite pass/weather disruption remains active (100% outage)...[/bold red]")

    # Wait for ONA to pick up beacons and attempt transmission
    time.sleep(1.0)
    buffered_stats = retry_queue.stats()
    console.print(f"Store-and-Forward Queue State during Radio Blackout:")
    console.print(f"  * Buffered / pending packets: [bold red]{buffered_stats['pending']}[/bold red]")
    console.print(f"  * Successfully sent packets: [bold]{buffered_stats['sent']}[/bold]")
    console.print(f"  * Retry backoff engaged: Exponential retry backoff active.")

    console.print("\n[bold green]Restoring satellite pass window; retry queue will flush...[/bold green]")
    satellite_driver.link_disrupted = False

    # Confirm actual Command Post reception. Retry backoff plus satellite latency
    # is intentionally variable, so a fixed sleep could falsely claim recovery.
    expected_events = len(writer.beacons_dropped)
    deliveries_confirmed = _wait_for_command_post_events(live_map, expected_events)
    restored_stats = retry_queue.stats()
    console.print(f"Store-and-Forward Queue State post-restoration:")
    console.print(f"  * Successfully delivered packets: [bold green]{restored_stats['sent']}[/bold green]")
    console.print(f"  * Command Post-confirmed beacon events: [bold green]{len(live_map.get_active_events())} / {expected_events}[/bold green]")
    console.print(f"  * Packets lost: [bold green]{restored_stats['failed']}[/bold green]")
    if not deliveries_confirmed:
        console.print("[bold red][FAIL] Buffered packets did not reach Command Post before the recovery deadline.[/bold red]")
        beacon_receiver.stop()
        live_map.stop()
        return False
    console.print("[bold green][OK] All preserved beacons reached Command Post after satellite recovery.[/bold green]\n")

    # ─────────────────────────────────────────────────────────────────────────
    # Phase 4: Command Post Mission Briefing from Partial Data
    # ─────────────────────────────────────────────────────────────────────────
    console.print(Rule("[bold yellow]PHASE 4: COMMAND POST MISSION SYNTHESIS FROM SURVIVING BEACONS[/bold yellow]"))
    active_events = live_map.get_active_events()
    console.print(f"Command Post compiled tactical map from {len(active_events)} surviving beacon(s):")

    cp_table = Table(title="Surviving In-Situ Beacons at Command Post", show_header=True, header_style="bold blue")
    cp_table.add_column("Beacon ID", style="bold yellow")
    cp_table.add_column("Discovered Event", style="cyan")
    cp_table.add_column("Severity", style="green")
    cp_table.add_column("GPS Coordinates", style="white")

    for e in active_events:
        cp_table.add_row(e.beacon_id, e.event_type, f"{e.severity:.2f}", f"{e.lat:.6f} N, {e.lon:.6f} E")
    console.print(cp_table)

    briefing = mission_generator.generate()
    if briefing is None:
        console.print("[bold red]Error: No surviving events to brief![/bold red]")
        return False

    set_latest_briefing(briefing.to_dict())
    # Executor enters AWAITING_AUTHORIZATION — must not move until operator authorizes
    set_awaiting_authorization()
    console.print(f"[bold green][OK] Mission Briefing {briefing.briefing_id} prepared; ONA Role 4 will deliver it after authorization.[/bold green]")
    console.print("[bold yellow][COMMAND POST] Response units are awaiting authorization — select a unit, then click 'Send Mission'.[/bold yellow]\n")

    # ─────────────────────────────────────────────────────────────────────────
    # Phase 5: Executor Robot Navigates to Surviving Targets
    # ─────────────────────────────────────────────────────────────────────────
    console.print(Rule("[bold yellow]PHASE 5: EXECUTOR ROBOT INTERVENTION (MISSION CONTINUITY)[/bold yellow]"))
    # ── AUTHORIZATION GATE ──
    console.print("[bold yellow][COMMAND POST] Select an available response unit, then authorize its ONA Role 4 dispatch...[/bold yellow]")
    authorized = wait_for_authorization(timeout_s=300.0)
    if not authorized:
        console.print("[bold red][COMMAND POST] Authorization timeout — no response unit was dispatched.[/bold red]")
        return False
    selected_unit_id = get_selected_executor_unit_id()
    if selected_unit_id not in executor_inboxes:
        console.print("[bold red][COMMAND POST] No valid selected unit; resilience handoff cannot begin.[/bold red]")
        return False
    selected_unit = next(unit for unit in EXECUTOR_UNITS if unit.unit_id == selected_unit_id)
    executor = ExecutorRobot(
        robot_id=selected_unit_id,
        executor_inbox=executor_inboxes[selected_unit_id],
        status_queue=status_q,
        beacon_store=beacon_store,
        start_lat=selected_unit.start_lat,
        start_lon=selected_unit.start_lon,
    )
    console.print(f"[bold green][COMMAND POST] Mission authorized — ONA Role 4 dispatching {selected_unit.display_name}.[/bold green]")
    console.print(f"[{executor.robot_id}] Inheriting discoveries from deceased {writer.robot_id} without re-exploring whole building.")

    navigation_success = executor.await_and_run(timeout_s=5.0)
    completion_beacon_delivered = (
        _wait_for_completion_beacon(beacon_store, beacon_receiver, live_map, executor.robot_id)
        if navigation_success else False
    )
    success = navigation_success and completion_beacon_delivered

    reports = []
    while not status_q.empty():
        reports.append(status_q.get_nowait())

    arrivals = [r for r in reports if r["report_type"] == "arrival"]
    skipped_reports = [r for r in reports if r.get("report_type") == "skipped"]

    # ── Push cycle summary to dashboard (triggers summary modal) ──────────────
    # Build failure list from script-injected failures (already logged above)
    import time as _t_sum
    script_failures = [
        {
            "type": "writer_killed",
            "label": "Writer Robot Killed (Script Injection)",
            "timestamp": _t_sum.time(),
            "ts_str": "Phase 2",
            "detail": (
                f"Writer killed at exploration step {writer._step} — "
                f"{len(writer.beacons_dropped)} in-situ beacon(s) survived robot destruction; "
                f"Executor used surviving beacons for mission continuity"
            ),
        },
        {
            "type": "satellite_disrupted",
            "label": "Satellite Link Disruption (Script Injection)",
            "timestamp": _t_sum.time(),
            "ts_str": "Phase 3",
            "detail": (
                f"100% packet loss injected; "
                f"store-and-forward RetryQueue buffered packets during blackout — "
                f"{restored_stats['sent']} packet(s) drained on recovery, 0 lost"
            ),
        },
    ]
    # Also collect any additional failures triggered via dashboard buttons
    import copy as _copy_f
    import command_post.dashboard as _dash_mod_f
    with _dash_mod_f._failure_log_lock:
        dash_failures = _copy_f.deepcopy(_dash_mod_f._failure_log)
    # Merge: script failures first, then any dashboard-triggered ones not already captured
    all_failures = script_failures + [
        f for f in dash_failures
        if f["type"] not in {"writer_killed", "satellite_disrupted"}
    ]

    push_cycle_summary({
        "cycle": 1,
        "mission_id": briefing.briefing_id,
        "timestamp": _t_sum.time(),
        "targets_visited": len(arrivals),
        "targets_total": len(briefing.waypoints),
        "completion_pct": round(100 * len(arrivals) / max(len(briefing.waypoints), 1)),
        "completion_beacon_delivered": completion_beacon_delivered,
        "beacons_by_priority": [
            {
                "id": w.beacon_id,
                "type": w.event_type,
                "priority_pct": round(w.priority * 100),
            }
            for w in briefing.waypoints
        ],
        "beacons_skipped_ttl": len(skipped_reports),
        "drift_corrections": drift_model.corrections_applied(),
        "drift_total_adjustment_m": drift_model.total_adjustment_m(),
        "failures": all_failures,
    })

    console.print(f"\n[{executor.robot_id}] Mission Completed:")
    console.print(f"  * Targets successfully reached: [bold green]{len(arrivals)} / {len(briefing.waypoints)}[/bold green]")
    console.print(f"  * In-situ beacons scanned directly on entry: [bold]{executor._direct_beacons_read}[/bold]")
    console.print(
        "  * Completion status beacon at Command Post: "
        + ("[bold green]DELIVERED[/bold green]" if completion_beacon_delivered else "[bold red]NOT DELIVERED[/bold red]")
    )
    for arr in arrivals:
        console.print(f"  * [bold green][OK][/bold green] {arr['message']}")

    # Clean shutdown
    beacon_receiver.stop()
    live_map.stop()

    # ─────────────────────────────────────────────────────────────────────────
    # Phase 6: Resilience Scorecard
    # ─────────────────────────────────────────────────────────────────────────
    console.print(Rule("[bold yellow]PHASE 6: RESILIENCE & FAULT-TOLERANCE SCORECARD[/bold yellow]"))
    scorecard = Table(title="Failure Case & Graceful Degradation Evaluation", show_header=True, header_style="bold red")
    scorecard.add_column("Failure Scenario / Metric", style="white")
    scorecard.add_column("Requirement / Goal", style="yellow")
    scorecard.add_column("Observed System Behaviour", style="white")
    scorecard.add_column("Resilience Verdict", style="bold", justify="center")

    scorecard.add_row(
        "Writer Robot Death Mid-Mission",
        "Goal G1 / NF-3",
        f"Writer killed at step {writer._step}; {len(writer.beacons_dropped)} in-situ beacons survived",
        "[green]TOLERATED[/green]"
    )
    scorecard.add_row(
        "Complete Satellite Link Disruption",
        "ON-5 / Goal G6",
        "100% loss injected; RetryQueue buffered packets and drained on recovery",
        "[green]TOLERATED[/green]"
    )
    scorecard.add_row(
        "Zero Direct Link Constraint",
        "Hard Constraint",
        "Relay through ONA strictly enforced even during failure recovery",
        "[green]VERIFIED[/green]"
    )
    scorecard.add_row(
        "Mission Continuity Handoff",
        "Goal G4 / EX-2",
        f"{selected_unit.display_name} reached {len(arrivals)} targets and sent a completion beacon",
        "[green]SUCCESS[/green]" if success else "[red]INCOMPLETE[/red]"
    )

    console.print(scorecard)
    result_label = "COMPLETED SUCCESSFULLY" if success else "COMPLETED WITH AN UNVERIFIED FINAL DELIVERY"
    result_style = "green" if success else "red"
    console.print(Panel.fit(
        f"[bold {result_style}][OK] FAILURE INJECTION DEMO {result_label}[/bold {result_style}]\n"
        f"Fault tolerance log saved to: [cyan]{log_file_path}[/cyan]\n"
        "Demonstrates preserved beacons, buffered satellite delivery, and ONA-only mission continuity.",
        border_style=result_style,
    ))

    return success


if __name__ == "__main__":
    success = run_failure_demo()
    sys.exit(0 if success else 1)
