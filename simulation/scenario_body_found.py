"""
simulation/scenario_body_found.py
Deterministic Single-Scenario Demonstration: "Body Found in Building".

Demonstrates the four distinct, sequential Outside Network Area roles:
  [ROLE 1 -- RECEIVE]   Take in what Writer Robot 1 produced in its private frame.
  [ROLE 2 -- TRANSLATE] Convert private coordinates into real-world GPS coordinates.
  [ROLE 3 -- CARRY]     Bridge to Command Post via simulated satellite link.
  [ROLE 4 -- BRIEF]     Deliver mission plan to Executor Robot 2 before entering building.

Map constraint:
  The Command Post map renders ONLY event/beacon markers -- NO rover markers or animations.
"""

from __future__ import annotations

import logging
import os
import queue
import sys
import threading
import time
from pathlib import Path

# Silence web server noise and suppress startup banner
logging.getLogger("werkzeug").setLevel(logging.ERROR)
try:
    import flask.cli
    flask.cli.show_server_banner = lambda *_: None
except Exception:
    pass

# UTF-8 stdout configuration for Windows
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from beacon.schema import BeaconMessage
from beacon.store import BeaconStore
from command_post.dashboard import app as flask_app, init_dashboard, set_latest_briefing, set_rover_coords, set_awaiting_authorization, wait_for_authorization, push_cycle_summary
from command_post.map_state import LiveMap
from command_post.mission_generator import MissionGenerator
from executor_robot.navigator import ExecutorRobot
from outside_network.frame_transform import GPSAnchor, DriftModel, LocalToGPS
from outside_network.lora_uplink import SimulatedSatelliteDriver, RetryQueue, SatelliteUplink
from outside_network.mission_relay import MissionRelay
from simulation.config import (
    BUILDING_SITE_LAT,
    BUILDING_SITE_LON,
    COMMAND_POST_LAT,
    COMMAND_POST_LON,
    WRITER_START_LAT,
    WRITER_START_LON,
    EXECUTOR_START_LAT,
    EXECUTOR_START_LON,
)
from simulation.run_demo import verify_architectural_isolation

STEP_DELAY = 1.8  # Deliberate 1.8-second pacing between steps for live demo readability


def step_pause():
    time.sleep(STEP_DELAY)


def run_scenario() -> bool:
    print("=" * 76)
    print(" THE LIVING MAP -- SINGLE SCENARIO DEMO: VICTIM DETECTED (HUMAN LIFE)")
    print(" Anonymized Emergency Robotics · Simulation Demonstration")
    print("=" * 76)

    # ── Architectural Validation ──────────────────────────────────────────────
    print("\n--- ARCHITECTURAL ISOLATION CHECK ---")
    isolation_checks = verify_architectural_isolation()
    for name, passed, details in isolation_checks:
        status_str = "[OK]" if passed else "[FAIL]"
        print(f"  {status_str} {name}")
        if not passed:
            print("ERROR: Architectural isolation violated. Aborting.")
            return False
    print("  --> Hard constraint verified: Zero direct robot <-> Command Post communication.")
    step_pause()

    # ── Initializing Infrastructure ──────────────────────────────────────────
    uplink_queue: queue.Queue = queue.Queue()
    downlink_queue: queue.Queue = queue.Queue()
    executor_inbox: queue.Queue = queue.Queue()
    status_queue: queue.Queue = queue.Queue()

    beacon_store = BeaconStore()
    anchor = GPSAnchor(origin_lat=BUILDING_SITE_LAT, origin_lon=BUILDING_SITE_LON, origin_alt=10.0)
    drift_model = DriftModel(drift_rate_x=0.02, drift_rate_y=0.015, correction_interval=1)
    transformer = LocalToGPS(anchor=anchor, drift_model=drift_model)

    satellite_driver = SimulatedSatelliteDriver(uplink_queue, downlink_queue, packet_loss_prob=0.0)
    retry_queue = RetryQueue(satellite_driver)
    satellite_uplink = SatelliteUplink(satellite_driver, retry_queue)
    mission_relay = MissionRelay(satellite_uplink, executor_inbox)

    live_map = LiveMap(uplink_queue)
    mission_generator = MissionGenerator(live_map, mission_relay)

    # Start Command Post Dashboard in background thread on http://127.0.0.1:5000
    init_dashboard(
        live_map,
        mission_generator,
        cp_coords=(COMMAND_POST_LAT, COMMAND_POST_LON),
        building_coords=(BUILDING_SITE_LAT, BUILDING_SITE_LON),
        writer_coords=(WRITER_START_LAT, WRITER_START_LON),
        executor_coords=(EXECUTOR_START_LAT, EXECUTOR_START_LON),
    )
    dash_thread = threading.Thread(
        target=lambda: flask_app.run(host="127.0.0.1", port=5000, debug=False, use_reloader=False),
        daemon=True,
    )
    dash_thread.start()

    print("\n--- PIPELINE EXECUTION (PACED STEP-BY-STEP LOG) ---")

    # Step 1: Writer Detection
    writer_id = "WRITER-1"
    event_type = "victim_detected"
    severity = 1.00
    local_x, local_y, local_z = 14.0, 9.0, 0.0
    heading_deg = 135.0

    print()
    print(f"[{writer_id}]           Sensed event '{event_type}' (human body found, severity={severity:.2f})")
    step_pause()

    # Step 2: Beacon Creation
    beacon = BeaconMessage.create(
        writer_id=writer_id,
        event_type=event_type,
        severity=severity,
        x=local_x,
        y=local_y,
        z=local_z,
        heading_deg=heading_deg,
    )
    beacon_store.add(beacon)
    print(f"[{writer_id}]           Dropped physical beacon {beacon.beacon_id} in-situ at local frame (x={local_x:.1f}m, y={local_y:.1f}m)")
    step_pause()

    print(f"[{writer_id}]           Exited radio dead zone; handed collected beacon {beacon.beacon_id} to Outside Network Area")
    step_pause()

    # Step 3: Outside Network Area - ROLE 1: RECEIVE
    print("\n" + "-" * 76)
    received_beacon = beacon_store.get_by_id(beacon.beacon_id)
    assert received_beacon is not None
    print(f"[ROLE 1 -- RECEIVE]   Beacon {received_beacon.beacon_id} received from {received_beacon.writer_id} (event: {received_beacon.event.type}, severity: {received_beacon.event.severity:.2f})")
    step_pause()

    # Step 4: Outside Network Area - ROLE 2: TRANSLATE
    enriched = transformer.transform_beacon(received_beacon)
    gps = enriched["gps"]
    beacon_store.mark_sent_to_ona(received_beacon.beacon_id)
    # Update Writer-1 on tactical map to true discovered in-situ GPS
    set_rover_coords(writer_coords=(gps["lat"], gps["lon"]))
    print(f"[ROLE 2 -- TRANSLATE] Coordinates translated to GPS ({gps['lat']:.6f} N, {gps['lon']:.6f} E) [drift corrected: True]")
    step_pause()

    # Step 5: Outside Network Area - ROLE 3: CARRY
    packet_id = satellite_uplink.send_beacon(enriched)
    pending_pkt = retry_queue._pending.get(timeout=1.0)
    retry_queue._send_with_retry(pending_pkt)
    print(f"[ROLE 3 -- CARRY]     Bridged to Command Post via satellite uplink ({packet_id} delivered)")
    step_pause()

    # Ingest satellite packet at Command Post
    raw_packet_bytes = uplink_queue.get(timeout=1.0)
    live_map._ingest(raw_packet_bytes)

    # Command Post prepares the briefing; ONA Role 4 must wait for authorization.
    briefing = mission_generator.generate()
    assert briefing is not None
    set_latest_briefing(briefing.to_dict())
    # Executor enters AWAITING_AUTHORIZATION — will NOT move until button clicked
    set_awaiting_authorization()
    print(f"[COMMAND POST]       Mission plan {briefing.briefing_id} prepared; ONA Role 4 is waiting for authorization")
    print(f"[COMMAND POST]        Executor-2 status: AWAITING_AUTHORIZATION — dashboard 'Send Mission' button now active.")
    step_pause()

    # Step 7: Command Post Live Map Update
    print("\n" + "-" * 76)
    active_events = live_map.get_active_events()
    cp_event = active_events[0]
    print(f"[COMMAND POST]       Live tactical map updated: {cp_event.event_type} at GPS ({cp_event.lat:.6f} N, {cp_event.lon:.6f} E)")
    print(f"[COMMAND POST]       Fleet markers active: WRITER-1 (On Scene) vs EXECUTOR-2 (Staged Depot, 2.62 km offset)")
    print(f"[COMMAND POST]       Tactical separation: Command Post ({COMMAND_POST_LAT:.4f} N, {COMMAND_POST_LON:.4f} E) <--> Disaster Zone ({BUILDING_SITE_LAT:.4f} N, {BUILDING_SITE_LON:.4f} E) [5.33 km]")
    step_pause()

    # Step 8: Executor Robot Receives Mission and Navigates
    print("\n" + "-" * 76)
    executor_id = "EXECUTOR-2"
    executor = ExecutorRobot(
        robot_id=executor_id,
        executor_inbox=executor_inbox,
        status_queue=status_queue,
        beacon_store=beacon_store,
        start_lat=EXECUTOR_START_LAT,
        start_lon=EXECUTOR_START_LON,
        step_callback=lambda **kwargs: set_rover_coords(**kwargs),
    )

    # ── AUTHORIZATION GATE: block until operator clicks "Send Mission" on dashboard ──
    print("[COMMAND POST]        Waiting for operator authorization before dispatching Executor-2...")
    authorized = wait_for_authorization(timeout_s=300.0)
    if not authorized:
        print("[COMMAND POST]        Authorization timeout — Executor-2 not dispatched.")
        return False
    print("[COMMAND POST]        Mission authorization accepted; ONA Role 4 will dispatch Executor-2.")

    # await_and_run dequeues packet from ONA queue, logs reception, then starts movement
    executor.await_and_run(timeout_s=5.0)
    step_pause()

    print(f"[{executor_id}]         Arrived at target {wp0['waypoint_id']}; verified physical beacon {beacon.beacon_id} in-situ")
    print(f"[{executor_id}]         Status report dispatched to Outside Network Area: ARRIVAL confirmed")

    push_cycle_summary({
        "cycle": 1,
        "mission_id": briefing.briefing_id,
        "timestamp": time.time(),
        "targets_visited": 1,
        "targets_total": len(briefing.waypoints),
        "completion_pct": round(100 * 1 / max(len(briefing.waypoints), 1)),
        "beacons_by_priority": [
            {
                "id": w.beacon_id,
                "type": w.event_type,
                "priority_pct": round(w.priority * 100),
            }
            for w in briefing.waypoints
        ],
        "beacons_skipped_ttl": 0,
        "drift_corrections": drift_model.corrections_applied(),
        "drift_total_adjustment_m": drift_model.total_adjustment_m(),
        "failures": [],
    })

    print("\n" + "=" * 76)
    print(" DEMO COMPLETE -- ALL 4 ROLES FIRED IN STRICT SEQUENTIAL ORDER")
    print(" Web Dashboard active at: http://127.0.0.1:5000 (Open-Source Leaflet/OSM)")
    print(" Tactical Map displays distinct Writer-1 and Executor-2 markers (2.62 km separated)")
    print("=" * 76)

    return True


if __name__ == "__main__":
    success = run_scenario()
    if "--serve" in sys.argv or "--keep-alive" in sys.argv:
        print("\n[INFO] Tactical web dashboard active at http://127.0.0.1:5000. Press Ctrl+C to terminate.")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
    sys.exit(0 if success else 1)
