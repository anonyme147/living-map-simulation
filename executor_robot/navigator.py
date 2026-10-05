"""
executor_robot/navigator.py
ExecutorRobot: receives a mission briefing via ONA, navigates to prioritized
targets using beacon data, and reports status back through ONA only.

PRD requirements:
  EX-1 Receives mission via Outside Network Area only
  EX-2 Navigates to briefed locations without re-exploring whole space
  EX-3 Reads beacons directly on entry to refine navigation
  EX-4 Prioritizes targets by severity/TTL
  EX-5 Reports mission outcome via ONA

CONSTRAINT: This module does NOT import from command_post/.
All inputs come from executor_inbox queue (ONA→Executor).
All outputs go to status_queue (Executor→ONA).
"""

from __future__ import annotations

import json
import logging
import math
import queue
import time
import urllib.request
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Callable, List, Optional, Tuple

from beacon.schema import BeaconMessage
from beacon.store import BeaconStore
from executor_robot.mission_parser import ParsedMission, Waypoint, parse_mission
from executor_robot.status_reporter import StatusReporter
# Removed direct import of command_post to satisfy architectural isolation
logger = logging.getLogger(__name__)

BEACON_READ_RADIUS = 5.0    # metres — direct beacon scan range on entry
ARRIVAL_THRESHOLD = 2.0     # metres — considered "arrived" at waypoint
BEACON_READ_GAP_STEPS = 5   # movement steps allowed without a confirmed beacon read

# Read from simulation config so timing can be tuned centrally
try:
    from simulation.config import BUILDING_SITE_LAT as _BUILDING_LAT, BUILDING_SITE_LON as _BUILDING_LON, ExecutorConfig as _ExCfg
    NAV_STEP_DELAY = _ExCfg().nav_step_delay_s
except Exception:
    _BUILDING_LAT, _BUILDING_LON = 38.8895, -77.0353
    NAV_STEP_DELAY = 0.14


def _fetch_osrm_route(start_lat: float, start_lon: float, end_lat: float, end_lon: float) -> Optional[List[Tuple[float, float]]]:
    """
    Calls public OSRM driving API to get a real street route between (start_lat, start_lon)
    and (end_lat, end_lon). Returns a list of (lat, lon) coordinates along streets, or None on error.
    """
    url = f"https://router.project-osrm.org/route/v1/driving/{start_lon},{start_lat};{end_lon},{end_lat}?geometries=geojson&overview=full"
    try:
        req = urllib.request.Request(url, headers={"User-Agent": "LivingMap-USAR/1.0"})
        with urllib.request.urlopen(req, timeout=3.0) as resp:
            data = json.loads(resp.read().decode("utf-8"))
            if data.get("code") == "Ok" and data.get("routes"):
                raw_coords = data["routes"][0]["geometry"]["coordinates"]
                # OSRM GeoJSON returns [lon, lat], convert to [lat, lon]
                route_pts = [(pt[1], pt[0]) for pt in raw_coords]
                return route_pts
    except Exception as e:
        logger.warning(f"[OSRM] Route fetch failed: {e}")
    return None


class ExecutorState(Enum):
    IDLE = auto()
    AWAITING_MISSION = auto()
    NAVIGATING = auto()
    AT_TARGET = auto()
    COMPLETE = auto()
    FAILED = auto()


@dataclass
class NavTelemetry:
    robot_id: str
    state: ExecutorState
    lat: float
    lon: float
    mission_id: Optional[str]
    waypoints_total: int
    waypoints_visited: int
    current_target: Optional[str]
    direct_beacons_read: int
    step: int


class ExecutorRobot:
    """
    Executor Robot that navigates to mission waypoints in priority order.
    Uses GPS coordinates from mission briefing (no re-exploration needed).
    Also scans for beacons directly on entry to cross-check/refine targets.
    """

    def __init__(
        self,
        robot_id: str,
        executor_inbox: queue.Queue,
        status_queue: queue.Queue,
        beacon_store: BeaconStore,
        start_lat: float = _BUILDING_LAT,
        start_lon: float = _BUILDING_LON,
        battery_pct: float = 100.0,
        step_callback: Optional[Callable[["ExecutorRobot"], None]] = None,
    ):
        self.robot_id = robot_id
        self._inbox = executor_inbox
        self.reporter = StatusReporter(robot_id, status_queue)
        self.beacon_store = beacon_store
        self.lat = start_lat
        self.lon = start_lon
        self.battery_pct = battery_pct
        self.state = ExecutorState.IDLE
        self.step_callback = step_callback

        self._mission: Optional[ParsedMission] = None
        self._step = 0
        self._direct_beacons_read = 0
        self._steps_since_beacon_read = 0
        self._last_successful_beacon_position: Optional[Tuple[float, float]] = None

    # ── Public API ────────────────────────────────────────────────────────────

    def await_and_run(self, timeout_s: float = 30.0) -> bool:
        """
        Wait for a mission briefing from ONA, then execute it.
        Returns True if mission completed successfully.
        """
        self.state = ExecutorState.AWAITING_MISSION
        logger.info(f"[{self.robot_id}] Awaiting mission briefing via ONA…")

        try:
            payload = self._inbox.get(timeout=timeout_s)
        except queue.Empty:
            logger.error(f"[{self.robot_id}] No mission received within {timeout_s}s")
            self.state = ExecutorState.FAILED
            self.reporter.report_error("Mission timeout — no briefing received")
            return False

        mission = parse_mission(payload)
        if mission is None:
            self.state = ExecutorState.FAILED
            self.reporter.report_error("Received malformed mission briefing")
            return False

        self._mission = mission
        # Tuneable energy model: 0.30% to receive and validate a mission briefing.
        self._drain_battery(0.30, "mission reception")
        t_recv = time.time()
        logger.info(
            f"[{self.robot_id}] [PACKET_RECEIVED] Mission received via ONA Role 4 (BRIEF) downlink: {mission.briefing_id} "
            f"(Relay: {mission.relay_id}) | {len(mission.waypoints)} waypoints | {mission.summary} at t={t_recv:.3f}s"
        )
        if mission.warnings:
            for w in mission.warnings:
                logger.warning(f"[{self.robot_id}] ⚠️ {w}")

        return self.execute_mission()

    def execute_mission(self) -> bool:
        """Execute the stored mission. Returns True on success."""
        if self._mission is None:
            return False

        mission = self._mission
        self.state = ExecutorState.NAVIGATING

        logger.info(f"[{self.robot_id}] Starting mission execution — {len(mission.waypoints)} targets")

        # Part 1 & 2: Outdoor street-based navigation leg (Staging Depot -> Building Entrance)
        # Uses real street routing via OSRM, animating marker along actual road geometry
        # Part 3: Handoff transition event logged at building entrance before indoor navigation starts
        self._navigate_outdoor_street_leg(target_lat=_BUILDING_LAT, target_lon=_BUILDING_LON)

        # Indoor beacon-guided navigation (starts continuously from building entrance handoff position)
        visited_count = 0
        last_confirmed_beacon_id: Optional[str] = None
        for waypoint in mission.waypoints:
            # Decision point check: if beacon is force-expired (TTL=0), skip target
            if self.beacon_store is not None and self.beacon_store.is_expired(waypoint.beacon_id):
                logger.warning(
                    f"[{self.robot_id}] Beacon {waypoint.beacon_id} EXPIRED — skipping waypoint {waypoint.waypoint_id}"
                )
                self.reporter.report_skipped(waypoint.waypoint_id, waypoint.beacon_id, "ttl_expired")
                continue

            if not self._navigate_to(waypoint):
                logger.warning(f"[{self.robot_id}] Could not reach {waypoint.waypoint_id}")
                continue

            # Arrived — scan for direct beacons to refine understanding
            self._scan_direct_beacons()

            # Confirm arrival
            waypoint.visited = True
            waypoint.confirmed = True
            visited_count += 1
            last_confirmed_beacon_id = waypoint.beacon_id
            logger.info(
                "[%s] Arrival confirmed; waypoint=%s event_type=%s",
                self.robot_id, waypoint.waypoint_id, waypoint.event_type,
            )
            self.reporter.report_arrival(
                waypoint.waypoint_id, waypoint.beacon_id, waypoint.event_type
            )

            if self.step_callback:
                # Pass executor coordinates to dashboard for live map update
                self.step_callback(executor_coords=(self.lat, self.lon))

        self.state = ExecutorState.COMPLETE
        mission_succeeded = visited_count >= 2  # PRD success: reach >=2 distinct beacon sites
        if mission_succeeded:
            self._drop_completion_beacon(last_confirmed_beacon_id)
        self.reporter.report_mission_complete(visited_count, len(mission.waypoints))
        logger.info("[%s] Mission complete; visited=%s/%s", self.robot_id, visited_count, len(mission.waypoints))
        return mission_succeeded

    def _drop_completion_beacon(self, reference_beacon_id: Optional[str]) -> None:
        """Record direct Executor completion at the final confirmed beacon zone.

        The new beacon is placed in BeaconStore, so the existing ONA receiver
        performs RECEIVE, TRANSLATE, and CARRY exactly as it does for Writer
        beacons. Executor has no Command Post communication path here.
        """
        if self.beacon_store is None or reference_beacon_id is None:
            logger.warning("[%s] Completion beacon not dropped: no confirmed target beacon", self.robot_id)
            return

        reference = self.beacon_store.get_by_id(reference_beacon_id)
        if reference is None:
            logger.warning("[%s] Completion beacon not dropped: target beacon %s is unavailable", self.robot_id, reference_beacon_id)
            return

        completion_beacon = BeaconMessage.create(
            writer_id=self.robot_id,
            event_type="mission_completed",
            severity=1.0,
            x=reference.position_local.x,
            y=reference.position_local.y,
            z=reference.position_local.z,
            heading_deg=reference.position_local.heading_deg,
        )
        if self.beacon_store.add(completion_beacon):
            logger.info(
                "[%s] Completion beacon dropped: %s at confirmed target %s",
                self.robot_id,
                completion_beacon.beacon_id,
                reference_beacon_id,
            )
        else:
            logger.error("[%s] Completion beacon ID collision: %s", self.robot_id, completion_beacon.beacon_id)

    def _navigate_outdoor_street_leg(self, target_lat: float = _BUILDING_LAT, target_lon: float = _BUILDING_LON) -> None:
        """
        Part 1 & 2: Outdoor street-based navigation from Staging Depot to Building Entrance via OSRM.
        Animates Executor marker smoothly along actual road geometry.
        Part 3: Logs transition event upon arrival at building entrance before handing off to indoor logic.
        """
        start_lat, start_lon = self.lat, self.lon
        logger.info(
            "[%s] Requesting outdoor route; from=(%.6f, %.6f) to=(%.6f, %.6f)",
            self.robot_id, start_lat, start_lon, target_lat, target_lon,
        )

        route = _fetch_osrm_route(start_lat, start_lon, target_lat, target_lon)

        if route and len(route) >= 2:
            logger.info("[%s] Outdoor route received; coordinates=%s", self.robot_id, len(route))
            NUM_STEPS = 40
            sampled_path = []
            for i in range(NUM_STEPS + 1):
                idx = int(i * (len(route) - 1) / float(NUM_STEPS))
                sampled_path.append(route[idx])

            log_samples = {
                0: "OUTDOOR START (0%)",
                10: "STREET CURVE 1 (25%)",
                20: "STREET MIDPOINT (50%)",
                30: "STREET CURVE 2 (75%)",
                40: "ENTRANCE ARRIVAL (100%)",
            }

            for idx, (cur_lat, cur_lon) in enumerate(sampled_path):
                previous_lat, previous_lon = self.lat, self.lon
                self.lat, self.lon = cur_lat, cur_lon
                self._drain_battery(_haversine_m(previous_lat, previous_lon, self.lat, self.lon) * 0.0005, "movement")
                self._step += 1

                if self.step_callback:
                    self.step_callback(executor_coords=(self.lat, self.lon), executor_battery=self.battery_pct)

                if idx in log_samples:
                    label = log_samples[idx]
                    logger.info(
                        "[%s] Outdoor route checkpoint=%s coordinates=(%.6f, %.6f)",
                        self.robot_id, label, cur_lat, cur_lon,
                    )

                if idx < NUM_STEPS:
                    time.sleep(NAV_STEP_DELAY)
        else:
            logger.warning("[%s] Outdoor route unavailable; using direct transit", self.robot_id)
            NUM_STEPS = 30
            for i in range(NUM_STEPS + 1):
                frac = i / float(NUM_STEPS)
                cur_lat = start_lat + (target_lat - start_lat) * frac
                cur_lon = start_lon + (target_lon - start_lon) * frac
                previous_lat, previous_lon = self.lat, self.lon
                self.lat, self.lon = cur_lat, cur_lon
                self._drain_battery(_haversine_m(previous_lat, previous_lon, self.lat, self.lon) * 0.0005, "movement")
                if self.step_callback:
                    self.step_callback(executor_coords=(self.lat, self.lon), executor_battery=self.battery_pct)
                time.sleep(NAV_STEP_DELAY)

        # Snap to exact building entrance coordinates
        self.lat, self.lon = target_lat, target_lon
        if self.step_callback:
            self.step_callback(executor_coords=(self.lat, self.lon), executor_battery=self.battery_pct)

        logger.info("[%s] Entered indoor navigation zone at (%.6f, %.6f)", self.robot_id, self.lat, self.lon)

    def get_telemetry(self) -> NavTelemetry:
        visited = sum(1 for w in self._mission.waypoints if w.visited) if self._mission else 0
        current = next((w.waypoint_id for w in (self._mission.waypoints if self._mission else []) if not w.visited), None)
        return NavTelemetry(
            robot_id=self.robot_id,
            state=self.state,
            lat=self.lat,
            lon=self.lon,
            mission_id=self._mission.briefing_id if self._mission else None,
            waypoints_total=len(self._mission.waypoints) if self._mission else 0,
            waypoints_visited=visited,
            current_target=current,
            direct_beacons_read=self._direct_beacons_read,
            step=self._step,
        )

    # ── Internal navigation ───────────────────────────────────────────────────

    def _navigate_to(self, waypoint: Waypoint, monitor_beacon_gap: bool = True) -> bool:
        """
        Explicit step-by-step waypoint path navigation for Executor Robot.
        Reads CURRENT marker coordinates (lat1, lon1) and target coordinates (lat2, lon2).
        Movement ONLY begins AFTER mission packet has been received.
        Pacing is distance-proportional for smooth visual travel across the map.
        """
        lat1, lon1 = self.lat, self.lon
        lat2, lon2 = waypoint.lat, waypoint.lon
        logger.info(
            f"[{self.robot_id}] Navigating to {waypoint.waypoint_id}: "
            f"{waypoint.event_type} [{(waypoint.priority*100):.0f}% priority] "
            f"from ({lat1:.6f}, {lon1:.6f}) to ({lat2:.6f}, {lon2:.6f})"
        )

        dist_m = _haversine_m(lat1, lon1, lat2, lon2)
        if dist_m > 100.0:
            NUM_WAYPOINTS = 30
            STEP_DELAY_S = NAV_STEP_DELAY
        else:
            NUM_WAYPOINTS = max(10, min(25, int(dist_m * 1.5)))
            STEP_DELAY_S = NAV_STEP_DELAY * 0.6

        # Generate explicit path of intermediate waypoints
        path_waypoints = []
        for k in range(NUM_WAYPOINTS + 1):
            fraction = k / float(NUM_WAYPOINTS)
            wp_lat = lat1 + (lat2 - lat1) * fraction
            wp_lon = lon1 + (lon2 - lon1) * fraction
            path_waypoints.append((wp_lat, wp_lon))

        # Sample checkpoints to log explicitly
        idx_20 = int(NUM_WAYPOINTS * 0.20)
        idx_50 = int(NUM_WAYPOINTS * 0.50)
        idx_80 = int(NUM_WAYPOINTS * 0.80)
        sample_indices = {
            0: "START (0%)",
            idx_20: f"STEP {idx_20:02d} (20%)",
            idx_50: f"MIDPOINT ({idx_50:02d})",
            idx_80: f"STEP {idx_80:02d} (80%)",
            NUM_WAYPOINTS: f"TARGET ({NUM_WAYPOINTS:02d})"
        }

        # Advance marker ONE waypoint at a time in sequence
        for idx, (cur_lat, cur_lon) in enumerate(path_waypoints):
            previous_lat, previous_lon = self.lat, self.lon
            self.lat, self.lon = cur_lat, cur_lon
            # Tuneable energy model: 0.0005% per metre travelled.
            self._drain_battery(_haversine_m(previous_lat, previous_lon, self.lat, self.lon) * 0.0005, "movement")
            self._step += 1

            if self.step_callback:
                self.step_callback(executor_coords=(self.lat, self.lon), executor_battery=self.battery_pct)

            if monitor_beacon_gap and self._beacon_read_gap_requires_repair():
                return self._repair_beacon_read_gap_and_resume(waypoint)

            if idx in sample_indices:
                label = sample_indices[idx]
                logger.info(
                    "[%s] Navigation checkpoint=%s coordinates=(%.6f, %.6f)",
                    self.robot_id, label, cur_lat, cur_lon,
                )

            if idx < NUM_WAYPOINTS:
                time.sleep(STEP_DELAY_S)

        # Snap exactly to final target coordinate (lat2, lon2) after last waypoint
        self.lat, self.lon = lat2, lon2
        if self.step_callback:
            self.step_callback(executor_coords=(self.lat, self.lon), executor_battery=self.battery_pct)

        self.state = ExecutorState.AT_TARGET
        return True

    def _beacon_read_gap_requires_repair(self) -> bool:
        """Record an in-route beacon read and report whether reconnection is needed."""
        if self._scan_direct_beacons():
            self._steps_since_beacon_read = 0
            self._last_successful_beacon_position = (self.lat, self.lon)
            return False

        if self._last_successful_beacon_position is None:
            # There is no known beacon position to which the Executor can safely return.
            return False

        self._steps_since_beacon_read += 1
        return self._steps_since_beacon_read >= BEACON_READ_GAP_STEPS

    def _repair_beacon_read_gap_and_resume(self, waypoint: Waypoint) -> bool:
        """Return to the last verified beacon position, then resume the mission target."""
        if self._last_successful_beacon_position is None:
            return False

        repair_lat, repair_lon = self._last_successful_beacon_position
        logger.warning(
            "[%s] Beacon read gap reached %s steps; returning to last verified beacon position",
            self.robot_id,
            BEACON_READ_GAP_STEPS,
        )

        recovery_waypoint = Waypoint(
            waypoint_id="RECOVERY-BEACON",
            beacon_id=waypoint.beacon_id,
            event_type="beacon_reconnection",
            priority=waypoint.priority,
            severity=waypoint.severity,
            lat=repair_lat,
            lon=repair_lon,
            alt=waypoint.alt,
            estimated_ttl_remaining_s=waypoint.estimated_ttl_remaining_s,
        )
        if not self._navigate_to(recovery_waypoint, monitor_beacon_gap=False):
            return False

        if not self._scan_direct_beacons():
            logger.warning("[%s] Recovery position no longer contains a readable beacon", self.robot_id)
            return False

        self._steps_since_beacon_read = 0
        self._last_successful_beacon_position = (self.lat, self.lon)
        self.state = ExecutorState.NAVIGATING
        # The target is resumed only after a successful reconnection. Gap
        # monitoring is disabled on this resumed leg to avoid an endless return
        # loop when the next beacon is beyond the configured read radius.
        return self._navigate_to(waypoint, monitor_beacon_gap=False)



    def _scan_direct_beacons(self) -> bool:
        """
        PRD EX-3: Read beacons directly within range at current position.
        Converts GPS position back to approximate local coords for beacon lookup.
        """
        # Approximate local coords from GPS (inverse of frame_transform)
        # Origin: shared public demonstration map anchor.
        METRES_PER_DEG_LAT = 111_320.0
        lat_origin = _BUILDING_LAT
        lon_origin = _BUILDING_LON
        metres_per_deg_lon = METRES_PER_DEG_LAT * math.cos(math.radians(lat_origin))

        x_local = (self.lon - lon_origin) * metres_per_deg_lon
        y_local = (self.lat - lat_origin) * METRES_PER_DEG_LAT

        if self.beacon_store is None:
            return False

        nearby = self.beacon_store.get_in_range(x_local, y_local, BEACON_READ_RADIUS)
        if nearby:
            # Tuneable energy model: 0.08% per directly scanned beacon.
            self._drain_battery(0.08 * len(nearby), "in-situ beacon scan")
            self._direct_beacons_read += len(nearby)
            for b in nearby:
                logger.debug(
                    f"[{self.robot_id}] In-situ beacon scan: {b.beacon_id} "
                    f"({b.event.type}, sev={b.event.severity:.2f}) confirmed on scene"
                )
            return True

        return False

    def _drain_battery(self, amount: float, action: str) -> None:
        previous = self.battery_pct
        self.battery_pct = max(0.0, self.battery_pct - amount)
        if previous >= 20.0 and self.battery_pct < 20.0:
            logger.warning(f"[{self.robot_id}] Battery critical: {self.battery_pct:.0f}%")


# ── Geo helpers ───────────────────────────────────────────────────────────────

def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Distance in metres between two GPS coordinates."""
    R = 6_371_000.0  # Earth radius metres
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlam/2)**2
    return 2 * R * math.asin(math.sqrt(a))


def _bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Compass bearing in degrees from (lat1,lon1) to (lat2,lon2)."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlam = math.radians(lon2 - lon1)
    x = math.sin(dlam) * math.cos(phi2)
    y = math.cos(phi1)*math.sin(phi2) - math.sin(phi1)*math.cos(phi2)*math.cos(dlam)
    return (math.degrees(math.atan2(x, y)) + 360) % 360


def _move(lat: float, lon: float, bearing_deg: float, dist_m: float) -> Tuple[float, float]:
    """Move dist_m metres along bearing from (lat, lon). Returns new (lat, lon)."""
    R = 6_371_000.0
    d = dist_m / R
    b = math.radians(bearing_deg)
    phi1 = math.radians(lat)
    lam1 = math.radians(lon)
    phi2 = math.asin(math.sin(phi1)*math.cos(d) + math.cos(phi1)*math.sin(d)*math.cos(b))
    lam2 = lam1 + math.atan2(math.sin(b)*math.sin(d)*math.cos(phi1), math.cos(d)-math.sin(phi1)*math.sin(phi2))
    return math.degrees(phi2), math.degrees(lam2)
