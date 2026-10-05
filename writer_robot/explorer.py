"""
writer_robot/explorer.py
WriterRobot: autonomous frontier-based explorer for GPS-denied USAR environments.

Behaviour loop:
  1. Identify nearest frontier cell.
  2. Move toward it (BFS pathfinding, 1 cell per step).
  3. Scan sensors at current position.
  4. If fused confidence > CONFIDENCE_THRESHOLD for any event type → drop beacon.
  5. Mark cell visited; repeat until no frontiers remain or robot is killed.

PRD requirements covered:
  WR-1 Autonomous exploration (frontier-based, no GPS)
  WR-2 ≥2 event types detected
  WR-3 Confidence threshold before beacon drop
  WR-4 Beacon deposited at event location
  WR-5 Continues after beacon drop
  WR-6 BFS path avoids obstacles; handles dead-ends
"""

from __future__ import annotations

import heapq
import logging
import time
from collections import deque
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np

from beacon.schema import BeaconMessage
from beacon.store import BeaconStore
from writer_robot.grid_world import GridWorld, OBSTACLE, VISITED, BEACON_PLACED, FREE
from writer_robot.sensors import SensorArray

logger = logging.getLogger(__name__)


# ── Constants ─────────────────────────────────────────────────────────────────

CONFIDENCE_THRESHOLD = 0.65  # PRD WR-3: drop beacon only above this
BEACON_COOLDOWN_CELLS = 4    # minimum cells between consecutive beacon drops
STEP_DELAY = 0.12            # readable default pacing for demos using WriterRobot.run()
# In the deterministic USAR simulation, a beacon is created only after the
# robot has identified a labelled hotspot.  A confirmed trapped victim is
# therefore shown as life-critical, rather than being visually downgraded by
# the rolling sensor-noise value used to decide whether to create the beacon.
CONFIRMED_EVENT_SEVERITY_FLOORS = {
    "victim_detected": 0.95,
}


class RobotState(Enum):
    IDLE = auto()
    EXPLORING = auto()
    NAVIGATING = auto()
    BEACON_DROPPED = auto()
    COMPLETE = auto()
    FAILED = auto()
    KILLED = auto()


@dataclass
class Telemetry:
    robot_id: str
    state: RobotState
    row: int
    col: int
    heading_deg: float
    battery_pct: float
    cells_visited: int
    beacons_dropped: int
    exploration_pct: float
    current_step: int


# ── Writer Robot ──────────────────────────────────────────────────────────────

class WriterRobot:
    """
    Autonomous Writer Robot that explores a GridWorld, detects events via
    sensors, and deposits BeaconMessages into a shared BeaconStore.

    CONSTRAINT ENFORCEMENT:
      This class imports ONLY from beacon/ and writer_robot/.
      It has NO knowledge of outside_network/, command_post/, or executor_robot/.
      All inter-module communication flows through the shared BeaconStore.
    """

    def __init__(
        self,
        robot_id: str,
        grid: GridWorld,
        beacon_store: BeaconStore,
        start_row: int = 1,
        start_col: int = 1,
        battery_pct: float = 100.0,
        step_callback: Optional[Callable[["WriterRobot"], None]] = None,
        rng_seed: Optional[int] = None,
    ):
        self.robot_id = robot_id
        self.grid = grid
        self.beacon_store = beacon_store
        self.row = start_row
        self.col = start_col
        self.heading_deg = 0.0
        self.battery_pct = battery_pct
        self.state = RobotState.IDLE
        self.step_callback = step_callback
        self._rng = np.random.default_rng(rng_seed)

        self.sensors = SensorArray(rng_seed=rng_seed)
        self.beacons_dropped: List[BeaconMessage] = []
        self.path_history: List[Tuple[int, int]] = []
        self._step = 0
        self._last_beacon_step = -BEACON_COOLDOWN_CELLS - 1
        self._alive = True

        # Mark start cell visited
        self.grid.mark_visited(start_row, start_col)

    # ── Public API ────────────────────────────────────────────────────────────

    def kill(self) -> None:
        """Simulate robot failure (power loss, physical damage, etc.)."""
        logger.warning(f"[{self.robot_id}] KILLED at step {self._step}")
        self._alive = False
        self.state = RobotState.KILLED

    def is_alive(self) -> bool:
        return self._alive

    def run(self, max_steps: int = 500) -> None:
        """Run exploration loop until no frontiers remain or max_steps exceeded."""
        self.state = RobotState.EXPLORING
        logger.info(f"[{self.robot_id}] Starting exploration. Grid: {self.grid.width}×{self.grid.height}")

        while self._alive and self._step < max_steps:
            frontiers = self.grid.get_frontiers()
            if not frontiers:
                logger.info(f"[{self.robot_id}] No frontiers remain. Exploration complete.")
                self.state = RobotState.COMPLETE
                break

            # Choose nearest frontier (greedy)
            target = self._nearest_frontier(frontiers)
            if target is None:
                logger.info(f"[{self.robot_id}] No reachable frontier. Done.")
                self.state = RobotState.COMPLETE
                break

            # Navigate one step toward target
            moved = self._step_toward(target)
            if not moved:
                # Mark target as obstacle equivalent to skip it
                # (could be surrounded by freshly-visited cells)
                self.grid.grid[target[0], target[1]] = VISITED
                continue

            self._step += 1
            # Tuneable energy model: 0.05% per one-metre grid movement.
            self._drain_battery(0.05, "movement")

            # Scan sensors at new position
            self._scan_and_maybe_drop()

            if self.step_callback:
                self.step_callback(self)

            if STEP_DELAY > 0:
                time.sleep(STEP_DELAY)

        if self._alive and self.state != RobotState.COMPLETE:
            self.state = RobotState.COMPLETE
            logger.info(f"[{self.robot_id}] Max steps reached. Exploration ended.")

    def get_telemetry(self) -> Telemetry:
        return Telemetry(
            robot_id=self.robot_id,
            state=self.state,
            row=self.row,
            col=self.col,
            heading_deg=self.heading_deg,
            battery_pct=round(self.battery_pct, 1),
            cells_visited=int(np.sum(
                (self.grid.grid == VISITED) | (self.grid.grid == BEACON_PLACED)
            )),
            beacons_dropped=len(self.beacons_dropped),
            exploration_pct=self.grid.exploration_pct(),
            current_step=self._step,
        )

    # ── Internal: navigation ──────────────────────────────────────────────────

    def _nearest_frontier(self, frontiers: List[Tuple[int, int]]) -> Optional[Tuple[int, int]]:
        """Return the reachable frontier closest to current position by BFS distance."""
        # BFS from current position to find closest frontier
        dist: dict[Tuple[int, int], int] = {}
        queue: deque[Tuple[int, int]] = deque()
        queue.append((self.row, self.col))
        dist[(self.row, self.col)] = 0
        frontier_set = set(frontiers)

        while queue:
            r, c = queue.popleft()
            if (r, c) in frontier_set:
                return (r, c)
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nr, nc = r + dr, c + dc
                if (nr, nc) not in dist and self.grid.is_walkable(nr, nc):
                    dist[(nr, nc)] = dist[(r, c)] + 1
                    queue.append((nr, nc))
        return None

    def _bfs_path(self, target: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """BFS path from current position to target. Returns list of cells."""
        start = (self.row, self.col)
        if start == target:
            return [start]

        came_from: dict[Tuple[int, int], Optional[Tuple[int, int]]] = {start: None}
        queue: deque[Tuple[int, int]] = deque([start])

        while queue:
            cur = queue.popleft()
            if cur == target:
                # Reconstruct path
                path = []
                while cur is not None:
                    path.append(cur)
                    cur = came_from[cur]
                path.reverse()
                return path

            r, c = cur
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nb = (r + dr, c + dc)
                if nb not in came_from and self.grid.is_walkable(nb[0], nb[1]):
                    came_from[nb] = cur
                    queue.append(nb)

        return None  # no path found

    def _step_toward(self, target: Tuple[int, int]) -> bool:
        """Move one cell toward target. Returns True if moved."""
        path = self._bfs_path(target)
        if not path or len(path) < 2:
            return False

        next_cell = path[1]
        old_r, old_c = self.row, self.col
        self.row, self.col = next_cell

        # Update heading
        dr = self.row - old_r
        dc = self.col - old_c
        self.heading_deg = _heading_from_delta(dr, dc)

        self.grid.mark_visited(self.row, self.col)
        self.path_history.append((self.row, self.col))
        return True

    # ── Internal: sensing & beacon drop ──────────────────────────────────────

    def _scan_and_maybe_drop(self) -> None:
        """Scan sensors; drop beacon if fused confidence exceeds threshold."""
        event_prob = self.grid.event_probability_at(self.row, self.col)
        primary_evt = getattr(self.grid, "event_type_at", lambda r, c: None)(self.row, self.col)
        self.sensors.scan(event_prob, self.row, self.col, primary_evt)
        # Tuneable energy model: 0.02% per sensor sweep.
        self._drain_battery(0.02, "sensing")
        fused = self.sensors.fused()

        # Check all event types — drop at most one beacon per cooldown period
        cooldown_ok = (self._step - self._last_beacon_step) >= BEACON_COOLDOWN_CELLS

        if not cooldown_ok:
            return

        # Sort by fused confidence descending; drop for the highest if above threshold
        best_evt, best_conf = max(fused.items(), key=lambda kv: kv[1])

        if best_conf >= CONFIDENCE_THRESHOLD:
            # A victim beacon only exists after the fusion threshold is met.
            # Keep every such survivor observation consistently life-critical
            # in the mission view, including a later confirmation produced as
            # the robot moves across the same detection zone.
            best_conf = max(best_conf, CONFIRMED_EVENT_SEVERITY_FLOORS.get(best_evt, 0.0))
            self._drop_beacon(best_evt, best_conf)

    def _drop_beacon(self, event_type: str, confidence: float) -> None:
        """Create and register a beacon at the current position."""
        x, y = self.grid.cell_to_metres(self.row, self.col)
        beacon = BeaconMessage.create(
            writer_id=self.robot_id,
            event_type=event_type,
            severity=round(confidence, 4),
            x=x,
            y=y,
            z=0.0,
            heading_deg=self.heading_deg,
        )
        added = self.beacon_store.add(beacon)
        if added:
            # Tuneable energy model: 0.15% to encode and transmit a beacon.
            self._drain_battery(0.15, "beacon transmission")
            self.beacons_dropped.append(beacon)
            self.grid.mark_beacon(self.row, self.col)
            self._last_beacon_step = self._step
            self.state = RobotState.BEACON_DROPPED
            logger.info(
                f"[{self.robot_id}] Beacon dropped: {beacon.beacon_id} "
                f"| {event_type} sev={confidence:.2f} | pos=({x:.1f},{y:.1f})"
            )
            # Reset fusion for this event to avoid duplicate drops
            self.sensors.fusion.reset(event_type)

    def _drain_battery(self, amount: float, action: str) -> None:
        """Apply action-derived drain and announce the critical threshold once."""
        previous = self.battery_pct
        self.battery_pct = max(0.0, self.battery_pct - amount)
        if previous >= 20.0 and self.battery_pct < 20.0:
            logger.warning(f"[{self.robot_id}] Battery critical: {self.battery_pct:.0f}%")


# ── Helpers ───────────────────────────────────────────────────────────────────

def _heading_from_delta(dr: int, dc: int) -> float:
    """Convert grid movement delta to compass heading in degrees."""
    if dr == -1 and dc == 0:
        return 0.0    # North
    if dr == 0 and dc == 1:
        return 90.0   # East
    if dr == 1 and dc == 0:
        return 180.0  # South
    if dr == 0 and dc == -1:
        return 270.0  # West
    return 0.0
