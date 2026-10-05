"""
writer_robot/grid_world.py
2D occupancy grid representing a GPS-denied USAR (Urban Search & Rescue)
environment. Provides frontier detection for autonomous exploration.

Cell values:
  0 = free (unexplored)
  1 = obstacle (wall / rubble)
  2 = visited (explored by robot)
  3 = beacon dropped here (sub-type of visited)
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field
from typing import List, Tuple

import numpy as np


CellState = int
FREE = 0
OBSTACLE = 1
VISITED = 2
BEACON_PLACED = 3


@dataclass
class GridWorld:
    """
    USAR building floor-plan represented as a 2D numpy grid.
    Origin (0,0) is top-left; x → right (East), y ↓ (South).
    Cell size: 1 metre × 1 metre.
    """

    width: int
    height: int
    grid: np.ndarray = field(init=False)
    # Event-probability map: per-cell probability of hosting an event
    event_prob: np.ndarray = field(init=False)

    def __post_init__(self):
        self.grid = np.zeros((self.height, self.width), dtype=np.int8)
        self.event_prob = np.zeros((self.height, self.width), dtype=np.float32)
        self._generate_usar_layout()

    # ── Layout generation ─────────────────────────────────────────────────────

    def _generate_usar_layout(self) -> None:
        """
        Procedurally generate a USAR building layout:
        - Outer walls
        - Interior rooms connected by corridors
        - Rubble patches (clusters of obstacles)
        - High event-probability zones (victim hotspots, fire zones)
        """
        rng = np.random.default_rng(42)  # fixed seed for reproducibility

        # Outer walls
        self.grid[0, :] = OBSTACLE
        self.grid[-1, :] = OBSTACLE
        self.grid[:, 0] = OBSTACLE
        self.grid[:, -1] = OBSTACLE

        # Interior walls creating rooms
        room_walls = [
            # (row_start, row_end, col_start, col_end) — partial walls with gaps
            (5, 15, 10, 11),   # vertical wall, room 1 | corridor
            (5, 6, 10, 25),    # horizontal wall, top partition
            (14, 15, 10, 25),  # horizontal wall, bottom partition
            (5, 15, 24, 25),   # vertical wall, corridor | room 2
            (5, 15, 35, 36),   # vertical wall, right section
            (15, 25, 15, 16),  # lower level divider
        ]
        for rs, re, cs, ce in room_walls:
            rs, re = min(rs, self.height - 1), min(re, self.height - 1)
            cs, ce = min(cs, self.width - 1), min(ce, self.width - 1)
            self.grid[rs:re, cs:ce] = OBSTACLE

        # Doorway gaps (clear single cells in walls)
        doorways = [
            (9, 10), (9, 24), (19, 15), (9, 35), (4, 17)
        ]
        for r, c in doorways:
            if 0 < r < self.height - 1 and 0 < c < self.width - 1:
                self.grid[r, c] = FREE

        # Rubble patches (clusters of obstacles)
        rubble_centers = [(3, 5), (8, 30), (18, 8), (20, 28), (12, 18)]
        for cr, cc in rubble_centers:
            size = rng.integers(2, 5)
            for dr in range(-size, size + 1):
                for dc in range(-size, size + 1):
                    r, c = cr + dr, cc + dc
                    if 0 < r < self.height - 1 and 0 < c < self.width - 1:
                        if rng.random() < 0.6:
                            self.grid[r, c] = OBSTACLE

        # Event probability zones (hot spots for victims/hazards)
        self.event_type_grid = {}

        # 1. Victim zones (trapped survivors in interior corners / void spaces)
        victim_zones = [(7, 6), (7, 13), (18, 5)]
        for zr, zc in victim_zones:
            for dr in range(-2, 3):
                for dc in range(-2, 3):
                    r, c = zr + dr, zc + dc
                    if 0 <= r < self.height and 0 <= c < self.width:
                        if self.grid[r, c] != OBSTACLE:
                            dist = max(abs(dr), abs(dc))
                            prob = max(0.0, 0.95 / (dist * 0.5 + 1))
                            if prob > self.event_prob[r, c]:
                                self.event_prob[r, c] = prob
                                self.event_type_grid[(r, c)] = "victim_detected"

        # 2. Fire zones (active thermal burn / intense heat)
        fire_zones = [(4, 3), (11, 28)]
        for zr, zc in fire_zones:
            for dr in range(-2, 3):
                for dc in range(-2, 3):
                    r, c = zr + dr, zc + dc
                    if 0 <= r < self.height and 0 <= c < self.width:
                        if self.grid[r, c] != OBSTACLE:
                            dist = (dr ** 2 + dc ** 2) ** 0.5
                            prob = max(0.0, 0.92 / (dist * 0.5 + 1))
                            if prob > self.event_prob[r, c]:
                                self.event_prob[r, c] = prob
                                self.event_type_grid[(r, c)] = "fire_detected"

    # ── Coordinate helpers ────────────────────────────────────────────────────

    def cell_to_metres(self, row: int, col: int) -> Tuple[float, float]:
        """Convert grid cell (row, col) → local (x, y) in metres."""
        return float(col), float(row)  # x=East(col), y=South(row)

    def metres_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        """Convert local metres (x, y) → grid cell (row, col)."""
        return int(round(y)), int(round(x))

    def is_walkable(self, row: int, col: int) -> bool:
        if row < 0 or row >= self.height or col < 0 or col >= self.width:
            return False
        return self.grid[row, col] != OBSTACLE

    # ── Frontier detection ────────────────────────────────────────────────────

    def get_frontiers(self) -> List[Tuple[int, int]]:
        """
        Return list of frontier cells: FREE cells that are adjacent (4-connected)
        to at least one VISITED cell. Sorted by distance to most-visited region.
        """
        frontiers = []
        for r in range(1, self.height - 1):
            for c in range(1, self.width - 1):
                if self.grid[r, c] != FREE:
                    continue
                # Check 4-connected neighbours for visited cells
                for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nr, nc = r + dr, c + dc
                    if self.grid[nr, nc] in (VISITED, BEACON_PLACED):
                        frontiers.append((r, c))
                        break
        return frontiers

    def mark_visited(self, row: int, col: int) -> None:
        if self.grid[row, col] == FREE:
            self.grid[row, col] = VISITED

    def mark_beacon(self, row: int, col: int) -> None:
        self.grid[row, col] = BEACON_PLACED

    def event_probability_at(self, row: int, col: int) -> float:
        if 0 <= row < self.height and 0 <= col < self.width:
            return float(self.event_prob[row, col])
        return 0.0

    def event_type_at(self, row: int, col: int) -> Optional[str]:
        if hasattr(self, "event_type_grid"):
            return self.event_type_grid.get((row, col))
        return None

    # ── Stats ─────────────────────────────────────────────────────────────────

    def exploration_pct(self) -> float:
        total_free = np.sum(self.grid != OBSTACLE)
        visited = np.sum((self.grid == VISITED) | (self.grid == BEACON_PLACED))
        if total_free == 0:
            return 100.0
        return round(100.0 * visited / total_free, 1)

    def ascii_render(self) -> str:
        """Compact ASCII map for terminal display."""
        symbols = {FREE: "·", OBSTACLE: "█", VISITED: "░", BEACON_PLACED: "B"}
        lines = []
        for row in self.grid:
            lines.append("".join(symbols.get(int(c), "?") for c in row))
        return "\n".join(lines)
