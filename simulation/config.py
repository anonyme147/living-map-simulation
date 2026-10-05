"""
simulation/config.py
Tunable configuration parameters for The Living Map simulation and demonstration.
All system thresholds, radio characteristics, and coordinate anchors are declared here.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path


# ── Directory Layout ─────────────────────────────────────────────────────────

ROOT_DIR = Path(__file__).resolve().parent.parent
SIMULATION_DIR = ROOT_DIR / "simulation"
LOGS_DIR = SIMULATION_DIR / "logs"
LOGS_DIR.mkdir(parents=True, exist_ok=True)


# ── System Environment ───────────────────────────────────────────────────────

ENVIRONMENT_NAME = "Urban Search & Rescue (USAR)"
ENVIRONMENT_DESCRIPTION = (
    "Collapsed commercial structure with reinforced concrete debris, interior partitions, "
    "doorway thresholds, fire hotspots, "
    "and trapped survivors."
)


# ── Demonstration Geographic Coordinates ──────────────────────────────────────
# Compact urban operating area used only for dashboard visualization.
BUILDING_SITE_LAT: float = 36.8189
BUILDING_SITE_LON: float = 10.1658
BUILDING_SITE_ALT: float = 10.0

# Incident Command Post (separate demonstration location)
COMMAND_POST_LAT: float = 36.8525
COMMAND_POST_LON: float = 10.2085
COMMAND_POST_ALT: float = 5.0

# Staging Area Coordinates for Robots (clearly separated for multi-rover scenarios)
WRITER_START_LAT: float = 36.8189
WRITER_START_LON: float = 10.1658
EXECUTOR_START_LAT: float = 36.8480
EXECUTOR_START_LON: float = 10.1620

# ── GPS Anchor & Georeferencing ──────────────────────────────────────────────

@dataclass
class AnchorConfig:
    """
    Geodetic origin mapping local Cartesian frame (0, 0, 0) to true GPS.
    Default: configured demonstration map anchor.
    """
    origin_lat: float = BUILDING_SITE_LAT
    origin_lon: float = BUILDING_SITE_LON
    origin_alt: float = BUILDING_SITE_ALT
    bearing_deg: float = 0.0  # Local +X axis aligned with True East


# ── Drift Correction Model ───────────────────────────────────────────────────

@dataclass
class DriftConfig:
    """
    Linear odometry drift model simulating uncorrected dead reckoning.
    Real emergency robots accumulate drift that must be compensated at the ONA boundary.
    """
    drift_rate_x: float = 0.02   # 2 cm error per 1 metre traveled
    drift_rate_y: float = 0.015  # 1.5 cm error per 1 metre traveled
    correction_interval: int = 1  # Re-anchor and correct every beacon at ONA


# ── Satellite Link Model ────────────────────────────────────────────────────

@dataclass
class SatelliteLinkConfig:
    """Placeholder satellite assumptions for the simulated ONA↔Command Post link."""
    one_way_latency_ms: int = 800   # Moderate pacing for a clearly visible satellite transfer
    nominal_packet_loss: float = 0.02  # Placeholder: weather/pass-handoff dropout
    failure_packet_loss: float = 1.00  # 100% loss during an injected disruption
    pass_window_available: bool = True  # Placeholder: service window is nominally open
    max_retry_attempts: int = 8
    base_retry_delay_s: float = 1.0
    max_retry_delay_s: float = 30.0


# ── Grid World & USAR Layout ─────────────────────────────────────────────────

@dataclass
class GridConfig:
    width: int = 40       # 40 metres East
    height: int = 30      # 30 metres South
    cell_size_m: float = 1.0
    random_seed: int = 42


# ── Writer Robot Configuration ───────────────────────────────────────────────

@dataclass
class WriterConfig:
    robot_id: str = "WRITER-1"
    start_row: int = 1
    start_col: int = 1
    battery_pct: float = 100.0
    confidence_threshold: float = 0.65  # WR-3 threshold: drop only significant events
    beacon_cooldown_cells: int = 4     # Min cells between consecutive beacon drops
    default_ttl_seconds: int = 3600    # 1 hour beacon freshness
    step_delay_s: float = 0.12         # ~120ms per step → readable exploration pacing


# ── Executor Robot Configuration ─────────────────────────────────────────────

@dataclass
class ExecutorConfig:
    robot_id: str = "EXECUTOR-1"
    start_lat: float = EXECUTOR_START_LAT
    start_lon: float = EXECUTOR_START_LON
    beacon_read_radius_m: float = 5.0  # EX-3 direct beacon scanning radius
    arrival_threshold_m: float = 2.0   # Distance to consider target reached
    nav_step_delay_s: float = 0.14     # ~140ms per long nav step → readable map movement


# ── Web Dashboard Configuration ──────────────────────────────────────────────

@dataclass
class DashboardConfig:
    host: str = "127.0.0.1"
    port: int = 5000
    poll_interval_s: int = 3
    command_post_lat: float = COMMAND_POST_LAT
    command_post_lon: float = COMMAND_POST_LON
    building_site_lat: float = BUILDING_SITE_LAT
    building_site_lon: float = BUILDING_SITE_LON
