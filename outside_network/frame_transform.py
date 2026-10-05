"""
outside_network/frame_transform.py
Local-to-GPS coordinate frame translation with drift correction.

DESIGN:
  The Writer Robot operates in a local Cartesian frame (metres, origin at
  robot start). The Outside Network Area holds the only anchor that maps
  this local frame to real-world GPS.

  Transform (simple affine):
    lat = origin_lat + (y_local / METRES_PER_DEG_LAT)
    lon = origin_lon + (x_local / metres_per_deg_lon(origin_lat))

DRIFT CORRECTION (PRD ON-2, Section 10 item 1):
  Real robots accumulate odometry drift over time. We model this as a
  linear drift in x and y that grows with path length. Correction is applied
  for every beacon by re-anchoring to a known reference
  beacon position (simulated ground-truth check-in).

  Documented drift model:
    drift_x(t) = drift_rate_x * path_length_metres
    drift_y(t) = drift_rate_y * path_length_metres
  Corrected position:
    x_corrected = x_raw - drift_x
    y_corrected = y_raw - drift_y

  In a real system, this correction would come from:
    - Loop-closure (if SLAM is available)
    - A GPS fix at the zone boundary (anchor point)
    - Known landmark positions verified by the ONA

GPS anchor: public, non-project-specific demonstration location.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from beacon.schema import BeaconMessage, PositionLocal
from simulation.config import BUILDING_SITE_ALT, BUILDING_SITE_LAT, BUILDING_SITE_LON

logger = logging.getLogger(__name__)

# ── Constants ─────────────────────────────────────────────────────────────────

METRES_PER_DEG_LAT = 111_320.0   # ~constant globally

def metres_per_deg_lon(lat_deg: float) -> float:
    return METRES_PER_DEG_LAT * math.cos(math.radians(lat_deg))


# ── GPS anchor ────────────────────────────────────────────────────────────────

@dataclass
class GPSAnchor:
    """
    Maps the local-frame origin (0,0,0) to a real GPS coordinate.
    Also holds the rotation angle (bearing of local +X axis from true North).
    """
    origin_lat: float = BUILDING_SITE_LAT
    origin_lon: float = BUILDING_SITE_LON
    origin_alt: float = BUILDING_SITE_ALT
    bearing_deg: float = 0.0       # local +X axis vs true East (0 = aligned)


# ── Drift model ───────────────────────────────────────────────────────────────

@dataclass
class DriftModel:
    """
    Linear drift model. In a real system this would be replaced by
    SLAM loop-closure or GPS-boundary re-anchoring.

    drift_rate: metres of error per 100 metres of path length
    correction_interval: apply correction every beacon received
    """
    drift_rate_x: float = 0.02   # 2 cm error per metre travelled
    drift_rate_y: float = 0.015  # 1.5 cm error per metre travelled
    correction_interval: int = 1  # correct every beacon

    # Internal state
    _cumulative_path_m: float = field(default=0.0, init=False, repr=False)
    _beacons_since_correction: int = field(default=0, init=False, repr=False)
    _applied_corrections: int = field(default=0, init=False, repr=False)
    _total_adjustment_m: float = field(default=0.0, init=False, repr=False)

    def accumulate_path(self, delta_metres: float) -> None:
        self._cumulative_path_m += abs(delta_metres)

    def current_drift(self) -> Tuple[float, float]:
        dx = self.drift_rate_x * self._cumulative_path_m
        dy = self.drift_rate_y * self._cumulative_path_m
        return dx, dy

    def apply_correction(self, x_raw: float, y_raw: float) -> Tuple[float, float, bool]:
        """
        Returns (x_corrected, y_corrected, was_corrected).
        Correction is applied to every beacon when the configured interval is 1.
        """
        self._beacons_since_correction += 1
        corrected = False

        if self._beacons_since_correction >= self.correction_interval:
            dx, dy = self.current_drift()
            x_corrected = x_raw - dx
            y_corrected = y_raw - dy
            # Reset accumulated path (re-anchor simulated)
            self._cumulative_path_m = 0.0
            self._beacons_since_correction = 0
            self._applied_corrections += 1
            adj = math.sqrt(dx ** 2 + dy ** 2)
            self._total_adjustment_m += adj
            corrected = True
            logger.info(
                f"[DriftCorrection] Applied #{self._applied_corrections}: "
                f"Δx={dx:.3f}m Δy={dy:.3f}m → ({x_corrected:.2f}, {y_corrected:.2f})"
            )
            return x_corrected, y_corrected, True

        return x_raw, y_raw, False

    def corrections_applied(self) -> int:
        return self._applied_corrections

    def total_adjustment_m(self) -> float:
        return round(self._total_adjustment_m, 4)


# ── Frame Transform ────────────────────────────────────────────────────────────

@dataclass
class GPSCoordinate:
    lat: float
    lon: float
    alt: float
    x_corrected: float  # corrected local x (for audit trail)
    y_corrected: float  # corrected local y (for audit trail)
    drift_corrected: bool


class LocalToGPS:
    """
    Converts robot-local (x, y, z) positions to GPS (lat, lon, alt).
    Applies drift correction per DriftModel.

    Usage:
        transformer = LocalToGPS(anchor, drift_model)
        gps = transformer.transform(position_local)
    """

    def __init__(
        self,
        anchor: Optional[GPSAnchor] = None,
        drift_model: Optional[DriftModel] = None,
    ):
        self.anchor = anchor or GPSAnchor()
        self.drift = drift_model or DriftModel()
        self._transform_count = 0
        self._last_local: Optional[Tuple[float, float]] = None

    def transform(self, position_local: PositionLocal) -> GPSCoordinate:
        """
        Transform a local position to GPS, with drift correction applied.
        """
        x_raw, y_raw = position_local.x, position_local.y

        # Accumulate path length for drift model
        if self._last_local is not None:
            dx_step = x_raw - self._last_local[0]
            dy_step = y_raw - self._last_local[1]
            path_delta = math.sqrt(dx_step ** 2 + dy_step ** 2)
            self.drift.accumulate_path(path_delta)
        self._last_local = (x_raw, y_raw)

        # Apply drift correction
        x_corr, y_corr, was_corrected = self.drift.apply_correction(x_raw, y_raw)

        # Affine local→GPS transform
        # y_local is South-increasing (grid row), convert to North-increasing for lat
        lat = self.anchor.origin_lat + (y_corr / METRES_PER_DEG_LAT)
        lon = self.anchor.origin_lon + (x_corr / metres_per_deg_lon(self.anchor.origin_lat))
        alt = self.anchor.origin_alt + position_local.z

        self._transform_count += 1
        coord = GPSCoordinate(
            lat=round(lat, 7),
            lon=round(lon, 7),
            alt=round(alt, 2),
            x_corrected=round(x_corr, 3),
            y_corrected=round(y_corr, 3),
            drift_corrected=was_corrected,
        )
        logger.debug(
            f"[Transform #{self._transform_count}] local({x_raw:.2f},{y_raw:.2f}) → "
            f"GPS({coord.lat},{coord.lon}) drift_corrected={was_corrected}"
        )
        return coord

    def transform_beacon(self, beacon: BeaconMessage) -> dict:
        """
        Convenience: transform a beacon's position and return an enriched dict.
        """
        gps = self.transform(beacon.position_local)
        result = beacon.to_dict()
        result["gps"] = {
            "lat": gps.lat,
            "lon": gps.lon,
            "alt": gps.alt,
            "drift_corrected": gps.drift_corrected,
        }
        return result

    @property
    def transform_count(self) -> int:
        return self._transform_count
