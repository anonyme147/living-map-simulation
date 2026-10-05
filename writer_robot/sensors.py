"""
writer_robot/sensors.py
Simulated sensor models for USAR event detection.

Supports 2 event types (PRD WR-2: ≥2 required):
  - victim_detected     (acoustic / thermal sensor)
  - fire_detected       (thermal / smoke sensor)

Each sensor returns a (event_type, raw_confidence) tuple.
The Writer Robot fuses the last N readings before deciding to drop a beacon.
"""

from __future__ import annotations

import random
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, List, Optional, Tuple

import numpy as np

from beacon.schema import EVENT_TYPES


# ── Sensor configuration ─────────────────────────────────────────────────────

@dataclass
class SensorConfig:
    noise_std: float = 0.08          # Gaussian noise σ on confidence readings
    false_positive_rate: float = 0.05  # P(spurious detection)
    sensor_range: float = 3.0        # metres — how close robot must be to detect
    fusion_window: int = 3           # readings to average before threshold check


SENSOR_CONFIGS: dict[str, SensorConfig] = {
    "victim_detected":   SensorConfig(noise_std=0.10, false_positive_rate=0.04, sensor_range=4.0),
    "fire_detected":     SensorConfig(noise_std=0.06, false_positive_rate=0.06, sensor_range=5.0),
}


# ── Reading accumulator (sensor fusion) ──────────────────────────────────────

class SensorFusion:
    """
    Rolling average of last `window` sensor readings per event type.
    PRD WR-3: Drop beacon only when fused confidence > threshold.
    """

    def __init__(self, window: int = 3):
        self.window = window
        self._readings: dict[str, Deque[float]] = {
            evt: deque(maxlen=window) for evt in EVENT_TYPES
        }

    def push(self, event_type: str, confidence: float) -> None:
        self._readings[event_type].append(confidence)

    def fused_confidence(self, event_type: str) -> float:
        readings = self._readings[event_type]
        if not readings:
            return 0.0
        # Blend peak reading with rolling mean to detect hazardous events while filtering spurious noise
        return float(0.65 * max(readings) + 0.35 * np.mean(readings))

    def reset(self, event_type: str) -> None:
        self._readings[event_type].clear()

    def all_fused(self) -> dict[str, float]:
        return {evt: self.fused_confidence(evt) for evt in EVENT_TYPES}


# ── Sensor model ─────────────────────────────────────────────────────────────

class SensorArray:
    """
    Multi-sensor array mounted on the Writer Robot.
    Simulates noisy readings based on the event probability field of the grid.
    """

    def __init__(self, rng_seed: Optional[int] = None):
        self._rng = np.random.default_rng(rng_seed)
        self.fusion = SensorFusion(window=3)
        self._scan_count = 0

    def scan(
        self,
        event_prob: float,
        grid_row: int,
        grid_col: int,
        primary_event: Optional[str] = None,
    ) -> List[Tuple[str, float]]:
        """
        Perform one scan at the current position.
        `event_prob` is the cell's base event probability from GridWorld.
        `primary_event` is the optional event type associated with the hotspot.

        Returns list of (event_type, noisy_confidence) for all event types.
        """
        self._scan_count += 1
        results = []

        for evt in EVENT_TYPES:
            cfg = SENSOR_CONFIGS[evt]

            # Modulate base prob by event type affinity
            affinity = _event_affinity(evt, event_prob, primary_event)
            raw_signal = affinity + self._rng.normal(0, cfg.noise_std)
            raw_signal = float(np.clip(raw_signal, 0.0, 1.0))

            # Inject random false positives
            if raw_signal < 0.1 and self._rng.random() < cfg.false_positive_rate:
                raw_signal = self._rng.uniform(0.1, 0.25)  # weak false positive

            self.fusion.push(evt, raw_signal)
            results.append((evt, raw_signal))

        return results

    def fused(self) -> dict[str, float]:
        return self.fusion.all_fused()

    @property
    def scan_count(self) -> int:
        return self._scan_count


# ── Event affinity model ──────────────────────────────────────────────────────

def _event_affinity(event_type: str, base_prob: float, primary_event: Optional[str] = None) -> float:
    """
    Map base_prob to per-event-type signal strength.
    When a primary event is assigned to a hotspot, that sensor registers a strong signal,
    while other sensors observe minor cross-talk or background ambient noise.
    """
    if primary_event is not None:
        if event_type == primary_event:
            return min(1.0, base_prob * 1.12)
        else:
            return min(0.20, base_prob * 0.12)

    if event_type == "victim_detected":
        return min(1.0, base_prob * 0.95)
    elif event_type == "fire_detected":
        return min(1.0, base_prob * 0.90)

    return base_prob
