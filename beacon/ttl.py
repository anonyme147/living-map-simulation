"""
beacon/ttl.py
TTL (Time-To-Live) and aging logic for beacon messages.
"""

from __future__ import annotations

from datetime import datetime, timezone
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from beacon.schema import BeaconMessage


# RCAMP-inspired relevance model: rapidly changing hazards lose operational
# value sooner, while victim observations remain useful longer.
EVENT_TYPE_DEFAULT_TTLS = {
    "fire_detected": 900,
    "victim_detected": 5_400,
    # Direct Executor confirmation is useful for the full mission/reporting window.
    "mission_completed": 7_200,
}


def default_ttl_for_event(event_type: str) -> int:
    """Return the event-specific TTL used when a beacon is first created."""
    return EVENT_TYPE_DEFAULT_TTLS[event_type]


def _now_utc() -> datetime:
    return datetime.now(timezone.utc)


def _beacon_born(beacon: "BeaconMessage") -> datetime:
    """Parse the beacon's ISO8601 timestamp."""
    ts = beacon.timestamp
    # Handle both offset-aware and naive timestamps
    dt = datetime.fromisoformat(ts)
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)
    return dt


def age_seconds(beacon: "BeaconMessage") -> float:
    """Seconds elapsed since beacon was created."""
    born = _beacon_born(beacon)
    elapsed = (_now_utc() - born).total_seconds()
    return max(0.0, elapsed)


def remaining_ttl(beacon: "BeaconMessage") -> float:
    """Seconds remaining before this beacon expires. Negative means expired."""
    return beacon.ttl_seconds - age_seconds(beacon)


def is_expired(beacon: "BeaconMessage") -> bool:
    """Return True if the beacon has exceeded its TTL."""
    return remaining_ttl(beacon) <= 0


def ttl_fraction(beacon: "BeaconMessage") -> float:
    """
    Fraction of TTL remaining (1.0 = fresh, 0.0 = just expired, negative = past).
    Clamped to [0.0, 1.0].
    """
    if beacon.ttl_seconds <= 0:
        return 0.0
    fraction = remaining_ttl(beacon) / beacon.ttl_seconds
    return max(0.0, min(1.0, fraction))


def effective_priority(beacon: "BeaconMessage") -> float:
    """
    Combined priority score for mission planning.
    Formula: severity * ttl_fraction
    - High severity + fresh beacon → highest priority
    - High severity + stale beacon → deprioritized
    - Expired beacons → 0.0 priority (will be filtered)

    Score range: 0.0–1.0
    """
    if is_expired(beacon):
        return 0.0
    return beacon.event.severity * ttl_fraction(beacon)


def summary(beacon: "BeaconMessage") -> dict:
    """Human-readable TTL summary dict."""
    return {
        "beacon_id": beacon.beacon_id,
        "age_s": round(age_seconds(beacon), 1),
        "ttl_s": beacon.ttl_seconds,
        "remaining_s": round(remaining_ttl(beacon), 1),
        "expired": is_expired(beacon),
        "priority": round(effective_priority(beacon), 4),
    }
