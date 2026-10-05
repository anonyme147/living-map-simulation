"""
beacon/store.py
In-memory beacon store with deduplication.
Accessible to both Writer Robot (write) and Outside Network Area (read).
Executor Robot may also read directly when inside range.
"""

from __future__ import annotations

import threading
from typing import Dict, List, Optional

from beacon.schema import BeaconMessage
from beacon import ttl as ttl_mod


class BeaconStore:
    """
    Thread-safe in-memory store for beacon messages.

    Design note (PRD BC-5):
      - Supports multiple readers (ONA and Executor) without duplication.
      - Deduplication is by beacon_id (first write wins).
      - Does NOT enforce comms constraints — those are enforced by module
        boundaries (only outside_network/ and executor_robot/ import from here).
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._store: Dict[str, BeaconMessage] = {}
        self._read_by_ona: set[str] = set()  # beacon_ids already sent to ONA

    # ── Write ─────────────────────────────────────────────────────────────────

    def add(self, beacon: BeaconMessage) -> bool:
        """
        Add a beacon. Returns True if added, False if duplicate (beacon_id clash).
        """
        if not beacon.validate_crc():
            raise ValueError(f"Rejected beacon {beacon.beacon_id}: CRC validation failed")
        with self._lock:
            if beacon.beacon_id in self._store:
                return False
            self._store[beacon.beacon_id] = beacon
            return True

    # ── Read ──────────────────────────────────────────────────────────────────

    def get_all(self) -> List[BeaconMessage]:
        """Return all beacons (active + expired), in insertion order."""
        with self._lock:
            return list(self._store.values())

    def get_active(self) -> List[BeaconMessage]:
        """Return only non-expired beacons."""
        with self._lock:
            return [b for b in self._store.values() if not ttl_mod.is_expired(b)]

    def get_expired(self) -> List[BeaconMessage]:
        """Return only expired beacons."""
        with self._lock:
            return [b for b in self._store.values() if ttl_mod.is_expired(b)]

    def get_unsent(self) -> List[BeaconMessage]:
        """Return beacons not yet picked up by ONA."""
        with self._lock:
            return [
                b for bid, b in self._store.items()
                if bid not in self._read_by_ona
            ]

    def get_by_id(self, beacon_id: str) -> Optional[BeaconMessage]:
        with self._lock:
            return self._store.get(beacon_id)

    def is_expired(self, beacon_id: str) -> bool:
        """Check if a given beacon_id is expired or missing."""
        with self._lock:
            b = self._store.get(beacon_id)
            if b is None:
                return False
            return ttl_mod.is_expired(b) or b.ttl_seconds <= 0

    def get_in_range(self, x: float, y: float, radius: float) -> List[BeaconMessage]:
        """
        Return active beacons within `radius` metres of (x, y).
        Used by Executor Robot for direct beacon reading on entry.
        """
        result = []
        with self._lock:
            for b in self._store.values():
                if ttl_mod.is_expired(b):
                    continue
                dx = b.position_local.x - x
                dy = b.position_local.y - y
                dist = (dx ** 2 + dy ** 2) ** 0.5
                if dist <= radius:
                    result.append(b)
        return result

    # ── ONA handoff ───────────────────────────────────────────────────────────

    def mark_sent_to_ona(self, beacon_id: str) -> None:
        """Called by Outside Network receiver after picking up a beacon."""
        with self._lock:
            self._read_by_ona.add(beacon_id)

    def expire_all(self) -> int:
        """Expire all beacons in store by setting ttl_seconds to 0."""
        with self._lock:
            count = 0
            for b in self._store.values():
                b.ttl_seconds = 0
                b.refresh_crc()
                count += 1
            return count

    # ── Stats ─────────────────────────────────────────────────────────────────

    def count(self) -> dict:
        with self._lock:
            total = len(self._store)
            active = sum(1 for b in self._store.values() if not ttl_mod.is_expired(b))
            unsent = sum(1 for bid in self._store if bid not in self._read_by_ona)
            return {"total": total, "active": active, "expired": total - active, "unsent": unsent}

    def __len__(self) -> int:
        with self._lock:
            return len(self._store)
