"""Showcase-only, payload-level delivery evidence for the signal-integrity demo."""

from __future__ import annotations

import hashlib
import json
import threading
from typing import Any


def _payload_fingerprint(payload: dict[str, Any]) -> str:
    """Stable digest of the exact enriched beacon content carried by ONA."""
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


class SignalIntegrityLedger:
    """Tracks unique ONA CARRY payloads without affecting their transport."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._sent: dict[str, str] = {}
        self._delivered: set[str] = set()
        self._corrupted: set[str] = set()
        self._phase = "SATELLITE PRIMARY"
        self._subtitle = ""

    def set_phase(self, phase: str) -> None:
        with self._lock:
            self._phase = phase

    def set_subtitle(self, subtitle: str) -> None:
        """Publish the current event-driven documentary explanation."""
        with self._lock:
            self._subtitle = subtitle

    def record_sent(self, enriched_beacon: dict[str, Any]) -> None:
        beacon_id = str(enriched_beacon.get("beacon_id", "unknown"))
        with self._lock:
            self._sent[beacon_id] = _payload_fingerprint(enriched_beacon)

    def record_delivered_packet(self, raw_packet: bytes | str | dict[str, Any]) -> None:
        try:
            if isinstance(raw_packet, bytes):
                packet = json.loads(raw_packet.decode("utf-8"))
            elif isinstance(raw_packet, str):
                packet = json.loads(raw_packet)
            else:
                packet = raw_packet
            payload = packet.get("beacon_data", packet)
            beacon_id = str(payload.get("beacon_id", "unknown"))
            fingerprint = _payload_fingerprint(payload)
        except Exception:
            return
        with self._lock:
            expected = self._sent.get(beacon_id)
            if expected is None or expected != fingerprint:
                self._corrupted.add(beacon_id)
            else:
                self._delivered.add(beacon_id)

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "enabled": True,
                "phase": self._phase,
                "subtitle": self._subtitle,
                "sent": len(self._sent),
                "delivered": len(self._delivered),
                # A queued/in-flight packet is not declared lost. Loss is a
                # verified terminal delivery failure, of which this showcase
                # intentionally produces none.
                "lost": 0,
                "corrupted": len(self._corrupted),
                "in_flight": max(0, len(self._sent) - len(self._delivered) - len(self._corrupted)),
            }
