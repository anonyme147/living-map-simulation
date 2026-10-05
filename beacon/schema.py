"""
beacon/schema.py
Beacon message schema — exact match to PRD Section 9 / prompt spec.
Includes JSON serialization and a compact binary pack for LoRa constraint.

Binary layout (target ≤ 64 bytes):
  4B  beacon_id hash (uint32)
  4B  writer_id hash (uint32)
  1B  event type index (uint8)
  4B  severity (float32)
  4B  x (float32)
  4B  y (float32)
  2B  z (int16, mm resolution)
  2B  heading_deg (uint16, 0–36000 for 0.01° resolution)
  4B  timestamp (uint32 unix seconds)
  4B  ttl_seconds (uint32)
  4B  crc32 (uint32, canonical payload integrity check)
  --- Total: 42 bytes core payload ---
"""

from __future__ import annotations

import json
import struct
import uuid
import zlib
from dataclasses import dataclass, field, asdict
from datetime import datetime, timezone
from typing import Optional


# ── Sensor-detectable event types (USAR environment) ─────────────────────────
# Writer sensors import this list, so status-only beacon types must not be added
# here: they are confirmed by a robot action rather than sensed from the scene.
EVENT_TYPES = [
    "victim_detected",
    "fire_detected",
]

# ── Additional supported beacon status types ──────────────────────────────────
STATUS_EVENT_TYPES = [
    "mission_completed",
]

# All beacon payload types are serializable and valid in EventPayload.
SUPPORTED_EVENT_TYPES = EVENT_TYPES + STATUS_EVENT_TYPES
_EVENT_INDEX = {evt: idx for idx, evt in enumerate(SUPPORTED_EVENT_TYPES)}
_INDEX_EVENT = {idx: evt for idx, evt in enumerate(SUPPORTED_EVENT_TYPES)}


@dataclass
class EventPayload:
    type: str       # one of SUPPORTED_EVENT_TYPES
    severity: float  # 0.0–1.0

    def __post_init__(self):
        if self.type not in _EVENT_INDEX:
            raise ValueError(f"Unknown event type '{self.type}'. Valid: {SUPPORTED_EVENT_TYPES}")
        self.severity = max(0.0, min(1.0, float(self.severity)))


@dataclass
class PositionLocal:
    x: float          # metres east of origin
    y: float          # metres north of origin
    z: float          # metres above floor
    heading_deg: float  # 0–360, robot heading


@dataclass
class BeaconMessage:
    beacon_id: str
    writer_id: str
    event: EventPayload
    position_local: PositionLocal
    timestamp: str       # ISO8601
    ttl_seconds: int
    crc32: Optional[int] = None  # CRC-32 over the canonical beacon payload

    def __post_init__(self) -> None:
        """Give every newly-created beacon a deterministic integrity checksum."""
        if self.crc32 is None:
            self.refresh_crc()
        else:
            self.crc32 = int(self.crc32) & 0xFFFFFFFF

    def _integrity_payload(self) -> dict:
        """Stable payload representation covered by the CRC (excluding the CRC itself)."""
        return {
            "beacon_id": self.beacon_id,
            "writer_id": self.writer_id,
            "event": {"type": self.event.type, "severity": round(self.event.severity, 4)},
            "position_local": {
                "x": self.position_local.x,
                "y": self.position_local.y,
                "z": self.position_local.z,
                "heading_deg": self.position_local.heading_deg,
            },
            "timestamp": self.timestamp,
            "ttl_seconds": self.ttl_seconds,
        }

    def compute_crc32(self) -> int:
        """Calculate CRC-32 from canonical UTF-8 beacon fields."""
        encoded = json.dumps(
            self._integrity_payload(), sort_keys=True, separators=(",", ":"), ensure_ascii=True
        ).encode("utf-8")
        return zlib.crc32(encoded) & 0xFFFFFFFF

    def refresh_crc(self) -> int:
        """Recalculate CRC after an intentional payload mutation such as forced expiry."""
        self.crc32 = self.compute_crc32()
        return self.crc32

    def validate_crc(self) -> bool:
        """Return True only when the stored CRC matches the current beacon payload."""
        return self.crc32 is not None and self.crc32 == self.compute_crc32()

    @property
    def crc_hex(self) -> str:
        return f"{self.crc32 or 0:08X}"

    # ── Constructors ─────────────────────────────────────────────────────────

    @classmethod
    def create(
        cls,
        writer_id: str,
        event_type: str,
        severity: float,
        x: float,
        y: float,
        z: float = 0.0,
        heading_deg: float = 0.0,
        ttl_seconds: Optional[int] = None,
    ) -> "BeaconMessage":
        """Factory: auto-generates beacon_id, timestamp, and event-aware TTL."""
        if event_type not in _EVENT_INDEX:
            raise ValueError(f"Unknown event type '{event_type}'. Valid: {SUPPORTED_EVENT_TYPES}")
        if ttl_seconds is None:
            from beacon.ttl import default_ttl_for_event
            ttl_seconds = default_ttl_for_event(event_type)
        return cls(
            beacon_id=f"B{uuid.uuid4().hex[:6].upper()}",
            writer_id=writer_id,
            event=EventPayload(type=event_type, severity=severity),
            position_local=PositionLocal(x=x, y=y, z=z, heading_deg=heading_deg),
            timestamp=datetime.now(timezone.utc).isoformat(),
            ttl_seconds=int(ttl_seconds),
        )

    # ── JSON ─────────────────────────────────────────────────────────────────

    def to_dict(self) -> dict:
        return {
            "beacon_id": self.beacon_id,
            "writer_id": self.writer_id,
            "event": {"type": self.event.type, "severity": round(self.event.severity, 4)},
            "position_local": {
                "x": self.position_local.x,
                "y": self.position_local.y,
                "z": self.position_local.z,
                "heading_deg": self.position_local.heading_deg,
            },
            "timestamp": self.timestamp,
            "ttl_seconds": self.ttl_seconds,
            "crc32": self.crc_hex,
        }

    def serialize(self) -> str:
        """JSON string — used for display and logging."""
        return json.dumps(self.to_dict(), indent=2)

    @classmethod
    def deserialize(cls, data: str | dict) -> "BeaconMessage":
        """Reconstruct from JSON string or dict."""
        if isinstance(data, str):
            data = json.loads(data)
        raw_crc = data.get("crc32")
        if raw_crc is None:
            raise ValueError("Beacon payload is missing required crc32")
        try:
            crc32 = int(raw_crc, 16) if isinstance(raw_crc, str) else int(raw_crc)
        except (TypeError, ValueError) as exc:
            raise ValueError("Beacon crc32 must be an unsigned hexadecimal value") from exc
        beacon = cls(
            beacon_id=data["beacon_id"],
            writer_id=data["writer_id"],
            event=EventPayload(
                type=data["event"]["type"],
                severity=data["event"]["severity"],
            ),
            position_local=PositionLocal(
                x=data["position_local"]["x"],
                y=data["position_local"]["y"],
                z=data["position_local"]["z"],
                heading_deg=data["position_local"]["heading_deg"],
            ),
            timestamp=data["timestamp"],
            ttl_seconds=data["ttl_seconds"],
            crc32=crc32,
        )
        if not beacon.validate_crc():
            raise ValueError(f"Beacon CRC mismatch for {beacon.beacon_id}")
        return beacon

    # ── Compact binary pack (LoRa payload ≤ 64 bytes) ────────────────────────
    # Format: !IIBBFFFFF hHII  (big-endian)
    # Fields: beacon_hash(4) writer_hash(4) event_idx(1) pad(1)
    #         severity(4) x(4) y(4) z(4) heading*100(4) ts_unix(4) ttl(4)
    #         crc32(4)
    # Total: 42 bytes including CRC-32 — within the 64-byte project payload
    #        constraint while preserving end-to-end corruption detection.

    _STRUCT = struct.Struct("!II BB f fff f III")  # 42 bytes, including CRC-32

    def pack(self) -> bytes:
        """Compact binary pack for LoRa transmission."""
        if not self.validate_crc():
            raise ValueError(f"Cannot pack beacon {self.beacon_id}: CRC validation failed")
        bid_hash = int(self.beacon_id.replace("B", "0x", 1), 16) & 0xFFFFFFFF if len(self.beacon_id) <= 8 else hash(self.beacon_id) & 0xFFFFFFFF
        wid_hash = hash(self.writer_id) & 0xFFFFFFFF
        evt_idx = _EVENT_INDEX.get(self.event.type, 0)
        ts_unix = int(datetime.fromisoformat(self.timestamp).timestamp())
        heading_raw = int(self.position_local.heading_deg * 100) & 0xFFFFFFFF
        return self._STRUCT.pack(
            bid_hash,
            wid_hash,
            evt_idx,
            0,  # pad byte
            self.event.severity,
            self.position_local.x,
            self.position_local.y,
            self.position_local.z,
            heading_raw,
            ts_unix,
            self.ttl_seconds,
            self.crc32,
        )

    def pack_size(self) -> int:
        return self._STRUCT.size

    # ── Helpers ───────────────────────────────────────────────────────────────

    def __repr__(self) -> str:
        return (
            f"Beacon({self.beacon_id}, {self.event.type}, "
            f"sev={self.event.severity:.2f}, crc={self.crc_hex}, "
            f"pos=({self.position_local.x:.1f},{self.position_local.y:.1f}))"
        )
