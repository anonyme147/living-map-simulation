"""
command_post/map_state.py
LiveMap: maintains the Command Post's picture of the disaster zone.
Ingests uplink packets from ONA; exports GeoJSON for the web dashboard.

CONSTRAINT: This module ONLY reads from the uplink_queue (populated by ONA).
It never reads from BeaconStore directly and has no imports from writer_robot/.
"""

from __future__ import annotations

import json
import logging
import queue
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional

from beacon.schema import BeaconMessage

logger = logging.getLogger(__name__)

# Event type → colour mapping for Leaflet map
EVENT_COLORS = {
    "victim_detected":   "#FF4136",   # red — urgent
    "fire_detected":     "#FF851B",   # orange
}


@dataclass
class MapEvent:
    """One event on the live map (enriched beacon data)."""
    beacon_id: str
    writer_id: str
    event_type: str
    severity: float
    lat: float
    lon: float
    alt: float
    timestamp: str
    ttl_seconds: int
    packet_id: str
    drift_corrected: bool
    crc32: str
    received_at: float = field(default_factory=time.time)

    def age_s(self) -> float:
        return time.time() - self.received_at

    def is_stale(self) -> bool:
        return self.age_s() > self.ttl_seconds

    def to_geojson_feature(self) -> dict:
        return {
            "type": "Feature",
            "geometry": {"type": "Point", "coordinates": [self.lon, self.lat, self.alt]},
            "properties": {
                "beacon_id": self.beacon_id,
                "writer_id": self.writer_id,
                "event_type": self.event_type,
                "severity": self.severity,
                "timestamp": self.timestamp,
                "ttl_seconds": self.ttl_seconds,
                "age_s": round(self.age_s(), 1),
                "stale": self.is_stale(),
                "drift_corrected": self.drift_corrected,
                "crc32": self.crc32,
                "color": EVENT_COLORS.get(self.event_type, "#AAAAAA"),
            },
        }

    def to_dict(self) -> dict:
        return {
            "beacon_id": self.beacon_id,
            "writer_id": self.writer_id,
            "event_type": self.event_type,
            "severity": self.severity,
            "lat": self.lat,
            "lon": self.lon,
            "alt": self.alt,
            "timestamp": self.timestamp,
            "ttl_seconds": self.ttl_seconds,
            "age_s": round(self.age_s(), 1),
            "stale": self.is_stale(),
            "drift_corrected": self.drift_corrected,
            "crc32": self.crc32,
            "color": EVENT_COLORS.get(self.event_type, "#AAAAAA"),
        }


class LiveMap:
    """
    Thread-safe live map of received events.
    Ingests from uplink_queue; exports GeoJSON / event lists.
    """

    def __init__(
        self,
        uplink_queue: queue.Queue,
        on_event_ingested: Optional[Callable[[MapEvent], None]] = None,
    ):
        self._uplink_q = uplink_queue
        self._on_event_ingested = on_event_ingested
        self._events: Dict[str, MapEvent] = {}
        self._lock = threading.RLock()
        self._ingest_count = 0
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        """Start background ingestion loop."""
        self._running = True
        self._thread = threading.Thread(
            target=self._ingest_loop, name="LiveMap-Ingest", daemon=True
        )
        self._thread.start()
        logger.info("[LiveMap] Started ingestion loop")

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def _ingest_loop(self) -> None:
        while self._running:
            try:
                raw = self._uplink_q.get(timeout=0.3)
                self._ingest(raw)
            except queue.Empty:
                continue
            except Exception as e:
                logger.error(f"[LiveMap] Ingest error: {e}")

    def _ingest(self, raw_bytes_or_str) -> None:
        """Parse an uplink packet and add/update the map."""
        try:
            if isinstance(raw_bytes_or_str, bytes):
                packet = json.loads(raw_bytes_or_str.decode("utf-8"))
            elif isinstance(raw_bytes_or_str, str):
                packet = json.loads(raw_bytes_or_str)
            elif isinstance(raw_bytes_or_str, dict):
                packet = raw_bytes_or_str
            else:
                logger.warning(f"[LiveMap] Unknown packet type: {type(raw_bytes_or_str)}")
                return

            bd = packet.get("beacon_data", packet)
            # Command Post accepts only an intact original beacon payload. GPS
            # enrichment is intentionally outside the CRC-covered source data.
            verified_beacon = BeaconMessage.deserialize(bd)
            gps = bd.get("gps", {})
            evt = bd.get("event", {})

            event = MapEvent(
                beacon_id=verified_beacon.beacon_id,
                writer_id=verified_beacon.writer_id,
                event_type=evt.get("type", "unknown"),
                severity=evt.get("severity", 0.0),
                lat=gps.get("lat", 0.0),
                lon=gps.get("lon", 0.0),
                alt=gps.get("alt", 0.0),
                timestamp=bd.get("timestamp", ""),
                ttl_seconds=bd.get("ttl_seconds", 3600),
                packet_id=packet.get("packet_id", ""),
                drift_corrected=gps.get("drift_corrected", False),
                crc32=verified_beacon.crc_hex,
            )

            with self._lock:
                self._events[event.beacon_id] = event
                self._ingest_count += 1

            logger.info(
                f"[LiveMap] +Event {event.beacon_id} | {event.event_type} "
                f"sev={event.severity:.2f} crc={event.crc32} @ ({event.lat:.5f}, {event.lon:.5f})"
            )
            if self._on_event_ingested is not None:
                try:
                    self._on_event_ingested(event)
                except Exception as callback_error:
                    logger.error(f"[LiveMap] Ingest callback error: {callback_error}")
        except Exception as e:
            logger.error(f"[LiveMap] Failed to ingest packet: {e} | raw={raw_bytes_or_str!r:.120}")

    # ── Public API ─────────────────────────────────────────────────────────────

    def ingest_direct(self, packet: dict) -> None:
        """Directly ingest a packet (used in simulation without queue delay)."""
        self._ingest(packet)

    def get_events(self) -> List[MapEvent]:
        with self._lock:
            return sorted(
                self._events.values(),
                key=lambda e: e.severity,
                reverse=True,
            )

    def get_active_events(self) -> List[MapEvent]:
        with self._lock:
            return sorted(
                [e for e in self._events.values() if not e.is_stale()],
                key=lambda e: e.severity,
                reverse=True,
            )

    def to_geojson(self) -> dict:
        with self._lock:
            return {
                "type": "FeatureCollection",
                "features": [e.to_geojson_feature() for e in self._events.values()],
            }

    def get_summary(self) -> dict:
        with self._lock:
            events = list(self._events.values())
            by_type: dict = {}
            for e in events:
                by_type.setdefault(e.event_type, []).append(e.severity)
            return {
                "total_events": len(events),
                "active_events": sum(1 for e in events if not e.is_stale()),
                "ingest_count": self._ingest_count,
                "by_type": {
                    k: {"count": len(v), "max_severity": max(v), "avg_severity": sum(v)/len(v)}
                    for k, v in by_type.items()
                },
            }

    def __len__(self) -> int:
        with self._lock:
            return len(self._events)
