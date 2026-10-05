"""Simulated satellite link for the ONA CARRY role.

The link models fixed propagation latency and weather/pass-handoff dropouts;
it deliberately omits short-range RF attenuation and ISM duty-cycle limits.
"""
from __future__ import annotations

import json
import logging
import queue
import random
import threading
import time
import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

logger = logging.getLogger(__name__)


# Lightweight RCAMP-inspired satellite-link quality model.  This is telemetry
# only: it deliberately never influences packet send, retry, or buffering.
LINK_QUALITY_BASE_STRENGTH = 1.0
LINK_QUALITY_PATH_LOSS_EXPONENT = 0.12
LINK_QUALITY_NOISE_TERM = 0.02


@dataclass
class UplinkPacket:
    packet_id: str
    beacon_data: dict
    attempt: int = 0
    first_sent_at: Optional[float] = None

    def to_json(self) -> str:
        return json.dumps({"packet_id": self.packet_id, "beacon_data": self.beacon_data, "attempt": self.attempt})


@dataclass
class DownlinkPacket:
    packet_id: str
    payload: dict
    attempt: int = 0


class BaseSatelliteDriver(ABC):
    """Transport interface for a simulated or physical satellite terminal."""

    @abstractmethod
    def send(self, data: bytes) -> bool: ...

    @abstractmethod
    def receive(self, timeout_s: float) -> Optional[bytes]: ...

    @abstractmethod
    def is_available(self) -> bool: ...


class SimulatedSatelliteDriver(BaseSatelliteDriver):
    """Queue transport with satellite latency and weather/pass dropouts.

    ``packet_loss_prob`` represents a weather or satellite-handoff dropout, not
    range-derived loss. ``link_disrupted`` is the dashboard's deterministic
    operator scenario control.
    """

    def __init__(self, uplink_queue: queue.Queue, downlink_queue: queue.Queue,
                 packet_loss_prob: float = 0.02, latency_ms: int = 650):
        self._uplink_q = uplink_queue
        self._downlink_q = downlink_queue
        self.packet_loss_prob = packet_loss_prob
        self.latency_ms = latency_ms
        self.link_disrupted = False
        self._lock = threading.Lock()
        self._last_successful_contact_at = time.monotonic()

    def set_disruption(self, duration_s: float = 12.0) -> None:
        """Simulate a satellite handoff/weather outage, then restore service."""
        with self._lock:
            self.link_disrupted = True
            logger.warning("[SimSatellite] Link disrupted for %ss; buffering enabled", duration_s)

        def restore() -> None:
            time.sleep(duration_s)
            with self._lock:
                self.link_disrupted = False
            logger.info("[SimSatellite] Link restored; draining retry queue")

        threading.Thread(target=restore, daemon=True).start()

    def send(self, data: bytes) -> bool:
        with self._lock:
            unavailable = self.link_disrupted
            dropout = self.packet_loss_prob
        if unavailable or random.random() < dropout:
            logger.debug("[SimSatellite] Packet delayed by simulated pass/weather dropout")
            return False
        time.sleep(self.latency_ms / 1000.0)
        self._uplink_q.put(data)
        with self._lock:
            self._last_successful_contact_at = time.monotonic()
        logger.debug("[SimSatellite] TX %s bytes after %sms latency", len(data), self.latency_ms)
        return True

    def receive(self, timeout_s: float = 1.0) -> Optional[bytes]:
        try:
            return self._downlink_q.get(timeout=timeout_s)
        except queue.Empty:
            return None

    def is_available(self) -> bool:
        with self._lock:
            return not self.link_disrupted

    def link_quality(self) -> float:
        """Return an RCAMP-inspired, telemetry-only satellite link estimate.

        Satellites do not use the robots' ground distance as their propagation
        distance in this simulation, so time since the last successful contact
        is the deterministic proxy.  This value must not gate transport: the
        existing dropout, retry, and store-and-forward paths remain unchanged.
        """
        with self._lock:
            seconds_since_contact = max(1.0, time.monotonic() - self._last_successful_contact_at)

        quality = (
            LINK_QUALITY_BASE_STRENGTH
            - LINK_QUALITY_PATH_LOSS_EXPONENT * math.log10(seconds_since_contact)
            - LINK_QUALITY_NOISE_TERM
        )
        return max(0.0, min(1.0, quality))


class RetryQueue:
    """Store-and-forward retry queue for temporary satellite-link outages."""
    BASE_DELAY = 1.0
    MAX_DELAY = 60.0
    MAX_ATTEMPTS = 8

    def __init__(self, driver: BaseSatelliteDriver):
        self._driver = driver
        self._pending: queue.Queue[UplinkPacket] = queue.Queue()
        self._sent_count = 0
        self._failed_count = 0
        self._active_transmission: Optional[dict] = None
        self._lock = threading.Lock()

    def enqueue(self, packet: UplinkPacket) -> None:
        self._pending.put(packet)

    def drain(self) -> None:
        while True:
            try:
                self._send_with_retry(self._pending.get(timeout=0.5))
            except queue.Empty:
                continue

    def _send_with_retry(self, packet: UplinkPacket) -> bool:
        for attempt in range(self.MAX_ATTEMPTS):
            packet.attempt = attempt + 1
            packet.first_sent_at = packet.first_sent_at or time.time()
            self._begin_transmission(packet)
            try:
                sent = self._driver.send(packet.to_json().encode("utf-8"))
            finally:
                self._end_transmission(packet.packet_id)
            if sent:
                with self._lock:
                    self._sent_count += 1
                logger.info("[RetryQueue] Sent %s (attempt %s)", packet.packet_id, packet.attempt)
                return True
            delay = min(self.BASE_DELAY * (2 ** attempt), self.MAX_DELAY)
            logger.warning("[RetryQueue] Satellite delivery delayed for %s; retry in %.1fs", packet.packet_id, delay)
            time.sleep(delay)
        with self._lock:
            self._failed_count += 1
        logger.error("[RetryQueue] Gave up on %s after %s attempts", packet.packet_id, self.MAX_ATTEMPTS)
        return False

    def _begin_transmission(self, packet: UplinkPacket) -> None:
        """Publish display-only metadata while ONA Role 3 is calling send()."""
        beacon = packet.beacon_data
        gps = beacon.get("gps", {})
        with self._lock:
            self._active_transmission = {
                "active": True,
                "packet_id": packet.packet_id,
                "attempt": packet.attempt,
                "source_unit_id": beacon.get("writer_id"),
                "beacon_id": beacon.get("beacon_id"),
                "event_type": beacon.get("event", {}).get("type"),
                "source": {
                    "lat": gps.get("lat"),
                    "lon": gps.get("lon"),
                },
                "started_at": time.time(),
            }

    def _end_transmission(self, packet_id: str) -> None:
        """Clear only the matching display snapshot after the send attempt ends."""
        with self._lock:
            if self._active_transmission and self._active_transmission["packet_id"] == packet_id:
                self._active_transmission = None

    def transmission_activity(self) -> dict:
        """Return a read-only snapshot for the Command Post CARRY visual."""
        with self._lock:
            return dict(self._active_transmission) if self._active_transmission else {"active": False}

    def stats(self) -> dict:
        with self._lock:
            return {"pending": self._pending.qsize(), "sent": self._sent_count, "failed": self._failed_count}


class SatelliteUplink:
    """ONA Role 3 CARRY transport between the perimeter node and Command Post."""
    def __init__(self, driver: BaseSatelliteDriver, retry_queue: RetryQueue):
        self.driver = driver
        self.retry_queue = retry_queue
        self._packet_counter = 0

    def send_beacon(self, enriched_beacon: dict) -> str:
        self._packet_counter += 1
        packet_id = f"PKT{self._packet_counter:04d}"
        self.retry_queue.enqueue(UplinkPacket(packet_id=packet_id, beacon_data=enriched_beacon))
        logger.info("[SatelliteUplink] Queued %s for uplink", packet_id)
        return packet_id

    def transmission_activity(self) -> dict:
        """Expose CARRY activity metadata without changing delivery behavior."""
        return self.retry_queue.transmission_activity()

    def receive_downlink(self, timeout_s: float = 0.5) -> Optional[dict]:
        data = self.driver.receive(timeout_s)
        if data is None:
            return None
        try:
            return json.loads(data.decode("utf-8"))
        except Exception as exc:
            logger.error("[SatelliteUplink] Failed to parse downlink: %s", exc)
            return None
