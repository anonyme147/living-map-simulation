"""ONA-to-Command-Post heartbeat observability for the Zombie Link demo.

This is intentionally independent from Role 3 packet delivery.  A heartbeat
is a trust signal only: losing it must never queue, block, retry, or reroute a
beacon packet.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Optional


logger = logging.getLogger(__name__)


class HeartbeatMonitor:
    """Simulate ONA keepalives and expose Command Post trust status.

    The fault is armed only after the first real Command Post beacon arrival.
    Recovery waits until a further real beacon arrives while the heartbeat gap
    is already degraded, making the zombie-link contrast demonstrable rather
    than merely asserted in logs.
    """

    def __init__(
        self,
        *,
        interval_s: float = 1.0,
        threshold_s: float = 4.0,
        normal_after_first_data_s: float = 2.0,
        recovery_after_degraded_data_s: float = 2.0,
        maximum_loss_s: float = 14.0,
        on_degraded=None,
        on_recovered=None,
    ) -> None:
        self.interval_s = float(interval_s)
        self.threshold_s = float(threshold_s)
        self.normal_after_first_data_s = float(normal_after_first_data_s)
        self.recovery_after_degraded_data_s = float(recovery_after_degraded_data_s)
        self.maximum_loss_s = float(maximum_loss_s)
        self._on_degraded = on_degraded
        self._on_recovered = on_recovered
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._started_at: Optional[float] = None
        self._last_received_at: Optional[float] = None
        self._loss_starts_at: Optional[float] = None
        self._recovery_at: Optional[float] = None
        self._loss_started_at: Optional[float] = None
        self._received_count = 0
        self._suppressed_count = 0
        self._data_arrivals = 0
        self._data_while_degraded = 0
        self._degraded_logged = False
        self._recovered_logged = False

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        with self._lock:
            self._started_at = time.monotonic()
            self._last_received_at = self._started_at
        self._thread = threading.Thread(target=self._loop, name="ONAHeartbeat", daemon=True)
        self._thread.start()
        logger.info("[Heartbeat] ONA ↔ Command Post monitor started; interval=%.1fs threshold=%.1fs", self.interval_s, self.threshold_s)

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.5)

    def note_command_post_beacon(self, event) -> None:
        """Observe a real ONA-carried beacon after Command Post ingestion."""
        now = time.monotonic()
        beacon_id = getattr(event, "beacon_id", "unknown")
        with self._lock:
            self._data_arrivals += 1
            if self._loss_starts_at is None:
                self._loss_starts_at = now + self.normal_after_first_data_s
                logger.info(
                    "[Heartbeat] First Command Post beacon %s confirmed; normal keepalives continue for %.1fs before zombie-link window",
                    beacon_id,
                    self.normal_after_first_data_s,
                )
                return
            if self._loss_started_at is not None and self._gap_seconds_locked(now) >= self.threshold_s:
                self._data_while_degraded += 1
                if self._recovery_at is None:
                    self._recovery_at = now + self.recovery_after_degraded_data_s
                    logger.warning(
                        "[Heartbeat] DATA STILL FLOWING WHILE DEGRADED: beacon=%s; heartbeat recovery scheduled in %.1fs",
                        beacon_id,
                        self.recovery_after_degraded_data_s,
                    )

    def _gap_seconds_locked(self, now: float) -> float:
        return max(0.0, now - (self._last_received_at or now))

    def _in_loss_window_locked(self, now: float) -> bool:
        if self._loss_starts_at is None or now < self._loss_starts_at:
            return False
        if self._recovery_at is not None and now >= self._recovery_at:
            return False
        if self._loss_started_at is not None and now - self._loss_started_at >= self.maximum_loss_s:
            return False
        return True

    def _loop(self) -> None:
        next_tick = time.monotonic()
        while not self._stop.wait(max(0.0, next_tick - time.monotonic())):
            now = time.monotonic()
            next_tick = now + self.interval_s
            with self._lock:
                in_loss_window = self._in_loss_window_locked(now)
                if in_loss_window:
                    if self._loss_started_at is None:
                        self._loss_started_at = now
                        logger.warning("[Heartbeat] KEEPALIVE DELIVERY SUPPRESSED — beacon CARRY remains nominal")
                    self._suppressed_count += 1
                    gap = self._gap_seconds_locked(now)
                    if gap >= self.threshold_s and not self._degraded_logged:
                        self._degraded_logged = True
                        logger.warning(
                            "[Heartbeat] TRUST THRESHOLD CROSSED: %.1fs without keepalive (threshold %.1fs) — DEGRADED / UNCONFIRMED",
                            gap,
                            self.threshold_s,
                        )
                        if self._on_degraded is not None:
                            try:
                                self._on_degraded()
                            except Exception:
                                logger.exception("[Heartbeat] Degraded-state observer failed")
                    continue

                self._received_count += 1
                was_degraded = self._degraded_logged
                self._last_received_at = now
                if was_degraded and not self._recovered_logged:
                    self._recovered_logged = True
                    logger.info("[Heartbeat] KEEPALIVE RECEIVED — TRUST STATUS RECOVERED to HEALTHY")
                    if self._on_recovered is not None:
                        try:
                            self._on_recovered()
                        except Exception:
                            logger.exception("[Heartbeat] Recovery observer failed")
                logger.info("[Heartbeat] ONA → Command Post keepalive received #%s", self._received_count)

    def status(self) -> dict:
        now = time.monotonic()
        with self._lock:
            gap = self._gap_seconds_locked(now)
            if self._loss_started_at is not None and gap >= self.threshold_s and not self._recovered_logged:
                state = "degraded_unconfirmed"
            elif self._loss_starts_at is not None and self._loss_started_at is not None and not self._recovered_logged:
                state = "awaiting_heartbeat"
            else:
                state = "healthy"
            return {
                "enabled": True,
                "state": state,
                "interval_s": self.interval_s,
                "threshold_s": self.threshold_s,
                "gap_s": round(gap, 1),
                "received_count": self._received_count,
                "suppressed_count": self._suppressed_count,
                "data_arrivals": self._data_arrivals,
                "data_while_degraded": self._data_while_degraded,
            }
