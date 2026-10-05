"""Automatic, demo-only ONA Role 3 satellite-to-mesh failover transport.

The normal satellite driver and its shared retry queue are deliberately not
modified.  This adapter is selected only by the satellite-failover scenario:
it attempts the satellite first and switches to the existing RF mesh only when
the existing dashboard failure target marks the satellite unavailable.
"""

from __future__ import annotations

import logging
import threading

from outside_network.lora_uplink import BaseSatelliteDriver, RetryQueue, SatelliteUplink, SimulatedSatelliteDriver
from outside_network.rf_mesh import MeshRelayDriver


logger = logging.getLogger(__name__)


class SatelliteMeshFailoverDriver(BaseSatelliteDriver):
    """Use satellite normally and automatically carry outage traffic via mesh."""

    def __init__(
        self,
        satellite_driver: SimulatedSatelliteDriver,
        mesh_driver: MeshRelayDriver,
        second_command_post_arrival: threading.Event | None = None,
    ) -> None:
        self.satellite_driver = satellite_driver
        self.mesh_driver = mesh_driver
        self._lock = threading.RLock()
        self._failover_active = False
        self._failure_logged = False
        self._satellite_packets = 0
        self._mesh_packets = 0
        # Used only by the arrival-trigger demo: keep packet three out of the
        # satellite send loop until Command Post has acknowledged packet two.
        self._second_command_post_arrival = second_command_post_arrival

    @property
    def link_disrupted(self) -> bool:
        """Expose the existing dashboard failure state through this facade."""
        return self.satellite_driver.link_disrupted

    @link_disrupted.setter
    def link_disrupted(self, value: bool) -> None:
        self.satellite_driver.link_disrupted = bool(value)
        if value:
            logger.warning(
                "[Failover] Satellite failure state received; automatic RF mesh failover armed"
            )

    @property
    def packet_loss_prob(self) -> float:
        return self.satellite_driver.packet_loss_prob

    @packet_loss_prob.setter
    def packet_loss_prob(self, value: float) -> None:
        self.satellite_driver.packet_loss_prob = value

    @property
    def failover_active(self) -> bool:
        with self._lock:
            return self._failover_active

    def send(self, data: bytes) -> bool:
        """Carry one packet over satellite, or mesh after actual satellite loss."""
        if self.satellite_driver.is_available():
            delivered = self.satellite_driver.send(data)
            if delivered:
                with self._lock:
                    self._satellite_packets += 1
                    satellite_packet_number = self._satellite_packets
                if satellite_packet_number == 2 and self._second_command_post_arrival is not None:
                    logger.info(
                        "[Failover] Waiting for Command Post acknowledgement of second satellite packet"
                    )
                    if not self._second_command_post_arrival.wait(timeout=5.0):
                        logger.error(
                            "[Failover] Timed out waiting for Command Post acknowledgement; "
                            "satellite failover was not armed"
                        )
            return delivered

        with self._lock:
            self._failover_active = True
            first_detection = not self._failure_logged
            self._failure_logged = True
        if first_detection:
            logger.warning(
                "[Failover] SATELLITE FAILURE DETECTED: ONA Role 3 switching queued traffic to RF mesh"
            )
        logger.info("[Failover] AUTOMATIC RF MESH FAILOVER: carrying packet through relay nodes")
        delivered = self.mesh_driver.send(data)
        if delivered:
            with self._lock:
                self._mesh_packets += 1
        return delivered

    def receive(self, timeout_s: float = 1.0):
        """Keep the existing SatelliteUplink downlink interface available."""
        return self.satellite_driver.receive(timeout_s)

    def is_available(self) -> bool:
        """ONA remains available if either the primary or mesh carrier is alive."""
        return self.satellite_driver.is_available() or self.mesh_driver.is_available()

    def link_quality(self) -> float:
        """The dashboard quality gauge continues to describe the satellite link."""
        return self.satellite_driver.link_quality()

    def mesh_snapshot(self) -> dict:
        """Hide mesh layers until the satellite-loss failover is genuinely active."""
        snapshot = self.mesh_driver.mesh_snapshot()
        snapshot["enabled"] = self.failover_active
        snapshot["mode"] = "mesh_failover" if self.failover_active else "satellite_primary"
        return snapshot

    def stats(self) -> dict:
        with self._lock:
            return {
                "satellite_packets": self._satellite_packets,
                "mesh_packets": self._mesh_packets,
                "failover_active": self._failover_active,
            }


class FailoverSatelliteUplink(SatelliteUplink):
    """Dashboard-aware uplink facade for the isolated failover demonstration."""

    def __init__(self, driver: SatelliteMeshFailoverDriver, retry_queue: RetryQueue):
        super().__init__(driver, retry_queue)
        self.failover_driver = driver

    def mesh_snapshot(self) -> dict:
        return self.failover_driver.mesh_snapshot()

    def transmission_activity(self) -> dict:
        """Show the normal direct line before failure, mesh hops after failover."""
        mesh_activity = self.failover_driver.mesh_driver.mesh_snapshot()["activity"]
        if mesh_activity.get("active"):
            # The map's normal satellite line must disappear while this same
            # packet is visibly travelling through the per-hop mesh renderer.
            return {"active": False, "mesh": True, "failover": True}
        return self.retry_queue.transmission_activity()
