"""
outside_network/receiver.py
Beacon receiver: polls the shared BeaconStore for newly dropped beacons,
transforms their coordinates, and queues them for satellite uplink.

CONSTRAINT: This is the ONLY module that reads beacons from the zone
and forwards them outward. No direct path to command_post/ exists here.
All data exits through the CARRY transport → uplink_queue → command_post reads.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Optional

from beacon.store import BeaconStore
from outside_network.frame_transform import LocalToGPS, GPSAnchor, DriftModel
from outside_network.lora_uplink import SatelliteUplink

logger = logging.getLogger(__name__)


class BeaconReceiver:
    """
    Polls the BeaconStore for unsent beacons.
    Transforms local coordinates to GPS.
    Hands off to SatelliteUplink for store-and-forward transmission.

    PRD requirements:
      ON-1 Receive beacons from Writer Robot's area
      ON-2 Translate local→GPS with drift correction
      ON-3 Forward via satellite uplink
      ON-4 Only path out of the zone (enforced by module structure)
      ON-6 Deduplication via BeaconStore.mark_sent_to_ona()
    """

    POLL_INTERVAL = 0.2  # seconds between store polls

    def __init__(
        self,
        beacon_store: BeaconStore,
        transformer: LocalToGPS,
        uplink: SatelliteUplink,
    ):
        self.store = beacon_store
        self.transformer = transformer
        self.uplink = uplink
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._beacons_forwarded = 0

    def start(self) -> None:
        """Start the receiver loop in a background thread."""
        self._running = True
        self._thread = threading.Thread(
            target=self._loop, name="BeaconReceiver", daemon=True
        )
        self._thread.start()
        logger.info("[BeaconReceiver] Started — polling BeaconStore")

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        logger.info(f"[BeaconReceiver] Stopped. Forwarded {self._beacons_forwarded} beacons.")

    def _loop(self) -> None:
        while self._running:
            unsent = self.store.get_unsent()
            for beacon in unsent:
                self._process(beacon)
            time.sleep(self.POLL_INTERVAL)

    def _process(self, beacon) -> None:
        """Transform + forward a single beacon."""
        try:
            if not beacon.validate_crc():
                raise ValueError("CRC validation failed before ONA RECEIVE")
            enriched = self.transformer.transform_beacon(beacon)
            self.uplink.send_beacon(enriched)
            self.store.mark_sent_to_ona(beacon.beacon_id)
            self._beacons_forwarded += 1
            logger.info(
                f"[BeaconReceiver] Forwarded {beacon.beacon_id} "
                f"({beacon.event.type}, CRC={beacon.crc_hex}) → GPS "
                f"({enriched['gps']['lat']}, {enriched['gps']['lon']})"
            )
        except Exception as e:
            logger.error(f"[BeaconReceiver] Error processing {beacon.beacon_id}: {e}")

    @property
    def beacons_forwarded(self) -> int:
        return self._beacons_forwarded
