"""
outside_network/mission_relay.py
Mission Relay: the ONLY channel through which Command Post communicates
with the Executor Robot.

Flow:
  command_post → ONA authorization control channel → ONA Role 4 (BRIEF)
  ONA Role 4 → executor_robot

CONSTRAINT ENFORCEMENT:
  command_post/ imports mission_relay only to submit operator authorization.
  executor_robot/ reads only the ONA-provided executor inbox.
  Neither robot imports anything from command_post/ directly.
  This file is the enforced relay boundary.
"""

from __future__ import annotations

import json
import logging
import queue
import threading
import time
from typing import Mapping

from outside_network.lora_uplink import SatelliteUplink, DownlinkPacket

logger = logging.getLogger(__name__)


class MissionRelay:
    """
    Receives an authorization control message from the Command Post. ONA Role 4
    then assembles and delivers the already-prepared mission briefing to the
    Executor Robot.

    The authorized path (CP→ONA→Executor):
      1. CP calls receive_authorization_and_brief(briefing_dict)
      2. ONA stores an authorization-control message in its control channel
      3. ONA Role 4 consumes that message and puts the briefing in the
         Executor inbox
      4. Executor Robot reads only that ONA-provided inbox

    The Executor never reads from command_post/ — it reads from the
    ONA-provided executor inbox populated by Role 4.
    """

    SIMULATED_DOWNLINK_DURATION_S = 1.5

    def __init__(
        self,
        uplink: SatelliteUplink,
        executor_inboxes: Mapping[str, queue.Queue] | queue.Queue,
    ):
        """
        uplink: SatelliteUplink — used to serialize downlink packets
        executor_inboxes: per-unit queues that ONA Role 4 uses for targeted
                          ONA→Executor delivery. Keys are stable unit IDs.
                          A single legacy queue is accepted only for older
                          one-Executor simulation scripts.
        """
        self.uplink = uplink
        if isinstance(executor_inboxes, queue.Queue):
            # Backward compatibility for the unchanged one-Executor scenario
            # scripts. New dashboard dispatches always provide a per-unit map.
            executor_inboxes = {"EXECUTOR-2": executor_inboxes}
        self.executor_inboxes = dict(executor_inboxes)
        if not self.executor_inboxes:
            raise ValueError("MissionRelay requires at least one Executor inbox")
        self._authorization_inbox: queue.Queue[dict] = queue.Queue()
        self._relay_count = 0
        self._active_dispatches: dict[str, dict] = {}
        self._lock = threading.Lock()

    def receive_authorization_and_brief(self, briefing: dict, target_unit_id: str) -> str:
        """
        ONA boundary for an operator-approved briefing.

        The Command Post submits authorization here, never to the Executor.
        Role 4 consumes the ONA-local control message before it can deliver the
        briefing to the Executor inbox.
        """
        if target_unit_id not in self.executor_inboxes:
            raise ValueError(f"Unknown Executor unit: {target_unit_id}")
        authorization = {
            "type": "mission_authorization",
            "briefing": briefing,
            "target_unit_id": target_unit_id,
            "authorized_at": time.time(),
        }
        self._authorization_inbox.put(authorization)
        logger.info(
            "[MissionRelay] Authorization received for %s; target=%s",
            briefing.get("briefing_id"), target_unit_id,
        )
        return self._brief_next_authorized_mission()

    def _brief_next_authorized_mission(self) -> str:
        """ONA Role 4 (BRIEF): deliver exactly one authorized mission."""
        try:
            authorization = self._authorization_inbox.get_nowait()
        except queue.Empty as exc:
            raise RuntimeError("ONA Role 4 cannot brief without Command Post authorization") from exc

        briefing = authorization["briefing"]
        target_unit_id = authorization["target_unit_id"]
        with self._lock:
            self._relay_count += 1
            relay_id = f"MISSION-{self._relay_count:03d}"

        payload = {
            "relay_id": relay_id,
            "type": "mission_briefing",
            "briefing": briefing,
            "target_unit_id": target_unit_id,
            "relayed_at": time.time(),
        }
        dispatch = {
            "active": True,
            "relay_id": relay_id,
            "target_unit_id": target_unit_id,
            "briefing_id": briefing.get("briefing_id"),
            "waypoint_count": len(briefing.get("waypoints", [])),
            "started_at": time.time(),
        }
        with self._lock:
            self._active_dispatches[target_unit_id] = dispatch
        # The target queue is still the sole Role 4 delivery path. The short
        # delay models an in-flight ONA downlink and makes its visual state
        # observable without altering mission content or routing.
        threading.Thread(
            target=self._deliver_after_downlink,
            args=(target_unit_id, payload, relay_id),
            name=f"ONARole4-{relay_id}",
            daemon=True,
        ).start()
        logger.info(
            "[MissionRelay] ONA Role 4 downlink started; relay_id=%s briefing_id=%s target=%s waypoints=%s",
            relay_id, briefing.get("briefing_id"), target_unit_id, len(briefing.get("waypoints", [])),
        )
        return relay_id

    def _deliver_after_downlink(self, target_unit_id: str, payload: dict, relay_id: str) -> None:
        """Complete one observable ONA Role 4 downlink without changing its route."""
        try:
            time.sleep(self.SIMULATED_DOWNLINK_DURATION_S)
            # ONA Role 4 is the only code path that puts a mission in this inbox.
            self.executor_inboxes[target_unit_id].put(payload)
            logger.info(
                "[MissionRelay] Briefing delivered; relay_id=%s target=%s",
                relay_id, target_unit_id,
            )
        finally:
            with self._lock:
                dispatch = self._active_dispatches.get(target_unit_id)
                if dispatch and dispatch["relay_id"] == relay_id:
                    del self._active_dispatches[target_unit_id]

    def dispatch_activity(self) -> dict:
        """Return read-only, per-target ONA Role 4 downlink activity."""
        with self._lock:
            transmissions = [dict(dispatch) for dispatch in self._active_dispatches.values()]
        return {"active": bool(transmissions), "transmissions": transmissions}

    def relay_count(self) -> int:
        with self._lock:
            return self._relay_count
