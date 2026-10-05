"""
command_post/mission_generator.py
Generates mission briefings from accumulated beacon data.
Dispatches a prepared briefing only after Command Post authorization through
the MissionRelay (outside_network/).

CONSTRAINT: This module's only delivery output channel is MissionRelay's ONA
Role 4 interface. It does NOT communicate directly with executor_robot/.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional

from command_post.map_state import LiveMap, MapEvent
from outside_network.mission_relay import MissionRelay

logger = logging.getLogger(__name__)


@dataclass
class WaypointNode:
    """
    A single navigation waypoint for the Executor Robot.
    Includes enough data to navigate without re-exploring.
    """
    waypoint_id: str
    beacon_id: str
    event_type: str
    priority: float          # 0.0–1.0 (higher = visit sooner)
    severity: float
    lat: float
    lon: float
    alt: float
    estimated_ttl_remaining: float   # seconds until beacon expires
    notes: str = ""

    def to_dict(self) -> dict:
        return {
            "waypoint_id": self.waypoint_id,
            "beacon_id": self.beacon_id,
            "event_type": self.event_type,
            "priority": round(self.priority, 4),
            "severity": round(self.severity, 4),
            "lat": self.lat,
            "lon": self.lon,
            "alt": self.alt,
            "estimated_ttl_remaining_s": round(self.estimated_ttl_remaining, 1),
            "notes": self.notes,
        }


@dataclass
class MissionBriefing:
    """
    Full mission briefing package sent to the Executor Robot.
    Serializable as JSON; transmitted via MissionRelay.
    """
    briefing_id: str
    generated_at: str
    environment: str
    total_events_detected: int
    waypoints: List[WaypointNode]
    summary: str
    warnings: List[str] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {
            "briefing_id": self.briefing_id,
            "generated_at": self.generated_at,
            "environment": self.environment,
            "total_events_detected": self.total_events_detected,
            "waypoints": [w.to_dict() for w in self.waypoints],
            "summary": self.summary,
            "warnings": self.warnings,
        }


class MissionGenerator:
    """
    Builds MissionBriefing from LiveMap data and dispatches it via MissionRelay
    only after an operator-authorized command is received.

    Priority scoring (PRD EX-4, communication-aware extension):
      priority_score = severity_priority * (1 + 0.3 * (1 - link_quality))
      - victim_detected gets a 1.5× urgency multiplier
      - fire_detected gets 1.15× multiplier
      - others: 1.0×
      - link_quality comes from an optional satellite-link supplier when one is
        available; otherwise beacon TTL freshness is the deterministic proxy.

    CP-3: Mission is sent ONLY through MissionRelay → outside_network.
    CP-4: The Command Post prepares a briefing, then waits for human approval
          before ONA Role 4 briefs the Executor.
    """

    URGENCY_WEIGHTS = {
        "victim_detected":   1.50,
        "fire_detected":     1.15,
    }

    def __init__(
        self,
        live_map: LiveMap,
        relay: MissionRelay,
        link_quality_provider: Optional[Callable[[], float]] = None,
    ):
        self.live_map = live_map
        self.relay = relay
        self._link_quality_provider = link_quality_provider
        self._briefing_count = 0

    def _link_quality_for_event(self, event: MapEvent, ttl_fraction: float) -> float:
        """Use current satellite quality when supplied, otherwise beacon freshness."""
        if self._link_quality_provider is not None:
            try:
                return max(0.0, min(1.0, float(self._link_quality_provider())))
            except (TypeError, ValueError):
                logger.warning("[MissionGen] Invalid link-quality provider value; using beacon TTL freshness")
        return max(0.0, min(1.0, ttl_fraction))

    def generate(self) -> Optional[MissionBriefing]:
        """
        Prepare a mission briefing from current map state without dispatching it.
        Returns the briefing or None if no events are available.
        """
        events = self.live_map.get_active_events()
        if not events:
            logger.warning("[MissionGen] No active events — briefing not generated.")
            return None

        self._briefing_count += 1
        briefing_id = f"BRIEF-{self._briefing_count:03d}"
        warnings = []

        ranked_waypoints: List[tuple[float, WaypointNode]] = []
        for idx, event in enumerate(events):
            ttl_remaining = max(0.0, event.ttl_seconds - event.age_s())
            ttl_frac = ttl_remaining / max(event.ttl_seconds, 1)
            urgency = self.URGENCY_WEIGHTS.get(event.event_type, 1.0)
            severity_priority = event.severity * ttl_frac * urgency
            link_quality = self._link_quality_for_event(event, ttl_frac)
            priority_score = severity_priority * (1.0 + 0.3 * (1.0 - link_quality))
            priority = min(1.0, priority_score)

            notes = ""
            if ttl_frac < 0.25:
                notes = "⚠️ Beacon nearly expired — visit urgently"
                warnings.append(f"{event.beacon_id}: TTL < 25%")
            elif event.severity > 0.85:
                notes = "🔴 High-severity event — prioritise"

            wp = WaypointNode(
                waypoint_id=f"WP{idx+1:02d}",
                beacon_id=event.beacon_id,
                event_type=event.event_type,
                priority=priority,
                severity=event.severity,
                lat=event.lat,
                lon=event.lon,
                alt=event.alt,
                estimated_ttl_remaining=ttl_remaining,
                notes=notes,
            )
            ranked_waypoints.append((priority_score, wp))

        # Rank by the complete communication-aware score, then express every
        # waypoint relative to the most urgent current target.  This preserves
        # the 0.0–1.0 public contract while keeping a life-critical victim
        # visibly above a simultaneous fire in the operator briefing.
        ranked_waypoints.sort(key=lambda item: item[0], reverse=True)
        highest_score = ranked_waypoints[0][0] if ranked_waypoints else 0.0
        if highest_score > 0:
            for score, waypoint in ranked_waypoints:
                waypoint.priority = min(1.0, max(0.0, score / highest_score))
        waypoints = [waypoint for _, waypoint in ranked_waypoints]
        # Re-assign waypoint IDs after sort
        for idx, wp in enumerate(waypoints):
            wp.waypoint_id = f"WP{idx+1:02d}"

        summary = (
            f"Mission covers {len(waypoints)} event(s) in USAR zone. "
            f"Top priority: {waypoints[0].event_type} at "
            f"({waypoints[0].lat:.5f}, {waypoints[0].lon:.5f}) "
            f"[sev={waypoints[0].severity:.2f}]."
        )

        briefing = MissionBriefing(
            briefing_id=briefing_id,
            generated_at=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            environment="Urban Search & Rescue (USAR)",
            total_events_detected=len(self.live_map.get_events()),
            waypoints=waypoints,
            summary=summary,
            warnings=warnings,
        )

        logger.info(
            f"[MissionGen] Generated {briefing_id} with {len(waypoints)} waypoints"
        )

        return briefing

    def dispatch_authorized(self, briefing: MissionBriefing | dict, target_unit_id: str) -> str:
        """Hand an approved briefing to ONA Role 4; never contact Executor directly."""
        payload = briefing.to_dict() if isinstance(briefing, MissionBriefing) else briefing
        relay_id = self.relay.receive_authorization_and_brief(payload, target_unit_id)
        logger.info(
            "[MissionGen] Authorized briefing dispatched through ONA as %s for %s",
            relay_id, target_unit_id,
        )
        return relay_id
