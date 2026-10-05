"""
executor_robot/mission_parser.py
Parse and validate mission briefings received from the Outside Network Area.

CONSTRAINT: Executor only receives missions from the shared executor_inbox queue
(populated by MissionRelay in outside_network/). It never imports command_post/.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import List, Optional

logger = logging.getLogger(__name__)


@dataclass
class Waypoint:
    """
    A single navigation waypoint parsed from a mission briefing.
    Sorted by priority descending before navigation begins.
    """
    waypoint_id: str
    beacon_id: str
    event_type: str
    priority: float      # 0.0–1.0
    severity: float
    lat: float
    lon: float
    alt: float
    estimated_ttl_remaining_s: float
    notes: str = ""
    visited: bool = False
    confirmed: bool = False

    def __lt__(self, other: "Waypoint") -> bool:
        # Higher priority = visit first (for heapq with negation)
        return self.priority > other.priority

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
            "estimated_ttl_remaining_s": round(self.estimated_ttl_remaining_s, 1),
            "notes": self.notes,
            "visited": self.visited,
            "confirmed": self.confirmed,
        }


@dataclass
class ParsedMission:
    briefing_id: str
    relay_id: str
    environment: str
    total_events: int
    waypoints: List[Waypoint]
    summary: str
    warnings: List[str]


def parse_mission(relay_payload: dict) -> Optional[ParsedMission]:
    """
    Parse a relay payload (from MissionRelay) into a ParsedMission.
    Returns None if the payload is malformed.
    """
    try:
        if relay_payload.get("type") != "mission_briefing":
            logger.error(
                f"[MissionParser] Rejected payload type '{relay_payload.get('type')}': "
                "Executor only accepts missions relayed via ONA Role 4 (BRIEF)"
            )
            return None

        relay_id = relay_payload.get("relay_id", "unknown")
        briefing = relay_payload.get("briefing", {})

        waypoints = []
        for wp_dict in briefing.get("waypoints", []):
            wp = Waypoint(
                waypoint_id=wp_dict["waypoint_id"],
                beacon_id=wp_dict["beacon_id"],
                event_type=wp_dict["event_type"],
                priority=float(wp_dict["priority"]),
                severity=float(wp_dict["severity"]),
                lat=float(wp_dict["lat"]),
                lon=float(wp_dict["lon"]),
                alt=float(wp_dict.get("alt", 0.0)),
                estimated_ttl_remaining_s=float(wp_dict.get("estimated_ttl_remaining_s", 3600)),
                notes=wp_dict.get("notes", ""),
            )
            waypoints.append(wp)

        # Sort by priority descending
        waypoints.sort(key=lambda w: w.priority, reverse=True)

        return ParsedMission(
            briefing_id=briefing.get("briefing_id", "?"),
            relay_id=relay_id,
            environment=briefing.get("environment", "USAR"),
            total_events=briefing.get("total_events_detected", len(waypoints)),
            waypoints=waypoints,
            summary=briefing.get("summary", ""),
            warnings=briefing.get("warnings", []),
        )

    except Exception as e:
        logger.error(f"[MissionParser] Failed to parse mission: {e}")
        return None
