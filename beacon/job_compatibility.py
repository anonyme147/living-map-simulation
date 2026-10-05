"""Explicit event-to-field-unit compatibility rules used for dispatch selection."""

from __future__ import annotations

from typing import Iterable


# Scores are intentionally explicit and deterministic: no learned or fuzzy matching.
EVENT_JOB_COMPATIBILITY: dict[str, dict[str, int]] = {
    "fire_detected": {"fire": 100, "police": 10, "rescue": 25},
    "victim_detected": {"fire": 30, "police": 20, "rescue": 100},
}

# A mission can contain several beacon types but dispatch selects one unit.  A
# confirmed fire takes response precedence: an airborne rescue unit must never
# be presented as the best match for a mission containing a fire beacon.  This
# is a dispatch policy, not a change to the underlying per-event scores above.
MISSION_EVENT_PRECEDENCE: tuple[str, ...] = (
    "fire_detected",
    "victim_detected",
)


def compatibility_score(event_type: str, job_type: str) -> int:
    """Return the explicit unit/job compatibility score for one beacon event."""
    return EVENT_JOB_COMPATIBILITY.get(event_type, {}).get(job_type, 0)


def mission_compatibility_score(event_types: Iterable[object], job_type: str) -> int:
    """Return the selected unit's severity-aware score for mission priority.

    The Command Post authorizes one Executor per briefing.  Averaging mixed
    events made a rescue aircraft outrank FIRE-1 for missions that included a
    confirmed fire plus victim observations.  Resolve one explicit primary
    response class instead: fire first, then victim, then the first known
    event.  When a ``(event_type, severity)`` pair is supplied, the fixed
    role-match percentage is scaled by that beacon's 0.0–1.0 severity.  Plain
    event-type strings remain supported for existing simulation demos and are
    treated as severity 1.0.
    """
    events: list[tuple[str, float]] = []
    for event in event_types:
        if isinstance(event, (tuple, list)) and len(event) >= 2:
            event_type, severity = event[0], event[1]
            try:
                severity = max(0.0, min(1.0, float(severity)))
            except (TypeError, ValueError):
                severity = 1.0
        else:
            event_type, severity = event, 1.0
        if isinstance(event_type, str):
            events.append((event_type, severity))

    for event_type in MISSION_EVENT_PRECEDENCE:
        for candidate_type, severity in events:
            if candidate_type == event_type:
                return round(compatibility_score(event_type, job_type) * severity)
    for event_type, severity in events:
        score = compatibility_score(event_type, job_type)
        if score:
            return round(score * severity)
    return 0
