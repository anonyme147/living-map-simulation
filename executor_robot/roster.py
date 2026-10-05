"""Executor-unit roster and deterministic mission compatibility rules."""

from __future__ import annotations

from dataclasses import dataclass
from beacon.job_compatibility import compatibility_score, mission_compatibility_score


@dataclass(frozen=True)
class ExecutorUnitDefinition:
    """Static identity and initial state for an ONA-dispatched field unit."""

    unit_id: str
    display_name: str
    job_type: str
    icon_url: str
    icon_tint: str
    start_lat: float
    start_lon: float
    initial_battery: float = 100.0

    def to_dashboard_dict(self) -> dict:
        return {
            "unit_id": self.unit_id,
            "display_name": self.display_name,
            "job_type": self.job_type,
            "icon_url": self.icon_url,
            "icon_tint": self.icon_tint,
            "lat": self.start_lat,
            "lon": self.start_lon,
            "battery": self.initial_battery,
            "status": "AVAILABLE",
        }


EXECUTOR_UNITS: tuple[ExecutorUnitDefinition, ...] = (
    ExecutorUnitDefinition(
        unit_id="FIRE-1",
        display_name="Fire Response Unit",
        job_type="fire",
        icon_url="https://cdn.jsdelivr.net/npm/@mdi/svg@7.4.47/svg/fire-truck.svg",
        icon_tint="#ef4444",
        start_lat=36.846000,
        start_lon=10.158000,
    ),
    ExecutorUnitDefinition(
        unit_id="POLICE-1",
        display_name="Police Support Unit",
        job_type="police",
        icon_url="https://cdn.jsdelivr.net/npm/@mdi/svg@7.4.47/svg/car-emergency.svg",
        icon_tint="#2563eb",
        start_lat=36.840000,
        start_lon=10.170000,
    ),
    ExecutorUnitDefinition(
        unit_id="RESCUE-1",
        display_name="Rescue Air Unit",
        job_type="rescue",
        icon_url="https://cdn.jsdelivr.net/npm/@mdi/svg@7.4.47/svg/helicopter.svg",
        icon_tint="#10b981",
        start_lat=36.856000,
        start_lon=10.152000,
    ),
)
