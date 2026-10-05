"""
executor_robot/status_reporter.py
Reports arrival confirmations and mission status back through Outside Network Area only.

CONSTRAINT: No direct link to command_post/. Status is put onto a shared
status_queue that outside_network/ (or the simulation) monitors.
"""

from __future__ import annotations

import logging
import queue
import time
from dataclasses import dataclass
from typing import Optional

logger = logging.getLogger(__name__)


@dataclass
class StatusReport:
    robot_id: str
    report_type: str    # "arrival", "mission_complete", "error"
    waypoint_id: Optional[str]
    beacon_id: Optional[str]
    event_type: Optional[str]
    message: str
    timestamp: float
    source: str = "ONA Relayed Mission"

    def to_dict(self) -> dict:
        return {
            "robot_id": self.robot_id,
            "report_type": self.report_type,
            "waypoint_id": self.waypoint_id,
            "beacon_id": self.beacon_id,
            "event_type": self.event_type,
            "message": self.message,
            "details": self.message,
            "source": self.source,
            "timestamp": self.timestamp,
        }


class StatusReporter:
    """
    Sends status reports to the shared status_queue (read by ONA/simulation).
    This is the only upward communication channel for the Executor Robot.
    """

    def __init__(self, robot_id: str, status_queue: queue.Queue):
        self.robot_id = robot_id
        self._queue = status_queue
        self._report_count = 0

    def report_arrival(self, waypoint_id: str, beacon_id: str, event_type: str) -> None:
        self._send(StatusReport(
            robot_id=self.robot_id,
            report_type="arrival",
            waypoint_id=waypoint_id,
            beacon_id=beacon_id,
            event_type=event_type,
            message=f"Arrived at {waypoint_id} ({event_type})",
            timestamp=time.time(),
        ))

    def report_mission_complete(self, visited: int, total: int) -> None:
        self._send(StatusReport(
            robot_id=self.robot_id,
            report_type="mission_complete",
            waypoint_id=None,
            beacon_id=None,
            event_type=None,
            message=f"Mission complete: visited {visited}/{total} waypoints",
            timestamp=time.time(),
        ))

    def report_skipped(self, waypoint_id: str, beacon_id: str, reason: str = "ttl_expired") -> None:
        self._send(StatusReport(
            robot_id=self.robot_id,
            report_type="skipped",
            waypoint_id=waypoint_id,
            beacon_id=beacon_id,
            event_type=reason,
            message=f"Skipped {waypoint_id} ({beacon_id}): {reason}",
            timestamp=time.time(),
        ))

    def report_error(self, message: str) -> None:
        self._send(StatusReport(
            robot_id=self.robot_id,
            report_type="error",
            waypoint_id=None,
            beacon_id=None,
            event_type=None,
            message=message,
            timestamp=time.time(),
        ))

    def _send(self, report: StatusReport) -> None:
        self._report_count += 1
        self._queue.put(report.to_dict())
        logger.info(f"[StatusReporter] Report #{self._report_count}: {report.message}")

    @property
    def report_count(self) -> int:
        return self._report_count
