"""Baseline: keepalives are lost, but no heartbeat status is surfaced to operators."""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from outside_network.heartbeat import HeartbeatMonitor
from outside_network.baseline_alert import BaselineAlertState
from simulation.run_demo import run_simulation

logger = logging.getLogger("zombie_link_no_detection_demo")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run Zombie Link baseline without heartbeat detection")
    parser.add_argument("--port", type=int, default=5009, help="Dashboard port (default: 5009)")
    parser.add_argument("--no-serve", action="store_true")
    return parser.parse_args()


def run_zombie_link_no_detection_demo(*, dashboard_port: int = 5009) -> bool:
    alert = BaselineAlertState(
        title="HEARTBEAT CONFIRMATION FAILED",
        detail="Beacon data is still arriving, but without the trust-monitoring solution operators cannot verify the ONA ↔ Command Post link.",
    )
    monitor = HeartbeatMonitor(on_degraded=alert.trigger)
    logger.warning(
        "[BaselineNoHeartbeat] Keepalives will disappear, but the dashboard trust indicator is intentionally NOT connected"
    )
    return run_simulation(
        cycle_num=1,
        dashboard_port=dashboard_port,
        heartbeat_monitor=monitor,
        heartbeat_dashboard_visible=False,
        max_beacons_target=6,
        baseline_alert=alert,
    )


if __name__ == "__main__":
    args = _parse_args()
    success = run_zombie_link_no_detection_demo(dashboard_port=args.port)
    if not args.no_serve:
        logger.info("Zombie Link no-detection baseline dashboard remains available at http://127.0.0.1:%s", args.port)
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
    sys.exit(0 if success else 1)
