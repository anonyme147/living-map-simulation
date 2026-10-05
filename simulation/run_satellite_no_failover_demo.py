"""Baseline: satellite loss without the RF-mesh continuity solution."""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from simulation.run_demo import run_simulation
from outside_network.baseline_alert import BaselineAlertState

logger = logging.getLogger("satellite_no_failover_demo")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run satellite loss without RF mesh failover")
    parser.add_argument("--port", type=int, default=5008, help="Dashboard port (default: 5008)")
    parser.add_argument("--no-serve", action="store_true")
    return parser.parse_args()


def run_satellite_no_failover_demo(*, dashboard_port: int = 5008) -> bool:
    alert = BaselineAlertState(
        title="SATELLITE CONNECTION FAILED",
        detail="No RF mesh failover is active. Later beacon packets cannot reach Command Post and remain queued.",
    )
    logger.warning("[BaselineNoFailover] Satellite-only baseline: traffic after the second Command Post arrival has no alternate path")
    return run_simulation(
        cycle_num=1,
        dashboard_port=dashboard_port,
        max_beacons_target=6,
        satellite_failure_after_command_post_events=2,
        baseline_alert=alert,
    )


if __name__ == "__main__":
    args = _parse_args()
    success = run_satellite_no_failover_demo(dashboard_port=args.port)
    if not args.no_serve:
        logger.info("Satellite-only baseline dashboard remains available at http://127.0.0.1:%s", args.port)
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
    sys.exit(0 if success else 1)
