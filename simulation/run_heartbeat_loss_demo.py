"""Separate Zombie Link demo: data carries normally while keepalives disappear."""

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
from simulation.run_demo import run_simulation


logger = logging.getLogger("heartbeat_loss_demo")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the Zombie Link heartbeat-loss demonstration")
    parser.add_argument("--port", type=int, default=5007, help="Dashboard port (default: 5007)")
    parser.add_argument("--no-serve", action="store_true", help="Exit after the simulation instead of keeping the dashboard open")
    return parser.parse_args()


def run_heartbeat_loss_demo(*, dashboard_port: int = 5007) -> bool:
    monitor = HeartbeatMonitor(
        # The Writer deposits several beacons in quick succession.  These
        # timings cross the trust threshold while later *real* beacon packets
        # are still reaching Command Post, making the contrast visible.
        interval_s=0.5,
        threshold_s=2.0,
        normal_after_first_data_s=0.5,
        recovery_after_degraded_data_s=1.0,
        maximum_loss_s=10.0,
    )
    logger.info(
        "[ZombieLink] Starting: beacon CARRY remains normal; only ONA↔Command Post keepalives will be suppressed"
    )
    return run_simulation(
        cycle_num=1,
        dashboard_port=dashboard_port,
        heartbeat_monitor=monitor,
        # More Writer observations make a real post-threshold beacon arrival
        # likely while the heartbeat monitor is deliberately degraded.
        max_beacons_target=6,
    )


if __name__ == "__main__":
    args = _parse_args()
    success = run_heartbeat_loss_demo(dashboard_port=args.port)
    if success and not args.no_serve:
        logger.info("Zombie Link dashboard remains available at http://127.0.0.1:%s", args.port)
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
    sys.exit(0 if success else 1)
