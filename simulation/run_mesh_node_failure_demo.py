"""Separate demo: reroute an in-flight ONA Role 3 packet after a relay dies."""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from simulation.run_rf_mesh_demo import run_rf_mesh_demo


logger = logging.getLogger("mesh_node_failure_demo")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the RF mesh mid-relay node-failure demonstration")
    parser.add_argument("--port", type=int, default=5005, help="Dashboard port (default: 5005)")
    parser.add_argument("--nodes", type=int, default=6, help="Primary relay node count (default: 6, plus one backup)")
    parser.add_argument("--seed", type=int, default=42, help="Topology seed (default: 42)")
    parser.add_argument("--no-serve", action="store_true", help="Exit after the scenario instead of keeping the dashboard open")
    return parser.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    completed = run_rf_mesh_demo(
        dashboard_port=args.port,
        node_count=args.nodes,
        seed=args.seed,
        resilient_node_failure=True,
    )
    if completed and not args.no_serve:
        logger.info("Node-failure dashboard remains available at http://127.0.0.1:%s", args.port)
        try:
            while True:
                import time
                time.sleep(1)
        except KeyboardInterrupt:
            pass
    sys.exit(0 if completed else 1)
