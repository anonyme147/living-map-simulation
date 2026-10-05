"""Separate dashboard demo: Webots Writer detection → real ONA dispatch → Webots fire response."""

from __future__ import annotations

import signal
import sys
import threading
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from simulation.run_demo import logger, run_simulation


def main() -> int:
    stop_event = threading.Event()

    def stop(*_args) -> None:
        logger.info("Webots fire-response demo shutdown requested")
        stop_event.set()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    success = run_simulation(
        cycle_num=1,
        dashboard_port=5012,
        webots_visualization_enabled=True,
        webots_fire_response_enabled=True,
    )
    # Preserve the final Command Post state after the scenario completes.
    while not stop_event.is_set():
        time.sleep(0.25)
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
