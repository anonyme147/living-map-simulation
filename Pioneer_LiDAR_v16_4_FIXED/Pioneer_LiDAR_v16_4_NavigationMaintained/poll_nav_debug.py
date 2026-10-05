#!/usr/bin/env python3
"""Poll the live navigation telemetry a few times and print the numbers
needed to diagnose the 'circling' behavior: pose, heading error, distance
to goal, and the last velocity command actually sent.

Usage: python3 poll_nav_debug.py [--server http://127.0.0.1:5000] [--count 15] [--interval 0.4]
"""
import argparse
import json
import time
from urllib.request import urlopen


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--server", default="http://127.0.0.1:5000")
    p.add_argument("--count", type=int, default=15)
    p.add_argument("--interval", type=float, default=0.4)
    args = p.parse_args()

    url = args.server.rstrip("/") + "/api/runtime-tests/status"
    for _ in range(args.count):
        try:
            with urlopen(url, timeout=2.0) as r:
                data = json.loads(r.read().decode("utf-8"))
        except Exception as exc:
            print(f"fetch failed: {exc}")
            time.sleep(args.interval)
            continue

        nav = (data.get("latest") or {}).get("nav") or {}
        pose = nav.get("pose") or {}
        cmd = nav.get("last_command") or {}
        print(
            f"pose=({pose.get('x')!s:>8}, {pose.get('y')!s:>8}) "
            f"theta_deg={pose.get('theta_deg')!s:>8} | "
            f"err_deg={nav.get('last_heading_error_deg')!s:>8} "
            f"dist={nav.get('last_goal_distance')!s:>8} | "
            f"cmd v={cmd.get('v')!s:>8} omega={cmd.get('omega')!s:>8} sent={cmd.get('sent')} | "
            f"mode={nav.get('mode')} path_cells={nav.get('path_cells')}"
        )
        time.sleep(args.interval)


if __name__ == "__main__":
    main()
