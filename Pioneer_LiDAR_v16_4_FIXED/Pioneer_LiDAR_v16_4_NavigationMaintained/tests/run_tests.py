#!/usr/bin/env python3
from pathlib import Path
import argparse
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def main():
    ap = argparse.ArgumentParser(description="Pioneer LiDAR runtime verification launcher")
    ap.add_argument("--live", action="store_true", help="also start the read-only live monitor")
    ap.add_argument("--url", default="http://127.0.0.1:5000")
    ap.add_argument("--interval", type=float, default=1.0)
    args = ap.parse_args()

    print("=" * 72)
    print("Pioneer LiDAR Runtime Test & Verification")
    print("Static checks + optional live read-only monitor")
    print("=" * 72)

    static_rc = subprocess.call([sys.executable, str(ROOT / "tests" / "test_static.py")], cwd=ROOT)
    if static_rc != 0:
        print("\nStatic verification FAILED.")
        return static_rc

    if not args.live:
        print("\nStatic verification PASS. Run: python tests/run_tests.py --live")
        return 0

    monitor = ROOT / "tests" / "runtime_monitor.py"
    return subprocess.call([
        sys.executable, str(monitor),
        "--url", args.url,
        "--interval", str(args.interval),
    ], cwd=ROOT)


if __name__ == "__main__":
    raise SystemExit(main())
