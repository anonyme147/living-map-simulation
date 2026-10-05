"""Reset only Living Map demo processes, then start a fresh launchpad.

Run from the project root when an older menu/dashboard is still occupying a
demo port or showing an already-completed cycle:

    python simulation/reset_demo_environment.py

Use --dry-run to see exactly which processes would be stopped first.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEMO_SCRIPT_NAMES = (
    "run_demo.py",
    "run_failure_demo.py",
    "run_writer_loss_continuity_demo.py",
    "run_rf_mesh_demo.py",
    "run_webots_fire_response_demo.py",
    "demo_menu.py",
)


def _matching_demo_processes() -> list[dict]:
    """Return only Python processes whose command line names a known demo script."""
    script_checks = " -or ".join(
        f"$_.CommandLine -match [regex]::Escape('{script}')" for script in DEMO_SCRIPT_NAMES
    )
    command = (
        "Get-CimInstance Win32_Process | "
        "Where-Object { $_.Name -eq 'python.exe' -and $_.CommandLine -and ("
        f"{script_checks}"
        ") } | Select-Object ProcessId, CommandLine | ConvertTo-Json -Compress"
    )
    result = subprocess.run(
        ["powershell", "-NoProfile", "-Command", command],
        capture_output=True,
        text=True,
        check=True,
    )
    raw = result.stdout.strip()
    if not raw:
        return []
    entries = json.loads(raw)
    return entries if isinstance(entries, list) else [entries]


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Reset Living Map demos and start a fresh menu")
    parser.add_argument("--dry-run", action="store_true", help="Show matching demo processes without stopping them")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    processes = [
        process for process in _matching_demo_processes()
        if int(process["ProcessId"]) != os.getpid()
    ]
    if processes:
        print("Living Map demo processes found:")
        for process in processes:
            print(f"  PID {process['ProcessId']}: {process['CommandLine']}")
    else:
        print("No existing Living Map demo processes found.")

    if args.dry_run:
        print("Dry run only: no process was stopped.")
        return 0

    for process in processes:
        os.kill(int(process["ProcessId"]), signal.SIGTERM)

    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        if not _matching_demo_processes():
            break
        time.sleep(0.1)

    remaining = _matching_demo_processes()
    if remaining:
        print("Could not close every old demo process. Close the listed process(es) in Task Manager, then retry.")
        return 1

    creation_flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
    subprocess.Popen(
        [sys.executable, str(PROJECT_ROOT / "simulation" / "demo_menu.py")],
        cwd=PROJECT_ROOT,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        creationflags=creation_flags,
    )
    print("Fresh demo menu started: http://127.0.0.1:5050")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
