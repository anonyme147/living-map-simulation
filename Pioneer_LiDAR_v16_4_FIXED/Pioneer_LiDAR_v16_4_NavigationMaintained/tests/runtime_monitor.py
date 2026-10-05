#!/usr/bin/env python3
"""Live, non-invasive runtime monitor for the running viewer/backend.

It only GETs diagnostic/target endpoints. It never sends velocity, target, or
configuration commands. Keep it running in a separate terminal while the
robot/system is operating.
"""
from __future__ import annotations
import argparse
import json
import math
import sys
import time
import urllib.error
import urllib.request
from collections import deque


class Monitor:
    def __init__(self, base_url: str, interval: float, stale_s: float, probe_ratio_window: int):
        self.base = base_url.rstrip("/")
        self.interval = max(0.2, interval)
        self.stale_s = max(0.5, stale_s)
        self.prev_mode = None
        self.transitions = deque(maxlen=200)
        self.probe_hist = deque(maxlen=max(10, probe_ratio_window))
        self.prev_points = None
        self.prev_point_time = None
        self.start = time.monotonic()

    def get_json(self, path):
        req = urllib.request.Request(self.base + path, headers={"Accept": "application/json"})
        with urllib.request.urlopen(req, timeout=2.0) as r:
            return json.loads(r.read().decode("utf-8"))

    @staticmethod
    def finite(v):
        return isinstance(v, (int, float)) and math.isfinite(v)

    def emit(self, level, text):
        print(f"[{time.strftime('%H:%M:%S')}] [{level}] {text}", flush=True)

    def evaluate(self, payload):
        now = time.time()
        latest = payload.get("latest") or {}
        nav = latest.get("nav") or {}
        arb = latest.get("arbitration") or {}
        mode = nav.get("mode")
        target = nav.get("external_target")
        target_status = nav.get("external_target_status")
        path_cells = int(nav.get("path_cells") or 0)

        # 1. Telemetry freshness / WebSocket health.
        scan_age_ms = latest.get("scan_last_age_ms")
        ws_open = bool(latest.get("viewer_ws_open"))
        if not ws_open:
            self.emit("WARN", "viewer WebSocket is not OPEN")
        elif self.finite(scan_age_ms) and scan_age_ms > self.stale_s * 1000:
            self.emit("WARN", f"telemetry stale: {scan_age_ms:.0f} ms")

        # 2. State transition legality.
        if self.prev_mode is not None and mode != self.prev_mode:
            self.transitions.append((self.prev_mode, mode))
            self.emit("INFO", f"state transition {self.prev_mode} -> {mode}")
        self.prev_mode = mode

        allowed = {
            "STARTING": {"REPLAN", "NAVIGATING", "TARGET PENDING", "TARGET NAVIGATING", "OUTSIDE AREA", "BLOCKED", "COMPLETE"},
            "REPLAN": {"NAVIGATING", "TARGET NAVIGATING", "PROBING", "LOCAL AVOIDANCE", "BOUNDARY RECOVERY", "OUTSIDE AREA", "BLOCKED", "COMPLETE"},
            "NAVIGATING": {"REPLAN", "NAVIGATING", "LOCAL AVOIDANCE", "BOUNDARY RECOVERY", "OUTSIDE AREA", "PROBING", "COMPLETE", "BLOCKED"},
            "TARGET PENDING": {"TARGET NAVIGATING", "TARGET WAITING", "TARGET BLOCKED", "TARGET REACHED", "REPLAN", "OUTSIDE AREA"},
            "TARGET WAITING": {"TARGET WAITING", "TARGET NAVIGATING", "TARGET BLOCKED", "TARGET REACHED", "REPLAN"},
            "TARGET NAVIGATING": {"TARGET NAVIGATING", "LOCAL AVOIDANCE", "BOUNDARY RECOVERY", "OUTSIDE AREA", "TARGET REACHED", "TARGET BLOCKED", "REPLAN", "TARGET WAITING"},
            "LOCAL AVOIDANCE": {"LOCAL AVOIDANCE", "REPLAN", "TARGET NAVIGATING", "NAVIGATING", "BOUNDARY RECOVERY", "OUTSIDE AREA", "BLOCKED"},
            "BOUNDARY RECOVERY": {"BOUNDARY RECOVERY", "REPLAN", "TARGET NAVIGATING", "NAVIGATING", "OUTSIDE AREA"},
            "OUTSIDE AREA": {"BOUNDARY RECOVERY", "REPLAN", "TARGET NAVIGATING", "BLOCKED"},
            "PROBING": {"PROBING", "REPLAN", "LOCAL AVOIDANCE", "NAVIGATING", "TARGET PENDING", "TARGET WAITING", "TARGET NAVIGATING", "TARGET BLOCKED"},
            "TARGET REACHED": {"TARGET REACHED", "IDLE", "REPLAN"},
            "TARGET BLOCKED": {"TARGET BLOCKED", "IDLE", "REPLAN"},
            "COMPLETE": {"COMPLETE", "IDLE", "TARGET PENDING"},
            "BLOCKED": {"BLOCKED", "REPLAN", "IDLE", "TARGET PENDING"},
            "IDLE": {"IDLE", "STARTING", "TARGET PENDING"},
        }
        if self.transitions:
            a, b = self.transitions[-1]
            if a in allowed and b not in allowed[a]:
                self.emit("FAIL", f"illegal state transition observed: {a} -> {b}")

        # 3. Terminal target priority / no dual mode.
        if target is not None:
            if mode in {"PROBING", "NAVIGATING", "COMPLETE"} and target_status not in {"reached", "blocked"}:
                self.emit("FAIL", f"terminal target active but conflicting mode is {mode}")
            if mode == "TARGET NAVIGATING" and path_cells == 0 and target_status == "running":
                reason = nav.get("last_plan_reason")
                if reason and "NO CURRENT A* PATH" in reason:
                    self.emit("INFO", "target A* currently has no path; waiting for replan evidence")

        # 4. Explicit unknown-allowance check from planner diagnostics.
        ta = nav.get("target_astar") or {}
        if target is not None and target_status in {"pending", "running"}:
            if ta and ta.get("allowUnknown") is not True and ta.get("allow_unknown") is not True:
                self.emit("FAIL", "terminal target planner diagnostics do not report allowUnknown=true")

        # 5. Arbitration ownership consistency.
        owner = arb.get("owner")
        source = arb.get("source")
        if target is not None and target_status not in {"reached", "blocked"}:
            if owner not in {"terminal_target", "manual", "safety"}:
                self.emit("FAIL", f"terminal target active but command owner is {owner or 'none'}")
        if target is not None and target_status not in {"reached", "blocked"} and mode in {"PROBING", "NAVIGATING", "BOUNDARY RECOVERY", "LOCAL RECOVERY"}:
            self.emit("FAIL", f"terminal target active while fallback mode is {mode}")
        if source == "terminal_target" and mode == "PROBING":
            self.emit("FAIL", "terminal target source and probing mode overlap")

        validation = nav.get("target_validation") or {}
        flow = nav.get("target_flow") or {}
        if target is not None and target_status in {"pending", "running"}:
            stage = validation.get("stage") or "unknown"
            reason_code = validation.get("reasonCode") or ""
            self.emit("INFO", f"target pipeline stage={stage} reason={reason_code or 'none'} plan={flow.get('plan_result') or nav.get('last_plan_reason') or 'none'}")
        if target is not None and target_status in {"pending", "running"} and mode == "TARGET NAVIGATING":
            if validation.get("aStarEntered") is not True:
                self.emit("FAIL", "TARGET NAVIGATING without recorded A* entry")
            if validation.get("aStarAllowUnknown") is not True:
                self.emit("FAIL", "terminal target A* is navigating without UNKNOWN allowance")

        if mode == "TARGET INVALID":
            self.emit("FAIL", "legacy TARGET INVALID state detected; use TARGET BLOCKED/TARGET WAITING with target_validation diagnostics")
        if target is not None and target_status in {"pending", "running"} and owner == "terminal_target":
            if mode == "TARGET PENDING" and validation.get("stage") not in {None, "RECEIVED"}:
                self.emit("INFO", f"target pending at validation stage {validation.get('stage')}")

        # 6. Point cloud growth probe.
        points = int(latest.get("point_count") or 0)
        t = time.monotonic()
        if self.prev_points is not None:
            dt = t - self.prev_point_time
            if dt >= 1.0 and points == self.prev_points and bool(latest.get("viewer_ws_open")) and nav.get("active"):
                self.emit("INFO", "point cloud unchanged during active navigation; verify whether robot is stationary")
        self.prev_points = points
        self.prev_point_time = t

        # 7. Probe dominance.
        probes = int(nav.get("probe_count") or 0)
        self.probe_hist.append(probes)
        if len(self.probe_hist) >= 10:
            delta = self.probe_hist[-1] - self.probe_hist[0]
            if delta >= 8:
                self.emit("WARN", f"probe dominance rising: {delta} probe events in monitor window")

        # 8. Path validity from published diagnostics.
        last_safety = nav.get("last_safety") or {}
        if last_safety.get("clear") is False and path_cells > 0:
            self.emit("INFO", "local safety currently reports blocked corridor while a global path exists")

        self.emit("OK", f"mode={mode} target={target_status or 'none'} path={path_cells} points={points} owner={owner or 'none'}")

    def loop(self):
        self.emit("INFO", f"live monitor attached to {self.base} (read-only)")
        while True:
            try:
                payload = self.get_json("/api/runtime-tests/status")
                self.evaluate(payload)
            except urllib.error.HTTPError as exc:
                self.emit("WARN", f"runtime telemetry endpoint returned HTTP {exc.code}; is the viewer loaded?")
            except urllib.error.URLError as exc:
                self.emit("WARN", f"backend unreachable: {exc.reason}")
            except KeyboardInterrupt:
                self.emit("INFO", "stopped")
                return 0
            except Exception as exc:
                self.emit("WARN", f"monitor error: {exc}")
            time.sleep(self.interval)


def main():
    ap = argparse.ArgumentParser(description="Live non-invasive runtime verification monitor")
    ap.add_argument("--url", default="http://127.0.0.1:5000", help="viewer/backend base URL")
    ap.add_argument("--interval", type=float, default=1.0)
    ap.add_argument("--stale", type=float, default=2.0, help="telemetry stale threshold in seconds")
    args = ap.parse_args()
    return Monitor(args.url, args.interval, args.stale, 60).loop()


if __name__ == "__main__":
    sys.exit(main())
