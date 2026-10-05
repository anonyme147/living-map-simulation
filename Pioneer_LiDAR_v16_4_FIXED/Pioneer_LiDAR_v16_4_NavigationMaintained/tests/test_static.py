#!/usr/bin/env python3
"""Static architecture/runtime-plan checks. No robot, Flask, or browser required."""
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]


def read(rel):
    return (ROOT / rel).read_text(encoding="utf-8")


def read_if_exists(rel):
    path = ROOT / rel
    return path.read_text(encoding="utf-8") if path.exists() else ""


def check(name, cond, detail):
    status = "PASS" if cond else "FAIL"
    print(f"[{status}] {name}: {detail}")
    return cond


def main():
    nav = read("static/js/navigation.js")
    nav_map = read("static/js/nav-map.js")
    control = read("static/js/control.js")
    ui = read("static/js/ui-controller.js")
    backend = read("backend/routes/navigation.py")
    runtime_bp = read("backend/routes/runtime_tests.py")
    cfg = read("map_parameter.yaml")
    ok = True

    ok &= check("Terminal target unknown-space mode",
                "aStar(startCell, targetGoal, c, true)" in nav,
                "explicit terminal-target A* is invoked with allowUnknown=true")
    ok &= check("Terminal target priority",
                "if (N.externalTarget && N.externalTargetStatus !== 'reached')" in nav,
                "planner enters terminal-target branch before exploration")
    ok &= check("Terminal target cannot fall back to probe",
                "A terminal target is exclusive" in nav and "else if (N.externalTarget && N.externalTargetStatus !== 'reached')" in nav,
                "active target sends a stop instead of entering probe/recovery/exploration fallback")
    ok &= check("Local safety wins",
                "localPathCorridorBlocked(path, pose, c" in nav and "PATH INVALIDATED / REPLAN" in nav,
                "local corridor can invalidate a global path before command execution")
    ok &= check("Command arbitration exists",
                "COMMAND_PRIORITY" in control and "canIssueVelocity" in control,
                "viewer commands pass a common priority gate")
    ok &= check("Manual control is source-tagged",
                "sendVelocity(0.35, 0, 'manual'" in ui and "sendVelocity(0, 1.0, 'manual'" in ui,
                "keyboard commands are explicitly manual")
    controller = read_if_exists("webots/controllers/pioneer_lidar_v17_binary_camera_merged_localmap_fixed/pioneer_lidar_v17_binary_camera_merged_localmap_fixed.py")
    ok &= check("Robot-side arbitration exists",
                ("command_owner" in controller and "_command_allowed" in controller) or
                ("COMMAND_TIMEOUT_SEC" in controller and "_handle_command" in controller and "_apply_drive" in controller),
                "controller protects the final velocity slot or enforces its fail-safe watchdog")
    ok &= check("Target terminal states deactivate",
                'if status in {"reached", "blocked"}:\n            _TARGET["active"] = False' in backend,
                "reached/blocked targets do not remain active")
    ok &= check("Runtime telemetry endpoint",
                "@runtime_tests_bp.post(\"/telemetry\")" in runtime_bp and "@runtime_tests_bp.get(\"/status\")" in runtime_bp,
                "live monitor has a non-invasive telemetry channel")
    ok &= check("Runtime telemetry publisher",
                "publishRuntimeTestTelemetry" in nav and "/api/runtime-tests/telemetry" in nav,
                "viewer periodically publishes navigation diagnostics")
    ok &= check("Safety-critical watchdog configured",
                "command_watchdog_timeout_sec" in cfg,
                "configuration contains the robot command watchdog")
    ok &= check("Terminal target immediate ownership",
                "claimCommandOwnership('terminal_target', 1400)" in nav and "sendVelocity(0, 0, 'terminal_target', 1400)" in nav,
                "target handover claims the browser and robot command lease before planning")
    ok &= check("Exact requested-target arrival",
                "requestedTargetDistance(pose)" in nav and "requestedDistance !== null && requestedDistance <= c.goalReach" in nav,
                "arrival is evaluated against the requested target rather than a lookahead waypoint")
    ok &= check("Shortest safe A* metric",
                "octileDistance" in nav and "Math.SQRT2" in nav and "Prevent diagonal corner cutting" in nav,
                "target A* optimizes the exact 8-connected grid distance without crossing blocked corners")
    ok &= check("Terminal target approach",
                "terminalApproachDistance" in nav and "terminalApproach ? N.externalTarget" in nav,
                "near-target steering uses the exact requested target")
    ok &= check("Target heading alignment gate",
                "targetHeadingStopRad" in nav and "N.externalTargetStatus !== 'reached'" in nav and
                "Math.abs(err) > c.targetHeadingStopRad" in nav,
                "explicit target navigation rotates before translating when strongly misaligned")
    ok &= check("Target completion policy",
                "continueExplorationAfterTargetReached" in nav and
                "continue_exploration_after_target_reached" in cfg and
                "RESUMING FULL-MAP EXPLORATION" in nav and
                "N.externalTargetStatus === 'reached'" in nav and
                "must not" in nav and "active arrival constraint" in nav,
                "target completion can either hold TARGET REACHED or resume exploration from the next cycle")
    ok &= check("Tile-map heading visibility",
                "ctx.rotate(-state.navPose.theta)" in nav_map and "state.navPose.x), py(state.navPose.y)" in nav_map,
                "tile map shows robot heading and target bearing")
    ws = read("static/js/ws-client.js")
    ok &= check("Yaw-to-theta convention",
                "heading_offset_rad" in ws and "controllerYaw + headingOffset" in ws and "rawYaw" in ws and
                "heading_offset_rad: 1.5707963267948966" in cfg,
                "controller yaw is converted to theta=yaw+pi/2 with a preserved raw diagnostic")
    ok &= check("Controlled forward obstacle recovery",
                "stopAndRotate" in nav and "FORWARD ESCAPE / REPLAN" in nav and "const v =" in nav,
                "blocked corridors use reduced forward escape motion while turning")
    ok &= check("Projected motion safety",
                "projectedMotionBlocked" in nav and "PROJECTED SAFETY MARGIN" in nav,
                "forward commands are rejected when the projected footprint is unsafe")
    ok &= check("Target failure hold",
                "targetFailureHold" in nav and "TARGET BLOCKED" in nav,
                "failed terminal targets stop and wait for a new target instead of resuming probing")
    ok &= check("Target validation diagnostics",
                "aStarEntered" in nav and "aStarAllowUnknown" in nav and "target_validation" in nav,
                "runtime telemetry exposes target validation and A* entry diagnostics")

    ok &= check("No legacy TARGET INVALID state",
                "TARGET INVALID" not in nav,
                "target failures use TARGET BLOCKED/TARGET WAITING rather than the legacy invalid state")
    ok &= check("Target pipeline stages",
                "stage: 'RECEIVED'" in nav and "reasonCode" in nav and "target_flow" in nav,
                "runtime telemetry identifies area/grid/nearest-cell/A* stages and outcome")
    ok &= check("Target waiting is a legal state",
                "N.mode = 'TARGET WAITING'" in nav and '"TARGET WAITING"' in read("tests/runtime_monitor.py"),
                "temporary planning failures wait for new map evidence instead of probing")

    print("\nStatic result:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
