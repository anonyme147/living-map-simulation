#!/usr/bin/env python3
"""Dependency-light acceptance tests for the perception math and failure states."""
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CONTROLLER = ROOT / "webots" / "controllers" / "pioneer_lidar_v17_binary_camera_merged_localmap_fixed"
sys.path.insert(0, str(CONTROLLER))

from perception.calibration import load_geometry  # noqa: E402
from perception.geometry import pixel_to_camera, camera_to_robot  # noqa: E402
from perception.roi import inner_roi  # noqa: E402
from perception.depth.robust import robust_depth  # noqa: E402
from perception.fusion import fuse_detection  # noqa: E402
from perception.detector import Detection  # noqa: E402


def check(name, condition):
    if not condition:
        raise AssertionError(name)
    print(f"[PASS] {name}")


def main():
    calibration = load_geometry(ROOT / "webots" / "controllers" / "robot_geometry.json")
    check("geometry path and intrinsics", (calibration.width, calibration.height) == (640, 480))
    center = pixel_to_camera(320, 240, 2.0, calibration)
    check("principal point", np.allclose(center, [2, 0, 0]))
    check("horizontal sign", pixel_to_camera(400, 240, 2, calibration)[1] < 0 and pixel_to_camera(240, 240, 2, calibration)[1] > 0)
    check("vertical sign", pixel_to_camera(320, 320, 2, calibration)[2] < 0 and pixel_to_camera(320, 160, 2, calibration)[2] > 0)
    check("composed camera transform", np.allclose(calibration.base_from_camera[:3, 3], [0.424, 0.107, 0.434], atol=0.01))
    point = pixel_to_camera(410, 175, 3.2, calibration)
    reproj = np.array([calibration.cx - calibration.fx * point[1] / point[0],
                       calibration.cy - calibration.fy * point[2] / point[0]])
    check("reprojection", np.allclose(reproj, [410, 175], atol=1))
    depth = np.full((480, 640), 4.0, dtype=np.float32)
    depth[180:220, 230:270] = np.nan
    depth[190, 240] = 0
    stats = robust_depth((180, 130, 320, 270), depth)
    check("robust ROI depth", stats["depth_valid"] and abs(stats["depth_m"] - 4) < 1e-6)
    bad = np.full((480, 640), np.nan, dtype=np.float32)
    stats = robust_depth((0, 0, 100, 100), bad)
    check("invalid depth failure", not stats["depth_valid"] and stats["depth_m"] is None)
    detection = Detection(0, "person", 0.9, 300, 180, 340, 400)
    fused = fuse_detection(detection, depth, calibration, np.eye(3), [0, 0, 0], 1)
    check("valid fusion packet", fused["depth_valid"] and fused["localization_valid"] and "position_world_frame" in fused)
    failed = fuse_detection(detection, bad, calibration, np.eye(3), [0, 0, 0], 2)
    check("failed fusion has no fake position", not failed["depth_valid"] and not failed["localization_valid"] and "position_world_frame" not in failed)
    robot = np.array([1.1923502236240688, 20.2235995197826, -0.0006052368726188473])
    person = np.array([-3.66286, 26.3144, 1.27])
    angle = 2.36171
    rotation = np.array([[np.cos(angle), -np.sin(angle), 0],
                         [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
    expected_robot = rotation.T @ (person - robot)
    expected_camera = calibration.camera_from_base @ np.r_[expected_robot, 1.0]
    check("known Webots target camera coordinates",
          np.allclose(expected_camera[:3], [7.31246, -1.02325, 0.82877], atol=0.01))
    print("Perception result: PASS")


if __name__ == "__main__":
    main()
