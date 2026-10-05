"""Independent and controller-integration tests for GPS-denied localization."""

import ast
import math
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "webots/controllers/pioneer_lidar_v17_binary_camera_merged_localmap_fixed"))

from perception.ekf_pose import DifferentialDrivePoseEKF, wrap_angle
from perception.calibration import load_geometry
from perception.detector import Detection
from perception.fusion import fuse_detection


CONTROLLER = ROOT / (
    "webots/controllers/"
    "pioneer_lidar_v17_binary_camera_merged_localmap_fixed/"
    "pioneer_lidar_v17_binary_camera_merged_localmap_fixed.py"
)


def make_filter():
    return DifferentialDrivePoseEKF((0.0, 0.0), 0.0, 0.11, 0.394)


def test_straight_motion_has_correct_metric_scale():
    ekf = make_filter()
    wheel_angle = 0.01 / 0.11
    for _ in range(100):
        pose = ekf.step(wheel_angle, wheel_angle, 0.0)

    assert np.allclose(pose[:2], (1.0, 0.0), atol=1e-9)
    assert abs(pose[2]) < 1e-9


def test_turning_motion_uses_imu_corrected_midpoint_heading():
    ekf = make_filter()
    # Both wheels advance, while the IMU reports the corrected world yaw.
    for index in range(100):
        imu_yaw = 0.004 * (index + 1)
        pose = ekf.step(0.01 / 0.11, 0.012 / 0.11, imu_yaw)

    assert abs(wrap_angle(pose[2] - 0.4)) < 0.01
    assert math.isfinite(pose[0]) and math.isfinite(pose[1])


def test_imu_yaw_correction_prevents_wheel_yaw_drift():
    ekf = make_filter()
    # Deliberately introduce a large wheel differential error while the IMU
    # keeps reporting a fixed heading.
    for _ in range(100):
        pose = ekf.step(0.01 / 0.11, 0.03 / 0.11, 0.0)

    assert abs(pose[2]) < 0.01
    assert abs(pose[1]) < 0.03


def test_yaw_wraparound_is_continuous():
    ekf = DifferentialDrivePoseEKF((0.0, 0.0), math.pi - 0.01, 0.11, 0.394)
    pose = ekf.step(0.0, 0.0, -math.pi + 0.01)
    assert abs(wrap_angle(pose[2] - (-math.pi + 0.01))) < 0.01


def test_ekf_pose_can_feed_world_frame_fusion():
    ekf = make_filter()
    pose = ekf.step(0.5 / 0.11, 0.5 / 0.11, 0.0)
    geometry = load_geometry(ROOT / "webots/controllers/robot_geometry.json")
    detection = Detection(1, "person", 0.9, 300, 180, 340, 400)
    depth = np.full((480, 640), 4.0, dtype=np.float32)
    fused = fuse_detection(
        detection,
        depth,
        geometry,
        np.array([[math.cos(pose[2]), -math.sin(pose[2]), 0.0],
                  [math.sin(pose[2]), math.cos(pose[2]), 0.0],
                  [0.0, 0.0, 1.0]]),
        pose,
        1,
        pose_source="ekf",
    )
    assert fused["localization_valid"]
    assert fused["pose_source"] == "ekf"
    assert len(fused["position_world_frame"]) == 3


def test_controller_routes_all_runtime_pose_consumers_through_selector():
    tree = ast.parse(CONTROLLER.read_text(encoding="utf-8"))
    source = CONTROLLER.read_text(encoding="utf-8")
    assert "def _pose_in_use(self):" in source
    assert "pose_source" in source
    assert '"gps_denied": self.gps_denied' in source
    assert '"pose_source": self.pose_source' in source
    assert "capture_pose = self._pose_in_use()" in source

    functions = {node.name: node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)}
    lidar_source = ast.get_source_segment(source, functions["_get_lidar_points"])
    enqueue_source = ast.get_source_segment(source, functions["_enqueue_detection"])
    assert "_pose_in_use()" in lidar_source
    assert "capture_pose" in enqueue_source


def test_viewer_announces_gps_denied_transition():
    ws_source = (ROOT / "static/js/ws-client.js").read_text(encoding="utf-8")
    html_source = (ROOT / "static/index.html").read_text(encoding="utf-8")
    assert "updateGpsPoseStatus(data)" in ws_source
    assert "GPS DENIED ZONE entered" in ws_source
    assert "gps-mode" in html_source
