"""Regression guards for the production perception contract."""
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
CONTROLLER = ROOT / "webots/controllers/pioneer_lidar_v17_binary_camera_merged_localmap_fixed/pioneer_lidar_v17_binary_camera_merged_localmap_fixed.py"


def test_no_fixed_depth_fallback():
    source = CONTROLLER.read_text(encoding="utf-8")
    assert "depth = 2.0" not in source
    assert "DepthAnythingV2Metric" in source


def test_shared_geometry_is_loaded_from_controllers_directory():
    source = CONTROLLER.read_text(encoding="utf-8")
    assert "controller_dir.parent / \"robot_geometry.json\"" in source


def test_failure_states_are_explicit():
    source = CONTROLLER.read_text(encoding="utf-8")
    assert '"depth_valid": False' in source
    assert '"localization_valid": False' in source


def test_detector_results_are_not_overwritten():
    source = CONTROLLER.read_text(encoding="utf-8")
    assert "raw_detections = self.detector.detect(rgb)" in source
    assert "for detection in raw_detections:" in source
