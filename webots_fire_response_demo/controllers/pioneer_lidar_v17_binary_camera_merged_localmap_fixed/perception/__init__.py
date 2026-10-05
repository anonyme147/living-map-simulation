"""Testable RGB detection, metric depth, and world-localization components."""

from .calibration import CameraCalibration, load_geometry
from .geometry import pixel_to_camera, camera_to_robot, robot_to_world
from .roi import inner_roi
from .fusion import fuse_detection
from .pose_provider import Pose, PoseProvider

__all__ = [
    "CameraCalibration", "load_geometry", "pixel_to_camera",
    "camera_to_robot", "robot_to_world", "inner_roi", "fuse_detection", "Pose", "PoseProvider",
]
