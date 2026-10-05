from dataclasses import dataclass
import json
from pathlib import Path
import numpy as np


@dataclass(frozen=True)
class CameraCalibration:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    base_from_camera: np.ndarray

    @property
    def camera_from_base(self):
        return np.linalg.inv(self.base_from_camera)


def axis_angle_matrix(axis_angle):
    values = np.asarray(axis_angle, dtype=float)
    if values.shape != (4,):
        raise ValueError("axis-angle must contain x, y, z, angle")
    axis, angle = values[:3], values[3]
    norm = np.linalg.norm(axis)
    if norm == 0:
        return np.eye(3)
    axis = axis / norm
    x, y, z = axis
    c, s, t = np.cos(angle), np.sin(angle), 1 - np.cos(angle)
    return np.array([[t*x*x+c, t*x*y-s*z, t*x*z+s*y],
                     [t*x*y+s*z, t*y*y+c, t*y*z-s*x],
                     [t*x*z-s*y, t*y*z+s*x, t*z*z+c]])


def homogeneous(rotation, translation):
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = np.asarray(translation, dtype=float)
    return result


def load_geometry(path):
    path = Path(path)
    with path.open(encoding="utf-8") as stream:
        camera = json.load(stream)["camera"]
    intr = camera["intrinsics"]
    mast = homogeneous(axis_angle_matrix(camera["mount_rotation_axis_angle"]),
                       camera["mount_translation_m"])
    local = homogeneous(axis_angle_matrix(camera["camera_local_rotation_axis_angle"]),
                        camera["camera_local_translation_m"])
    # Camera optical coordinates are the camera-local Webots coordinates.
    return CameraCalibration(float(intr["fx"]), float(intr["fy"]),
                             float(intr["cx"]), float(intr["cy"]),
                             int(camera["width_px"]), int(camera["height_px"]),
                             mast @ local)
