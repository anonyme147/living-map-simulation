import numpy as np


def pixel_to_camera(u, v, depth_m, calibration):
    z = float(depth_m)
    if not np.isfinite(z) or z <= 0:
        raise ValueError("depth must be finite and positive")
    # Webots cameras use FLU coordinates: +X forward, +Y left, +Z up.
    # Image u grows right and image v grows down, hence the two minus signs.
    return np.array([z,
                     -(float(u) - calibration.cx) * z / calibration.fx,
                     -(float(v) - calibration.cy) * z / calibration.fy], dtype=float)


def camera_to_robot(point_camera, calibration):
    point = np.r_[np.asarray(point_camera, dtype=float), 1.0]
    return (calibration.base_from_camera @ point)[:3]


def robot_to_world(point_robot, robot_rotation, robot_translation):
    return np.asarray(robot_rotation, dtype=float) @ np.asarray(point_robot, dtype=float) + np.asarray(robot_translation, dtype=float)
