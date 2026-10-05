"""Small planar EKF for Webots wheel odometry plus IMU yaw."""

import math
import numpy as np


def wrap_angle(angle):
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


class DifferentialDrivePoseEKF:
    """Estimate x/y/yaw from wheel increments and an IMU yaw measurement."""

    def __init__(self, position, yaw, wheel_radius, track_width):
        self.wheel_radius = float(wheel_radius)
        self.track_width = float(track_width)
        self.state = np.array([float(position[0]), float(position[1]), float(yaw)], dtype=float)
        self.covariance = np.diag([0.0025, 0.0025, 0.02 ** 2])

    def reset(self, position, yaw):
        self.state[:] = [float(position[0]), float(position[1]), float(yaw)]
        self.covariance[:] = np.diag([0.0025, 0.0025, 0.02 ** 2])

    def step(self, left_delta_ticks, right_delta_ticks, imu_yaw):
        dl = float(left_delta_ticks) * self.wheel_radius
        dr = float(right_delta_ticks) * self.wheel_radius
        distance = 0.5 * (dl + dr)
        wheel_delta_yaw = (dr - dl) / self.track_width
        x, y, yaw = self.state
        previous_yaw = yaw
        self.state[2] = wrap_angle(yaw + wheel_delta_yaw)

        q_distance = 0.0025 + 0.04 * abs(distance)
        q_yaw = 0.004 + 0.05 * abs(wheel_delta_yaw)
        self.covariance[0, 0] += q_distance
        self.covariance[1, 1] += q_distance
        self.covariance[2, 2] += q_yaw

        if imu_yaw is not None and math.isfinite(float(imu_yaw)):
            # Scalar yaw measurement update.
            measurement_variance = 0.035 ** 2
            innovation = wrap_angle(float(imu_yaw) - self.state[2])
            innovation_variance = self.covariance[2, 2] + measurement_variance
            gain = self.covariance[2, 2] / innovation_variance
            self.state[2] = wrap_angle(self.state[2] + gain * innovation)
            self.covariance[2, 2] *= (1.0 - gain)

        # Use the corrected heading for the translational increment. Using the
        # wheel-only heading here creates lateral position drift during turns
        # even when the IMU has already corrected the yaw estimate.
        corrected_delta_yaw = wrap_angle(self.state[2] - previous_yaw)
        mid_yaw = wrap_angle(previous_yaw + 0.5 * corrected_delta_yaw)
        self.state[0] = x + distance * math.cos(mid_yaw)
        self.state[1] = y + distance * math.sin(mid_yaw)

        return tuple(float(value) for value in self.state)
