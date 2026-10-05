from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class Pose:
    position: tuple
    rotation: np.ndarray
    source: str = "odometry"


class PoseProvider:
    """Pose boundary; ground truth is an explicit test-only source."""
    def __init__(self, position, rotation, source="odometry"):
        self._pose = Pose(tuple(float(v) for v in position), np.asarray(rotation, dtype=float), source)

    def get_pose(self):
        return self._pose
