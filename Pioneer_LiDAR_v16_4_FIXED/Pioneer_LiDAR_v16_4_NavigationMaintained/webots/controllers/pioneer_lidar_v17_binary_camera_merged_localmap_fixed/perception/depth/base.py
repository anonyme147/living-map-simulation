from abc import ABC, abstractmethod
import numpy as np


class DepthModelUnavailable(RuntimeError):
    pass


class DepthModel(ABC):
    @abstractmethod
    def predict(self, rgb):
        raise NotImplementedError

    @staticmethod
    def validate(depth_map, expected_shape, max_depth_m):
        array = np.asarray(depth_map, dtype=np.float32)
        if array.shape != tuple(expected_shape):
            raise ValueError(f"depth shape {array.shape} != {tuple(expected_shape)}")
        valid = np.isfinite(array) & (array > 0) & (array <= max_depth_m)
        if valid.mean() < 0.95:
            raise ValueError("depth map contains fewer than 95% valid pixels")
        return array
