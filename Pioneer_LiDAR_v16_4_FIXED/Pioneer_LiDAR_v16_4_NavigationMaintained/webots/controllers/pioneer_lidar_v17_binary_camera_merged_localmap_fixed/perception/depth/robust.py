import numpy as np
from ..roi import inner_roi


def robust_depth(bbox, depth_map, border_fraction=0.15, min_depth_m=0.05,
                 max_depth_m=80.0, depth_scale=1.0):
    values = np.asarray(inner_roi(bbox, depth_map, border_fraction), dtype=float).ravel()
    valid = values[np.isfinite(values) & (values > min_depth_m) & (values <= max_depth_m)]
    if valid.size == 0:
        return {"depth_m": None, "depth_valid": False, "valid_pixels": 0,
                "min": None, "max": None, "median": None, "iqr": None}
    q1, median, q3 = np.percentile(valid, [25, 50, 75])
    return {"depth_m": float(median * depth_scale), "depth_valid": True, "valid_pixels": int(valid.size),
            "min": float(valid.min() * depth_scale), "max": float(valid.max() * depth_scale),
            "median": float(median * depth_scale), "iqr": float((q3 - q1) * depth_scale),
            "raw_median": float(median), "scale": float(depth_scale)}
