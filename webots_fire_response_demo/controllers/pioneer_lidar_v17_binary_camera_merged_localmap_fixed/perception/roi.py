import numpy as np


def inner_roi(bbox, depth_map, border_fraction=0.15):
    x1, y1, x2, y2 = map(float, bbox)
    height, width = depth_map.shape[:2]
    dx, dy = (x2 - x1) * border_fraction, (y2 - y1) * border_fraction
    left = max(0, int(np.ceil(x1 + dx)))
    top = max(0, int(np.ceil(y1 + dy)))
    right = min(width, int(np.floor(x2 - dx)))
    bottom = min(height, int(np.floor(y2 - dy)))
    return depth_map[top:bottom, left:right]
