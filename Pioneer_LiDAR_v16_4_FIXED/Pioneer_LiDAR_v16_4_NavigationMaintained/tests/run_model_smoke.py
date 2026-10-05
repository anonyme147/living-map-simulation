#!/usr/bin/env python3
"""Load both production checkpoints and run one inference through each wrapper."""
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "webots/controllers/pioneer_lidar_v17_binary_camera_merged_localmap_fixed"))

from perception.detector import YoloDetector  # noqa: E402
from perception.depth.depth_anything_v2 import DepthAnythingV2Metric  # noqa: E402


def main():
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    detector = YoloDetector(ROOT / "models/detection/yolov8s-world.pt",
                            classes=["person", "fire", "house", "tree"])
    detections = detector.detect(image)
    depth = DepthAnythingV2Metric(
        ROOT / "models/depth/depth_anything_v2_metric_vkitti_vits.pth",
        implementation_root=ROOT / "vendor/Depth-Anything-V2-main")
    depth_map = depth.predict(image)
    assert depth_map.shape == (480, 640)
    assert np.isfinite(depth_map).all() and (depth_map > 0).all()
    print("DETECTOR_READY classes=person,fire,house,tree detections=%d" % len(detections))
    print("DEPTH_READY shape=%s range=(%.3f, %.3f)" %
          (depth_map.shape, float(depth_map.min()), float(depth_map.max())))


if __name__ == "__main__":
    main()
