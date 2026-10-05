import numpy as np

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "webots/controllers/pioneer_lidar_v17_binary_camera_merged_localmap_fixed"))

from perception.detector import Detection, YoloDetector


def test_pose_prompts_are_normalized_to_person():
    assert YoloDetector._canonical_class("lying person") == "person"
    assert YoloDetector._canonical_class("human lying on ground") == "person"
    assert YoloDetector._canonical_class("fire") == "fire"


def test_tiled_image_offsets_cover_the_frame():
    detector = object.__new__(YoloDetector)
    detector.tile_grid = 2
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    tiles = list(detector._tiled_images(image))
    assert len(tiles) == 4
    assert min(offset[0] for _, offset in tiles) == 0.0
    assert min(offset[1] for _, offset in tiles) == 0.0
    assert max(offset[0] for _, offset in tiles) > 0.0
    assert max(offset[1] for _, offset in tiles) > 0.0


def test_duplicate_tiled_person_boxes_are_merged():
    detector = object.__new__(YoloDetector)
    boxes = [
        Detection(0, "person", 0.82, 10, 10, 100, 200),
        Detection(1, "person", 0.71, 12, 12, 98, 198),
        Detection(2, "tree", 0.66, 10, 10, 100, 200),
    ]
    merged = detector._merge(boxes)
    assert len(merged) == 2
    assert merged[0].confidence == 0.82
