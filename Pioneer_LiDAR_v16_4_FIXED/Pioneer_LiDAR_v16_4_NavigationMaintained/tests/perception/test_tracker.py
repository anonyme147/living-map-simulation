from dataclasses import replace
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "webots/controllers/pioneer_lidar_v17_binary_camera_merged_localmap_fixed"))

from perception.detector import Detection
from perception.tracker import DetectionTracker


def detection(x1, y1, x2, y2):
    return Detection(0, "person", 0.9, x1, y1, x2, y2)


def test_tracker_keeps_ids_when_detector_order_changes():
    tracker = DetectionTracker()
    first = tracker.update([detection(10, 10, 50, 100), detection(200, 10, 240, 100)])
    first_ids = {item[0].bbox: item[1] for item in first}

    second = tracker.update([detection(202, 11, 242, 101), detection(12, 11, 52, 101)])
    second_ids = {item[0].bbox: item[1] for item in second}

    assert second_ids[(202, 11, 242, 101)] == first_ids[(200, 10, 240, 100)]
    assert second_ids[(12, 11, 52, 101)] == first_ids[(10, 10, 50, 100)]


def test_tracker_does_not_match_different_classes():
    tracker = DetectionTracker()
    first = tracker.update([detection(10, 10, 50, 100)])[0][1]
    fire = replace(detection(11, 11, 51, 101), class_name="fire")
    assert tracker.update([fire])[0][1] != first
