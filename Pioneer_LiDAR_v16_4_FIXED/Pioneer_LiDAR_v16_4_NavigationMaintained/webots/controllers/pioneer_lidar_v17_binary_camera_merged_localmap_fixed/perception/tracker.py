"""Small, dependency-free multi-object tracker for camera detections.

The detector is run periodically and does not provide persistent identities.
This module keeps identities stable between detector frames so that a temporary
confidence or depth change cannot make two nearby objects exchange IDs.
"""

from dataclasses import dataclass
import math


@dataclass
class _Track:
    track_id: int
    class_name: str
    bbox: tuple
    missed: int = 0


class DetectionTracker:
    """Greedy global-assignment tracker suitable for a small number of objects."""

    def __init__(self, max_missed=4, min_iou=0.05, max_center_distance=1.25):
        self.max_missed = int(max_missed)
        self.min_iou = float(min_iou)
        self.max_center_distance = float(max_center_distance)
        self._tracks = []
        self._next_id = 1

    @staticmethod
    def _iou(a, b):
        ax1, ay1, ax2, ay2 = a
        bx1, by1, bx2, by2 = b
        ix1, iy1 = max(ax1, bx1), max(ay1, by1)
        ix2, iy2 = min(ax2, bx2), min(ay2, by2)
        intersection = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
        area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
        area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
        union = area_a + area_b - intersection
        return intersection / union if union else 0.0

    @staticmethod
    def _center_distance(a, b):
        acx, acy = (a[0] + a[2]) / 2.0, (a[1] + a[3]) / 2.0
        bcx, bcy = (b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0
        distance = math.hypot(acx - bcx, acy - bcy)
        diagonal = max(math.hypot(a[2] - a[0], a[3] - a[1]), 1.0)
        return distance / diagonal

    def update(self, detections):
        """Return ``[(detection, stable_track_id), ...]`` in input order."""
        detections = list(detections)
        candidates = []
        for track_index, track in enumerate(self._tracks):
            for detection_index, detection in enumerate(detections):
                if track.class_name != detection.class_name:
                    continue
                iou = self._iou(track.bbox, detection.bbox)
                distance = self._center_distance(track.bbox, detection.bbox)
                if iou >= self.min_iou or distance <= self.max_center_distance:
                    # Higher score is better. IoU dominates when boxes overlap;
                    # center distance keeps an object identifiable when its box
                    # changes shape or briefly has little overlap.
                    score = iou + max(0.0, 1.0 - distance) * 0.25
                    candidates.append((score, track_index, detection_index))

        matches = {}
        used_tracks = set()
        used_detections = set()
        for _, track_index, detection_index in sorted(candidates, reverse=True):
            if track_index in used_tracks or detection_index in used_detections:
                continue
            used_tracks.add(track_index)
            used_detections.add(detection_index)
            matches[detection_index] = track_index

        for track_index, track in enumerate(self._tracks):
            if track_index not in used_tracks:
                track.missed += 1

        result = []
        for detection_index, detection in enumerate(detections):
            if detection_index in matches:
                track = self._tracks[matches[detection_index]]
                track.bbox = detection.bbox
                track.missed = 0
            else:
                track = _Track(self._next_id, detection.class_name, detection.bbox)
                self._next_id += 1
                self._tracks.append(track)
            result.append((detection, track.track_id))

        self._tracks = [track for track in self._tracks if track.missed <= self.max_missed]
        return result
