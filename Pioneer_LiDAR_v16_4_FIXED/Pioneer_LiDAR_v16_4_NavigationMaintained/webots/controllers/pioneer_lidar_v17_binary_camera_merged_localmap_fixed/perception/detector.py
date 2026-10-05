from dataclasses import dataclass
from pathlib import Path

import numpy as np


class DetectorUnavailable(RuntimeError):
    pass


@dataclass(frozen=True)
class Detection:
    class_id: int
    class_name: str
    confidence: float
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def bbox(self):
        return (self.x1, self.y1, self.x2, self.y2)


class YoloDetector:
    """YOLO-World detector with a small-object/pose fallback.

    A person lying on the ground is a difficult zero-shot prompt for a
    generic detector: the person is wide and short, and at the camera's
    640x480 resolution may occupy very few useful features.  The normal
    full-frame pass is retained for speed.  If it finds no person, a 2x2
    tiled pass is run with person-pose synonyms and a lower candidate
    threshold.  Results are projected back into full-frame coordinates and
    merged before being returned.
    """

    PERSON_PROMPTS = {
        "person", "lying person", "person lying down", "human lying on ground",
        "casualty", "human body"
    }

    def __init__(self, model_path, confidence=0.3, image_size=640, classes=None,
                 candidate_confidence=0.05, tile_grid=2, enable_tiled_fallback=True):
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise DetectorUnavailable("ultralytics is not installed") from exc
        path = Path(model_path)
        if not path.is_file():
            raise DetectorUnavailable(f"detection model unavailable: {path}")
        try:
            self.model = YOLO(str(path))
        except Exception as exc:
            raise DetectorUnavailable(f"detection model cannot be loaded: {path}") from exc
        self.prompts = list(classes or ["person"])
        if classes:
            try:
                self.model.set_classes(self.prompts)
            except Exception as exc:
                raise DetectorUnavailable("YOLO-World custom classes could not be configured") from exc
        self.confidence = float(confidence)
        self.candidate_confidence = float(candidate_confidence)
        self.image_size = int(image_size)
        self.tile_grid = max(1, int(tile_grid))
        self.enable_tiled_fallback = bool(enable_tiled_fallback)

    @classmethod
    def _canonical_class(cls, name):
        lowered = str(name).strip().lower()
        if lowered in cls.PERSON_PROMPTS or "person" in lowered or "human" in lowered or "casualty" in lowered:
            return "person"
        return lowered

    @staticmethod
    def _iou(a, b):
        ax1, ay1, ax2, ay2 = a
        bx1, by1, bx2, by2 = b
        ix1, iy1 = max(ax1, bx1), max(ay1, by1)
        ix2, iy2 = min(ax2, bx2), min(ay2, by2)
        inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
        area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
        area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
        union = area_a + area_b - inter
        return inter / union if union > 0.0 else 0.0

    def _predict(self, image, offset=(0.0, 0.0), confidence=None):
        detections = []
        ox, oy = offset
        threshold = self.candidate_confidence if confidence is None else confidence
        results = self.model.predict(image, imgsz=self.image_size, conf=threshold, verbose=False)
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(float, box.xyxy[0])
                class_id = int(box.cls[0])
                names = self.model.names
                prompt_name = names[class_id] if isinstance(names, dict) else names[class_id]
                class_name = self._canonical_class(prompt_name)
                confidence_value = float(box.conf[0])
                if confidence_value < self.confidence:
                    continue
                detections.append(Detection(
                    class_id, class_name, confidence_value,
                    x1 + ox, y1 + oy, x2 + ox, y2 + oy
                ))
        return detections

    def _tiled_images(self, rgb):
        height, width = rgb.shape[:2]
        # 10% overlap prevents a person on a tile boundary from being split.
        overlap = 0.10
        tile_w = int(np.ceil(width / self.tile_grid * (1.0 + overlap)))
        tile_h = int(np.ceil(height / self.tile_grid * (1.0 + overlap)))
        for row in range(self.tile_grid):
            for col in range(self.tile_grid):
                x0 = min(width - tile_w, int(col * width / self.tile_grid))
                y0 = min(height - tile_h, int(row * height / self.tile_grid))
                x1, y1 = min(width, x0 + tile_w), min(height, y0 + tile_h)
                yield rgb[y0:y1, x0:x1], (float(x0), float(y0))

    def _merge(self, detections):
        merged = []
        for detection in sorted(detections, key=lambda item: item.confidence, reverse=True):
            duplicate = any(
                detection.class_name == existing.class_name and
                self._iou(detection.bbox, existing.bbox) >= 0.50
                for existing in merged
            )
            if not duplicate:
                merged.append(detection)
        return merged

    def detect(self, rgb):
        rgb = np.asarray(rgb)
        detections = self._predict(rgb, confidence=self.confidence)
        has_person = any(item.class_name == "person" for item in detections)
        if self.enable_tiled_fallback and not has_person and self.tile_grid > 1:
            for tile, offset in self._tiled_images(rgb):
                detections.extend(self._predict(tile, offset=offset,
                                                confidence=self.candidate_confidence))
        return self._merge(detections)
