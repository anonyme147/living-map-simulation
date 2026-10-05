"""Persistent LocateAnything worker.

The controller can keep this process alive when the native C API is not
available.  It loads the GGUF once, then accepts newline-delimited JSON:
{"image": "...", "prompt": "..."}

Each request produces one JSON response containing ``detections`` or an
``error``.  Image paths are used instead of base64 so camera frames do not
needlessly get copied through the controller pipe.
"""

import argparse
import json
import sys
from pathlib import Path

import cv2

try:
    from .locate_anything import LocateAnythingDetector
except ImportError:
    from locate_anything import LocateAnythingDetector


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cli", required=True)
    parser.add_argument("--model", required=True)
    parser.add_argument("--prompt", required=True)
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--mode", choices=("fast", "hybrid", "slow"), default="fast")
    parser.add_argument("--timeout-sec", type=float, default=900.0)
    args = parser.parse_args()
    detector = LocateAnythingDetector(args.cli, args.model, args.prompt,
                                       threads=args.threads,
                                       timeout_sec=args.timeout_sec,
                                       mode=args.mode)
    print(json.dumps({"ready": True}), flush=True)
    try:
        for line in sys.stdin:
            if not line.strip():
                continue
            try:
                request = json.loads(line)
                image_path = Path(request["image"])
                image = cv2.cvtColor(cv2.imread(str(image_path)), cv2.COLOR_BGR2RGB)
                detections = detector.detect(image)
                payload = {"detections": [
                    {"class": item.class_name, "confidence": item.confidence,
                     "bbox": list(item.bbox)} for item in detections
                ]}
            except Exception as exc:
                payload = {"error": str(exc)}
            print(json.dumps(payload), flush=True)
    finally:
        detector.close()


if __name__ == "__main__":
    main()
