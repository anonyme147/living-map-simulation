from pathlib import Path
import sys
import numpy as np
from .base import DepthModel, DepthModelUnavailable


class DepthAnythingV2Metric(DepthModel):
    """Adapter boundary for the official metric checkpoint.

    The checkpoint is intentionally required; relative-depth or fixed-depth
    substitutes are not valid localization inputs.
    """
    def __init__(self, checkpoint, encoder="vits", max_depth_m=80.0, device=None, implementation_root=None):
        checkpoint = Path(checkpoint)
        if not checkpoint.is_file():
            raise DepthModelUnavailable(f"metric depth model unavailable: {checkpoint}")
        try:
            import torch
        except ImportError as exc:
            raise DepthModelUnavailable("PyTorch is not installed") from exc
        self.max_depth_m = float(max_depth_m)
        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        # The upstream model implementation is supplied with the checkpoint.
        # Keep construction explicit so deployment can select its pinned API.
        try:
            if implementation_root:
                implementation_root = Path(implementation_root)
                if not (implementation_root / "metric_depth").is_dir():
                    extracted = implementation_root.parent / "Depth-Anything-V2-main"
                    if (extracted / "metric_depth").is_dir():
                        implementation_root = extracted
                implementation_path = implementation_root / "metric_depth"
                if str(implementation_path) not in sys.path:
                    sys.path.insert(0, str(implementation_path))
            from depth_anything_v2.dpt import DepthAnythingV2
            self.model = DepthAnythingV2(encoder=encoder, features=64, out_channels=[48, 96, 192, 384], max_depth=self.max_depth_m)
            try:
                state = torch.load(str(checkpoint), map_location="cpu", weights_only=True)
            except TypeError:  # Older Torch releases do not expose weights_only.
                state = torch.load(str(checkpoint), map_location="cpu")
            self.model.load_state_dict(state)
            self.model = self.model.to(self.device).eval()
        except Exception as exc:
            raise DepthModelUnavailable("Depth Anything V2 metric backend is not loadable") from exc

    def predict(self, rgb):
        depth = self.model.infer_image(np.asarray(rgb))
        return self.validate(depth, rgb.shape[:2], self.max_depth_m)
