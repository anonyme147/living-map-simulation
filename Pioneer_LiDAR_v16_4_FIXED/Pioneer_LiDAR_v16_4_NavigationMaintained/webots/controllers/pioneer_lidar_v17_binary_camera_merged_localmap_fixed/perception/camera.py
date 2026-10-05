import numpy as np


class Camera:
    """Small Webots adapter; image normalization is kept out of the controller."""
    def __init__(self, device):
        self.device = device
        self.width = int(device.getWidth())
        self.height = int(device.getHeight())

    def enable(self, timestep):
        self.device.enable(timestep)

    def capture_rgb(self):
        raw = self.device.getImage()
        if raw is None:
            raise RuntimeError("camera returned no image")
        array = np.frombuffer(raw, dtype=np.uint8)
        channels = array.size // (self.width * self.height)
        if channels not in (3, 4):
            raise ValueError(f"unsupported camera buffer with {channels} channels")
        image = array.reshape(self.height, self.width, channels)
        # Webots returns BGRA/BGR; downstream models consume RGB.
        return image[..., :3][:, :, ::-1].copy()
