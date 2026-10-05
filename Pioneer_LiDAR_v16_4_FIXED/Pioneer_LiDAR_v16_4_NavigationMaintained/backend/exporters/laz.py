"""LAS/LAZ export only."""

import io

try:
    import laspy
    import numpy as np
    HAS_LASPY = True
except ImportError:
    laspy = None
    np = None
    HAS_LASPY = False


def export_to_laz(decoded: dict) -> bytes:
    """Convert decoded LMAP data to compressed LAZ bytes."""
    if not HAS_LASPY:
        raise RuntimeError(
            "laspy is not installed. Run: pip install laspy[lazrs]"
        )

    positions = decoded["positions"]
    n = len(positions) // 3

    if n == 0:
        raise ValueError("No points to export")

    header = laspy.LasHeader(point_format=3, version="1.2")
    header.scales = np.array([0.001, 0.001, 0.001])
    header.offsets = np.array([0.0, 0.0, 0.0])

    las = laspy.LasData(header)
    las.x = np.array(positions[0::3])
    las.y = np.array(positions[1::3])
    las.z = np.array(positions[2::3])

    z_vals = np.array(positions[2::3])
    z_min, z_max = z_vals.min(), z_vals.max()
    rng = z_max - z_min if z_max != z_min else 1.0
    las.intensity = (
        ((z_vals - z_min) / rng * 65535).astype(np.uint16)
    )

    r = np.zeros(n, dtype=np.uint16)
    g = np.zeros(n, dtype=np.uint16)
    b = np.zeros(n, dtype=np.uint16)

    for i in range(n):
        z = positions[i*3 + 2]

        if z < -0.5:
            r[i], g[i], b[i] = 40000, 8000, 60000
        elif z < 0.0:
            r[i], g[i], b[i] = 32000, 0, 32000
        elif z < 0.05:
            r[i], g[i], b[i] = 20000, 0, 0
        elif z < 0.10:
            r[i], g[i], b[i] = 52000, 0, 0
        elif z < 0.15:
            r[i], g[i], b[i] = 65535, 13000, 0
        elif z < 0.20:
            r[i], g[i], b[i] = 65535, 26000, 0
        elif z < 0.50:
            r[i], g[i], b[i] = 65535, 65535, 0
        elif z < 1.00:
            r[i], g[i], b[i] = 0, 65535, 0
        else:
            r[i], g[i], b[i] = 0, 34000, 65535

    las.red = r
    las.green = g
    las.blue = b

    buf = io.BytesIO()
    las.write(buf, do_compress=True)
    return buf.getvalue()
