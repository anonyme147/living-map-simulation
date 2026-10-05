from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
VIEWER = ROOT / "static/js/detection-viz.js"


def test_detection_spheres_use_viewer_world_transform():
    source = VIEWER.read_text(encoding="utf-8")
    assert "import { enuToThree } from './utils.js';" in source
    assert "state.areaGroup.add(sprite)" in source
    assert "sprite.position.copy(enuToThree(x, y, z))" in source
    assert "state.scene.add(sprite)" not in source
