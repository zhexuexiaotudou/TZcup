from pathlib import Path

from PIL import Image
import yaml

from sanitation_tasks.map_quality import inspect_map


def test_trinary_205_pixel_is_unknown_not_free(tmp_path: Path):
    Image.new("L", (3, 1)).save(tmp_path / "map.pgm")
    image = Image.open(tmp_path / "map.pgm").convert("L")
    image.putdata([0, 205, 254])
    image.save(tmp_path / "map.pgm")
    metadata = {
        "image": "map.pgm",
        "resolution": 1.0,
        "origin": [0.0, 0.0, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    (tmp_path / "map.yaml").write_text(yaml.safe_dump(metadata), encoding="utf-8")
    report = inspect_map(tmp_path / "map.yaml")
    assert report["occupied_cells"] == 1
    assert report["unknown_cells"] == 1
    assert report["free_cells"] == 1
