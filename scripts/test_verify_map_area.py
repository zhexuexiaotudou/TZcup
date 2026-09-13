import json
from pathlib import Path
import sys

from PIL import Image
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))

from verify_map_area import MapAreaError, assess_map, main, scan_maps


def _write_map(
    root: Path,
    *,
    name: str = "map",
    mode: str = "trinary",
    resolution: float = 0.5,
    pixels: tuple[int, ...] = (0, 205, 254),
) -> Path:
    root.mkdir(parents=True, exist_ok=True)
    image = Image.new("L", (len(pixels), 1))
    image.putdata(pixels)
    image.save(root / f"{name}.pgm")
    metadata = {
        "image": f"{name}.pgm",
        "mode": mode,
        "resolution": resolution,
        "origin": [0.0, 0.0, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    yaml_path = root / f"{name}.yaml"
    yaml_path.write_text(yaml.safe_dump(metadata), encoding="utf-8")
    return yaml_path


def test_exact_known_area_counts_free_and_occupied_only(tmp_path: Path):
    yaml_path = _write_map(
        tmp_path,
        pixels=(0, 205, 254, 100, 230, 255),
        resolution=0.5,
    )
    report = assess_map(yaml_path, root=tmp_path, minimum_area_m2=1.0)
    assert report["occupied_cells"] == 1
    assert report["free_cells"] == 3
    assert report["unknown_cells"] == 2
    assert report["known_cells"] == 4
    assert report["known_area_m2"] == 1.0
    assert report["unknown_area_m2"] == 0.5
    assert report["area_gate"] is True


def test_205_is_unknown_even_when_free_threshold_would_match(tmp_path: Path):
    yaml_path = _write_map(tmp_path, pixels=(205,))
    report = assess_map(yaml_path, root=tmp_path, minimum_area_m2=1.0)
    assert report["free_cells"] == 0
    assert report["unknown_cells"] == 1
    assert report["known_area_m2"] == 0.0
    assert report["area_gate"] is False
    assert report["required_isotropic_linear_scale"] is None


def test_scan_excludes_scale_masks_and_ranks_largest_known_area(tmp_path: Path):
    passing = _write_map(
        tmp_path / "passing",
        pixels=(0,) * 8,
        resolution=0.5,
    )
    closest = _write_map(
        tmp_path / "closest",
        pixels=(254,) * 3,
        resolution=0.5,
    )
    _write_map(tmp_path / "mask", mode="scale", pixels=(205,) * 8)
    report = scan_maps(tmp_path, minimum_area_m2=1.5)
    assert report["map_count"] == 2
    assert report["pass_count"] == 1
    assert report["pass"] is True
    assert report["largest_known_area_map_yaml"] == "passing/map.yaml"
    assert report["largest_known_area_m2"] == 2.0
    assert {row["map_yaml"] for row in report["maps"]} == {
        "closest/map.yaml",
        "passing/map.yaml",
    }
    assert report["excluded_maps"] == [
        {
            "map_yaml": "mask/map.yaml",
            "mode": "scale",
            "reason": "not a trinary occupancy map",
        }
    ]


def test_rejects_non_trinary_and_invalid_thresholds(tmp_path: Path):
    mask = _write_map(tmp_path, mode="scale")
    with pytest.raises(MapAreaError, match="unsupported map mode"):
        assess_map(mask, root=tmp_path)

    yaml_path = _write_map(tmp_path / "thresholds")
    metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    metadata["free_thresh"] = 0.9
    yaml_path.write_text(yaml.safe_dump(metadata), encoding="utf-8")
    with pytest.raises(MapAreaError, match="free_thresh must be less"):
        assess_map(yaml_path, root=tmp_path)


def test_cli_writes_deterministic_json_and_fails_require_pass(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
):
    _write_map(tmp_path, pixels=(0, 205, 254), resolution=0.5)
    output = tmp_path / "report.json"
    assert (
        main(
            [
                "--root",
                str(tmp_path),
                "--minimum-area-m2",
                "1.0",
                "--output",
                str(output),
            ]
        )
        == 0
    )
    first = output.read_text(encoding="utf-8")
    assert (
        main(
            [
                "--root",
                str(tmp_path),
                "--minimum-area-m2",
                "1.0",
                "--output",
                str(output),
            ]
        )
        == 0
    )
    assert output.read_text(encoding="utf-8") == first
    assert capsys.readouterr().out == ""
    assert json.loads(first)["largest_known_area_m2"] == 0.5

    assert (
        main(
            [
                "--root",
                str(tmp_path),
                "--minimum-area-m2",
                "1.0",
                "--require-pass",
            ]
        )
        == 2
    )
    assert json.loads(capsys.readouterr().out)["pass"] is False
