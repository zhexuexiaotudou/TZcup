import hashlib
import json
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from prepare_auto05r_factorized_capture import prepare_capture_root


def _base(tmp_path: Path) -> Path:
    base = tmp_path / "base"
    (base / "models" / "asset").mkdir(parents=True)
    (base / "models" / "asset" / "model.sdf").write_text("asset", encoding="utf-8")
    (base / "worlds" / "ground_textures").mkdir(parents=True)
    worlds = []
    for world_id, split in (
        ("world_g4_01_asphalt_campus", "train"),
        ("world_g4_09_light_paver_pedestrian", "val"),
    ):
        text = (
            f'<sdf><world name="{world_id}"><light><diffuse>1 1 1 1</diffuse>'
            '<direction>0 0 -1</direction></light><visual><pbr><metal>'
            f'<albedo_map>ground_textures/{world_id}.png</albedo_map>'
            '<roughness>0.92</roughness></metal></pbr></visual></world></sdf>\n'
        )
        path = base / "worlds" / f"{world_id}.sdf"
        path.write_text(text, encoding="utf-8")
        (base / "worlds" / "ground_textures" / f"{world_id}.png").write_bytes(b"png")
        worlds.append(
            {
                "world_id": world_id,
                "path": path.name,
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                "split_eligibility": [split],
                "material_id": "asphalt",
                "ground_texture_family": "asphalt",
                "ground_texture_path": f"ground_textures/{world_id}.png",
                "lighting_family": "noon",
            }
        )
    manifest = {
        "dataset_domain": "G4",
        "worlds": worlds,
        "assets": [],
        "negative_assets": [],
        "world_split_counts": {"train": 1, "val": 1},
    }
    (base / "worlds" / "g4_world_manifest.json").write_text(
        json.dumps(manifest), encoding="utf-8"
    )
    return base


@pytest.mark.parametrize("role", ("D1", "D2", "D3", "D4", "D5"))
def test_prepare_factorized_role(tmp_path: Path, role: str) -> None:
    output = tmp_path / role
    result = prepare_capture_root(_base(tmp_path), role, output)
    manifest = json.loads(
        (output / "worlds" / "g4_world_manifest.json").read_text(encoding="utf-8")
    )
    assert result["role"] == role
    assert manifest["worlds"][0]["split_eligibility"] == [role]
    assert manifest["factorized_diagnostic"]["single_factor_native_capture"] is True
    assert result["runner_environment"]["AUTO05R_DIAGNOSTIC_ROLE"] == role
    if role == "D5":
        assert result["runner_environment"]["AUTO05R_FORCE_NEGATIVE_ONLY"] == "1"
    if role in {"D3", "D4"}:
        assert manifest["worlds"][0]["world_id"].startswith("world_auto05r_")


def test_existing_output_is_rejected(tmp_path: Path) -> None:
    base = _base(tmp_path)
    output = tmp_path / "existing"
    output.mkdir()
    with pytest.raises(FileExistsError):
        prepare_capture_root(base, "D1", output)
