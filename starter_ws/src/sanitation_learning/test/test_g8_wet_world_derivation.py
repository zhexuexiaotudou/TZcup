import importlib.util
import json
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[4] / "scripts" / "derive_rgdrv8_g8_wet_world.py"
SPEC = importlib.util.spec_from_file_location("derive_rgdrv8_g8_wet_world", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_wet_world_derivation_changes_pixel_source_and_preserves_split(tmp_path):
    source = tmp_path / "source"
    worlds = source / "worlds"
    worlds.mkdir(parents=True)
    world_id = "world_example"
    sdf = worlds / f"{world_id}.sdf"
    texture = worlds / "ground_textures/example.png"
    texture.parent.mkdir()
    texture.write_bytes(b"ground-texture")
    sdf.write_text(
        f'<sdf><world name="{world_id}"><light><diffuse>0.8 0.8 0.8 1</diffuse><direction>-0.2 0.1 -0.9</direction></light><model><roughness>0.90</roughness></model></world></sdf>\n',
        encoding="utf-8",
    )
    manifest = {
        "worlds": [{"world_id": world_id, "material_id": "dry", "lighting_family": "day", "ground_texture_path": "ground_textures/example.png", "path": sdf.name, "sha256": MODULE.sha256(sdf), "split_eligibility": ["val"]}],
        "assets": [], "negative_assets": [],
    }
    (worlds / "g4_world_manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    report = MODULE.derive(source, world_id, tmp_path / "derived")
    assert report["G8_WET_WORLD_DERIVATION_PASS"] is True
    derived = json.loads((tmp_path / "derived/worlds/g4_world_manifest.json").read_text())
    assert derived["worlds"][0]["split_eligibility"] == ["val"]
    text = (tmp_path / "derived/worlds" / derived["worlds"][0]["path"]).read_text()
    assert "<roughness>0.12</roughness>" in text
    assert world_id + "_rgdrv8_wet_specular" in text
    assert (tmp_path / "derived/worlds/ground_textures/example.png").read_bytes() == b"ground-texture"
