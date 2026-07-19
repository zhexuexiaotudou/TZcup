from pathlib import Path

from sanitation_learning.assets import load_asset_registry, registry_summary, write_gazebo_assets


REGISTRY = Path(__file__).parents[1] / "config" / "asset_registry.yaml"


def test_registry_has_six_variants_and_shared_palettes(tmp_path):
    payload = load_asset_registry(REGISTRY)
    summary = registry_summary(payload)
    assert summary["target_variant_count"] == 30
    assert summary["negative_asset_count"] >= 10
    assert not summary["fixed_class_color_encoding"]
    generated = write_gazebo_assets(REGISTRY, tmp_path)
    assert len(generated["generated_assets"]) == 30
    assert all((tmp_path / item["asset_id"] / "model.sdf").is_file() for item in generated["generated_assets"])
