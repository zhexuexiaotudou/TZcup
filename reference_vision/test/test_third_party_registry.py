from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]


def test_dependency_registry_is_pinned_and_product_fail_closed():
    registry = yaml.safe_load((ROOT / "third_party/perception/dependency_registry.yaml").read_text(encoding="utf-8"))
    assert {item["name"] for item in registry["dependencies"]} == {
        "Grounding DINO", "Grounded SAM 2", "SAM 2", "YOLO-World"
    }
    for item in registry["dependencies"]:
        assert len(item["upstream_commit"]) == 40
        assert item["license"]
        assert item["shipped_in_product"] is False
    yolo = next(item for item in registry["dependencies"] if item["name"] == "YOLO-World")
    assert yolo["license"] == "GPL-3.0"
    assert "benchmark_only" in yolo["redistribution_policy"]


def test_external_dataset_is_training_only_and_not_ingested():
    registry = yaml.safe_load((ROOT / "third_party/perception/dataset_registry.yaml").read_text(encoding="utf-8"))
    taco = registry["datasets"][0]
    assert taco["ingested"] is False
    assert taco["split_policy"] == "external_train_only_no_sealed_final"
    assert taco["dataset_license"] == "per_image_metadata_audit_required"
