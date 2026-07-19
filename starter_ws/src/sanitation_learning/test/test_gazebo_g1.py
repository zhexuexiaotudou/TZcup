import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from sanitation_learning.gazebo_g1 import write_g1_world


def test_g1_world_has_coview_sensors_and_all_semantic_labels(tmp_path):
    registry = Path(__file__).parents[1] / "config" / "asset_registry.yaml"
    world = tmp_path / "g1.sdf"
    manifest = write_g1_world(registry, world)
    ET.parse(world)
    text = world.read_text(encoding="utf-8")
    assert 'type="rgbd_camera"' in text
    assert text.count('type="segmentation"') == 2
    assert "<segmentation_type>semantic</segmentation_type>" in text
    assert "<segmentation_type>instance</segmentation_type>" in text
    assert {item["semantic_label"] for item in manifest["models"]} == set(range(6))
    assert len([item for item in manifest["models"] if item.get("class_id") != "background"]) == 30
    assert manifest["online_fuel_dependency"] is False
    assert manifest["world_sha256"] == hashlib.sha256(world.read_bytes()).hexdigest()
    loaded = json.loads(world.with_suffix(".manifest.json").read_text(encoding="utf-8"))
    assert loaded["dataset_domain"].startswith("G1_actual_gazebo_camera")
