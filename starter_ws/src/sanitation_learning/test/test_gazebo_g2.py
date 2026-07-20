import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

from sanitation_learning.gazebo_g2 import write_g2_worlds


ROOT = Path(__file__).resolve().parents[2]


def test_g2_worlds_are_distinct_deployment_aligned_and_unscaled(tmp_path):
    registry = ROOT / "sanitation_learning" / "config" / "asset_registry.yaml"
    xacro = ROOT / "sanitation_vehicle_description" / "urdf" / "sanitation_vehicle.urdf.xacro"
    manifest = write_g2_worlds(registry, xacro, tmp_path)
    assert len(manifest["worlds"]) == 6
    assert len({world["sha256"] for world in manifest["worlds"]}) == 6
    assert len({world["material_id"] for world in manifest["worlds"]}) == 6
    assert len({world["layout_family"] for world in manifest["worlds"]}) == 6
    assert {tuple(world["split_eligibility"]) for world in manifest["worlds"]} == {("train",), ("val",), ("test",)}
    assert manifest["camera_contract"]["extrinsics"]["xyz_m"] == [0.53, 0.0, 0.22]
    assert manifest["training_only_ground_truth"] is True
    assert all(asset["scale_factor"] == 1.0 for asset in manifest["assets"])
    for world in manifest["worlds"]:
        path = tmp_path / world["path"]
        ET.parse(path)
        assert hashlib.sha256(path.read_bytes()).hexdigest() == world["sha256"]
        text = path.read_text(encoding="utf-8")
        assert "g2_vehicle_training_rig" not in text
        assert 'type="segmentation"' not in text
    assert manifest["world_split_counts"] == {"train": 3, "val": 1, "test": 2}
    assert manifest["static_independent_camera_rig_forbidden"] is True
