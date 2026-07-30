import hashlib
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

from sanitation_learning.g2_capture import outside_start_envelope, should_reapply_start
from sanitation_learning.g2_scene import set_poses
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


def test_capture_rejects_cross_scene_vehicle_motion_state():
    start = [-8.0, 0.0, 0.18]
    assert not outside_start_envelope((-8.0, 0.0), start)
    assert not outside_start_envelope((-7.51, 0.0), start)
    assert outside_start_envelope((-5.10, 0.15), start)
    assert should_reapply_start(0, (-5.10, 0.15), start)
    assert not should_reapply_start(1, (-5.10, 0.15), start)


def test_set_pose_vector_retries_transient_service_timeout(monkeypatch):
    responses = iter(
        [
            SimpleNamespace(returncode=1, stdout="", stderr="timed out"),
            SimpleNamespace(returncode=1, stdout="", stderr="timed out"),
            SimpleNamespace(returncode=0, stdout="data: true", stderr=""),
        ]
    )
    calls = []

    def fake_run(*args, **kwargs):
        calls.append((args, kwargs))
        return next(responses)

    monkeypatch.setattr("sanitation_learning.g2_scene.subprocess.run", fake_run)
    monkeypatch.setattr("sanitation_learning.g2_scene.time.sleep", lambda _: None)
    set_poses(
        "world",
        [{"name": "vehicle", "xyz": [-8.0, 0.0, 0.18], "yaw": 0.0}],
    )
    assert len(calls) == 3
