import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from sanitation_learning.g3_scene import randomize
from sanitation_learning.gazebo_g3 import write_g3_worlds
from sanitation_learning.g2_capture import (
    adjacent_motion_gate, adjacent_translation_gate, frame_motion_ready,
    frame_translation_ready, motion_command_for_frame, nearest_stamp_within,
    observed_speeds_from_records, wrapped_angle_delta,
)


ROOT = Path(__file__).resolve().parents[2]


def _find_repository_root() -> Path:
    for candidate in Path(__file__).resolve().parents:
        if (candidate / "scripts" / "auto05_capture_all.sh").is_file():
            return candidate
    raise RuntimeError("could not locate repository root containing auto05_capture_all.sh")


REPO = _find_repository_root()


def test_g3_world_contract_is_8_world_4_2_2_and_distinct(tmp_path):
    registry = ROOT / "sanitation_learning" / "config" / "asset_registry.yaml"
    xacro = (
        ROOT
        / "sanitation_vehicle_description"
        / "urdf"
        / "sanitation_vehicle.urdf.xacro"
    )
    manifest = write_g3_worlds(registry, xacro, tmp_path)
    assert len(manifest["worlds"]) == 8
    assert manifest["world_split_counts"] == {"train": 4, "val": 2, "test": 2}
    assert len({world["sha256"] for world in manifest["worlds"]}) == 8
    assert len({world["material_id"] for world in manifest["worlds"]}) == 8
    assert len({world["layout_family"] for world in manifest["worlds"]}) == 8
    assert len({world["lighting_family"] for world in manifest["worlds"]}) >= 4
    for world in manifest["worlds"]:
        path = tmp_path / world["path"]
        ET.parse(path)
        assert hashlib.sha256(path.read_bytes()).hexdigest() == world["sha256"]
        text = path.read_text(encoding="utf-8")
        assert 'type="segmentation"' not in text
        assert "<cast_shadows>true</cast_shadows>" in text


def test_g3_heldout_world_has_five_forced_negative_scenes(monkeypatch, tmp_path):
    registry = ROOT / "sanitation_learning" / "config" / "asset_registry.yaml"
    xacro = (
        ROOT
        / "sanitation_vehicle_description"
        / "urdf"
        / "sanitation_vehicle.urdf.xacro"
    )
    manifest = write_g3_worlds(registry, xacro, tmp_path / "worlds")
    manifest_path = tmp_path / "worlds" / "g3_world_manifest.json"
    calls = []
    monkeypatch.setattr(
        "sanitation_learning.g3_scene.set_poses",
        lambda world_id, poses: calls.append((world_id, poses)),
    )
    reports = [
        randomize(
            manifest_path,
            "world_d_mixed_curb_vegetation",
            60 + index,
            index,
            tmp_path / f"scene_{index:02d}.json",
        )
        for index in range(15)
    ]
    assert sum(report["negative_only"] for report in reports) == 5
    assert all(report["split"] == "val" for report in reports)
    assert all(
        0 <= count <= 3
        for report in reports
        for count in report["target_count_by_class"].values()
    )
    assert any(report["overlap_executed"] for report in reports)
    assert any(report["dynamic_motion_plan"] for report in reports)
    for report in reports:
        plan = report["dynamic_motion_plan"]
        if plan:
            assert all(
                abs(plan["start_xyz_m"][1] + frame * plan["delta_per_frame_m"][1])
                >= 0.65
                for frame in range(10)
            )
    assert all(
        abs(item["xyz_m"][1]) >= 0.65
        for report in reports
        for item in report["objects"]
        if item["xyz_m"][0] - (-8.0) < 4.5
    )
    assert {report["active_observation_phase"] for report in reports} >= {
        "before",
        "after",
    }
    assert len(calls) == 15
    assert manifest["dataset_domain"].startswith("G3_")


def test_g3_capture_owns_the_real_bridge_process():
    capture_script = (REPO / "scripts" / "auto05_capture_all.sh").read_text(
        encoding="utf-8"
    )
    assert "/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge" in capture_script
    assert "ros2 run ros_gz_bridge parameter_bridge" not in capture_script


def test_g4_capture_timeout_is_explicitly_configurable():
    capture_script = (REPO / "scripts" / "auto05r_g4_capture_all.sh").read_text(
        encoding="utf-8"
    )
    docker_wrapper = (
        REPO / "scripts" / "run_auto05r_g4_capture_docker.ps1"
    ).read_text(encoding="utf-8")
    assert 'CAPTURE_TIMEOUT_SECONDS="${AUTO05R_CAPTURE_TIMEOUT_SECONDS:-90}"' in capture_script
    assert '--timeout "${CAPTURE_TIMEOUT_SECONDS}"' in capture_script
    assert "[double]$CaptureTimeoutSeconds = 90.0" in docker_wrapper
    assert "AUTO05R_CAPTURE_TIMEOUT_SECONDS=$CaptureTimeoutSeconds" in docker_wrapper


def test_adjacent_translation_gate_rejects_rotation_only_frames():
    records = [
        {"vehicle_xy_m": [0.0, 0.0]},
        {"vehicle_xy_m": [0.25, 0.0]},
        {"vehicle_xy_m": [0.25, 0.0]},
    ]
    assert not adjacent_translation_gate(records, requested_frames=3)
    records[-1]["vehicle_xy_m"] = [0.50, 0.0]
    assert adjacent_translation_gate(records, requested_frames=3)


def test_adjacent_translation_gate_supports_full_rate_oprv3_capture():
    records = [
        {"vehicle_xy_m": [0.00, 0.0]},
        {"vehicle_xy_m": [0.04, 0.0]},
        {"vehicle_xy_m": [0.08, 0.0]},
    ]
    assert adjacent_translation_gate(
        records, requested_frames=3, minimum_translation_m=0.01
    )
    assert not adjacent_translation_gate(
        records, requested_frames=3, minimum_translation_m=0.05
    )
    assert frame_translation_ready((0.0, 0.0), (0.04, 0.0), 0.01)
    assert not frame_translation_ready((0.0, 0.0), (0.04, 0.0), 0.05)
    assert nearest_stamp_within([900, 1_010, 1_100], 1_000, 50) == 1_010
    assert nearest_stamp_within([900, 1_100], 1_000, 50) is None
    records_with_odom = [
        {"odom_timestamp_ns": 0, "timestamp_ns": 0, "vehicle_xy_m": [0.0, 0.0]},
        {"odom_timestamp_ns": 100_000_000, "timestamp_ns": 50_000_000, "vehicle_xy_m": [0.065, 0.0]},
        {"odom_timestamp_ns": 200_000_000, "timestamp_ns": 250_000_000, "vehicle_xy_m": [0.130, 0.0]},
    ]
    assert observed_speeds_from_records(records_with_odom) == pytest.approx([0.65, 0.65])


def test_oprv3_turn_profile_accepts_rotation_without_weakening_straight_gate():
    records = [
        {"vehicle_xy_m": [0.0, 0.0], "vehicle_yaw_rad": 1.57},
        {"vehicle_xy_m": [0.0, 0.0], "vehicle_yaw_rad": 1.47},
        {"vehicle_xy_m": [0.05, 0.0], "vehicle_yaw_rad": 1.47},
    ]
    assert not adjacent_translation_gate(records, requested_frames=3)
    assert adjacent_motion_gate(
        records,
        requested_frames=3,
        minimum_translation_m=0.04,
        minimum_rotation_rad=0.08,
    )
    assert frame_motion_ready(
        (0.0, 0.0, 1.57),
        (0.0, 0.0, 1.47),
        minimum_translation_m=0.04,
        minimum_rotation_rad=0.08,
    )
    assert wrapped_angle_delta(3.13, -3.13) == pytest.approx(0.023185, abs=1e-5)


def test_motion_profile_is_frame_counted_and_fails_closed_when_exhausted():
    profile = {
        "name": "turn_then_approach",
        "phases": [
            {"name": "turn", "frame_count": 2, "angular_z_rad_s": -0.5},
            {"name": "approach", "frame_count": 3, "linear_x_mps": 0.65},
        ],
    }
    assert motion_command_for_frame(profile, 0, 0.35) == (0.0, -0.5, "turn")
    assert motion_command_for_frame(profile, 2, 0.35) == (0.65, 0.0, "approach")
    with pytest.raises(ValueError, match="does not cover"):
        motion_command_for_frame(profile, 5, 0.35)
