from __future__ import annotations

from pathlib import Path
import sys

import pytest
import yaml


_PACKAGE_DIR = Path(__file__).resolve().parents[1]
if str(_PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_DIR))


from sanitation_learning import g4_scene
from sanitation_learning.g4_scene import (
    DISTANCE_BUCKETS,
    FRAMES_PER_SCENE,
    OCCLUSION_BUCKETS,
    SCENES_PER_WORLD,
    SIZE_BUCKETS,
    VISIBLE_FRACTION_BUCKETS,
    negative_only_rule,
)
from sanitation_learning.gazebo_g4 import write_g4_worlds


ROOT = Path(__file__).resolve().parents[2]
REGISTRY = ROOT / "sanitation_learning" / "config" / "g4_asset_registry.yaml"
XACRO = (
    ROOT
    / "sanitation_vehicle_description"
    / "urdf"
    / "sanitation_vehicle.urdf.xacro"
)


def _find_repository_root() -> Path:
    for candidate in Path(__file__).resolve().parents:
        if (candidate / "scripts" / "auto05r_g4_capture_all.sh").is_file():
            return candidate
    raise RuntimeError("could not locate repository root")


REPO = _find_repository_root()


@pytest.fixture(scope="session")
def g4_world_manifest(tmp_path_factory):
    root = tmp_path_factory.mktemp("g4_worlds")
    manifest = write_g4_worlds(REGISTRY, root / "models", XACRO, root / "worlds")
    manifest_path = root / "worlds" / "g4_world_manifest.json"
    return manifest, manifest_path


def test_g4_world_contract_12_worlds_8_2_2_distinct(g4_world_manifest):
    manifest, _ = g4_world_manifest
    assert len(manifest["worlds"]) == 12
    assert manifest["world_split_counts"] == {"train": 8, "val": 2, "test": 2}
    assert len({world["sha256"] for world in manifest["worlds"]}) == 12
    assert len({world["material_id"] for world in manifest["worlds"]}) == 12
    assert len({world["layout_family"] for world in manifest["worlds"]}) == 12
    assert len({world["geometry_family"] for world in manifest["worlds"]}) == 12
    assert len({world["lighting_family"] for world in manifest["worlds"]}) == 12
    assert len(manifest["assets"]) == 166
    assert len(manifest["negative_assets"]) == 84
    assert manifest["test_used_for_model_selection"] is False
    assert manifest["G4_dataset_gate_pass"] is False
    for world in manifest["worlds"]:
        assert world["sensor_topics"] == [
            "/camera/color/image_raw",
            "/camera/depth/image_rect_raw",
            "/camera/color/camera_info",
            "/ground_truth/semantic/image",
            "/ground_truth/instance/image",
        ]


def test_negative_only_prior_and_scene_contract(
    monkeypatch, tmp_path_factory, g4_world_manifest
):
    manifest, manifest_path = g4_world_manifest
    scenes_root = tmp_path_factory.mktemp("scenes")
    calls = []
    monkeypatch.setattr(
        "sanitation_learning.g4_scene.set_poses",
        lambda world_id, poses: calls.append((world_id, poses)),
    )
    reports = []
    for world_index, world in enumerate(manifest["worlds"]):
        for index in range(SCENES_PER_WORLD):
            seed = world_index * SCENES_PER_WORLD + index
            reports.append(
                g4_scene.randomize(
                    manifest_path,
                    world["world_id"],
                    seed,
                    index,
                    scenes_root / f"scene_{seed:04d}.json",
                )
            )
    assert len(reports) == 300
    per_world = {}
    per_split = {}
    for report in reports:
        per_world.setdefault(report["world_id"], []).append(report)
        per_split.setdefault(report["split"], []).append(report)
    for world_id, world_reports in per_world.items():
        ratio = sum(r["negative_only"] for r in world_reports) / len(world_reports)
        assert 0.25 <= ratio <= 0.35
    split_ratios = {
        split: sum(r["negative_only"] for r in split_reports) / len(split_reports)
        for split, split_reports in per_split.items()
    }
    assert max(split_ratios.values()) - min(split_ratios.values()) <= 0.10
    assert {split: len(items) for split, items in per_split.items()} == {
        "train": 200,
        "val": 50,
        "test": 50,
    }
    train_reports = per_split["train"]
    train_negative_only_frames = sum(
        FRAMES_PER_SCENE for r in train_reports if r["negative_only"]
    )
    train_paper_like_frames = sum(
        FRAMES_PER_SCENE
        for r in train_reports
        if r["paper_like_hard_negative_count"] > 0
    )
    assert train_negative_only_frames >= 500
    assert train_paper_like_frames >= 300
    for report in reports:
        assert report["schema_version"] == 2
        assert report["native_gazebo_applied"] is True
        assert report["offline_sensor_augmentation"] == {
            "requested_only": False,
            "applied": False,
            "plan": None,
        }
        assert "trajectory_id" in report
        assert "manifest_sha256" in report
        assert set(report["distance_bucket_counts"]) == {
            "0.5_2.0",
            "2.0_4.0",
            "4.0_8.0",
        }
        assert set(report["size_bucket_counts"]) == set(SIZE_BUCKETS)
        assert set(report["occlusion_bucket_counts"]) == set(OCCLUSION_BUCKETS)
        assert set(report["visible_fraction_bucket_counts"]) == set(
            VISIBLE_FRACTION_BUCKETS
        )
        for item in report["objects"]:
            assert list(item["distance_bucket_m"]) in [
                list(bucket) for bucket in DISTANCE_BUCKETS
            ]
            assert item["size_bucket"] in SIZE_BUCKETS
            assert item["occlusion_bucket"] in OCCLUSION_BUCKETS
            assert item["visible_fraction_bucket"] in VISIBLE_FRACTION_BUCKETS
        positive = [item for item in report["objects"] if item["semantic_label"] > 0]
        if report["negative_only"]:
            assert positive == []
        else:
            assert len(positive) == 5
            assert all(1.76 <= item["distance_m"] <= 3.44 for item in positive)
            assert {item["xyz_m"][1] for item in positive} == {
                -1.05,
                -0.8,
                -0.6,
                0.6,
                0.8,
            }
            assert len({round(item["distance_m"], 1) for item in positive}) == 5
            assert sorted(item["xyz_m"][1] for item in positive) == [
                -1.05, -0.8, -0.6, 0.6, 0.8
            ]
            assert report["overlap_executed"] is False
    assert len(calls) == 300
    expected_asset_count = len(manifest["assets"]) + len(
        manifest["negative_assets"]
    )
    for _world_id, poses in calls:
        assert len(poses) == expected_asset_count + 1
        assert poses[0]["name"] == "sanitation_vehicle"
        asset_names = [pose["name"] for pose in poses[1:]]
        assert len(asset_names) == len(set(asset_names)) == expected_asset_count
    for report in reports:
        assert report["pose_reset_contract"] == {
            "all_world_assets_accounted_for": True,
            "selected_asset_count": len(report["objects"]),
            "parked_asset_count": expected_asset_count - len(report["objects"]),
            "asset_pose_count": expected_asset_count,
            "duplicate_asset_pose_names": 0,
        }


def test_negative_only_rule_is_frozen():
    assert sum(negative_only_rule("train", i) for i in range(25)) == 7
    assert sum(negative_only_rule("val", i) for i in range(25)) == 8
    assert sum(negative_only_rule("test", i) for i in range(25)) == 7


def test_oprv3_coverage_profiles_are_explicit_and_fail_closed(
    monkeypatch, tmp_path, g4_world_manifest
):
    _, manifest_path = g4_world_manifest
    monkeypatch.setattr(
        "sanitation_learning.g4_scene.set_poses", lambda *_args: None
    )
    turn = g4_scene.randomize(
        manifest_path,
        "world_g4_01_asphalt_campus",
        9002,
        2,
        tmp_path / "turn.json",
        oprv3_coverage_profile="turn_entry",
    )
    assert turn["vehicle_start_yaw_rad"] == pytest.approx(1.5707963268)
    assert turn["oprv3_motion_profile"]["name"] == "turn_then_approach"
    assert turn["oprv3_coverage_requirements"] == {
        "behind_vehicle_fov_entry": True,
        "turning": True,
        "occlusion": False,
        "reflection": False,
    }

    occlusion = g4_scene.randomize(
        manifest_path,
        "world_g4_01_asphalt_campus",
        9012,
        2,
        tmp_path / "occlusion.json",
        oprv3_coverage_profile="occlusion",
    )
    paper = next(
        item for item in occlusion["objects"] if item["class_id"] == "paper_litter"
    )
    assert occlusion["overlap_executed"] is True
    assert paper["occlusion_bucket"] == "heavy"
    assert paper["occluded_by_model_name"]
    assert occlusion["occlusion_bucket_counts"]["heavy"] >= 1

    reflection = g4_scene.randomize(
        manifest_path,
        "world_g4_03_wet_courtyard",
        9022,
        2,
        tmp_path / "reflection.json",
        oprv3_coverage_profile="reflection",
    )
    assert reflection["oprv3_coverage_requirements"]["reflection"] is True
    with pytest.raises(ValueError, match="wet world"):
        g4_scene.randomize(
            manifest_path,
            "world_g4_01_asphalt_campus",
            9032,
            2,
            tmp_path / "bad-reflection.json",
            oprv3_coverage_profile="reflection",
        )


def test_factorized_diagnostic_uses_explicit_asset_sources(
    monkeypatch, tmp_path, g4_world_manifest
):
    _, manifest_path = g4_world_manifest
    monkeypatch.setattr(
        "sanitation_learning.g4_scene.set_poses", lambda *_args: None
    )
    report = g4_scene.randomize(
        manifest_path,
        "world_g4_01_asphalt_campus",
        1000,
        2,
        tmp_path / "d1.json",
        diagnostic_role="D1",
        asset_source_split="val",
        negative_source_split="train",
    )
    assert report["split"] == "D1"
    assert report["source_world_split"] == "train"
    assert report["factorized_diagnostic"] == {
        "enabled": True,
        "role": "D1",
        "asset_source_split": "val",
        "negative_source_split": "train",
        "force_negative_only": False,
        "single_factor_capture": True,
    }
    positive = [item for item in report["objects"] if item["semantic_label"]]
    assert positive
    assert all(item["split_eligibility"] == ["val"] for item in positive)

    negative = g4_scene.randomize(
        manifest_path,
        "world_g4_01_asphalt_campus",
        1400,
        1,
        tmp_path / "d5.json",
        diagnostic_role="D5",
        asset_source_split="train",
        negative_source_split="val",
        force_negative_only=True,
    )
    assert negative["negative_only"] is True
    assert all(not item["semantic_label"] for item in negative["objects"])
    assert all(item["split_eligibility"] == ["val"] for item in negative["objects"])


def test_g4_contract_frozen_contract_matches_generator():
    contract = yaml.safe_load(
        (
            ROOT
            / "sanitation_learning"
            / "config"
            / "auto05r_g4_contract.yaml"
        ).read_text(encoding="utf-8")
    )
    formal = contract["formal_scale"]
    assert formal == {
        "worlds": 12,
        "world_split_counts": {"train": 8, "val": 2, "test": 2},
        "scenes": 300,
        "scenes_per_world": 25,
        "frames": 3000,
        "frames_per_scene": 10,
    }
    assert contract["assets"]["target_variants_total"] == 166
    assert contract["assets"]["hard_negative_families_total"] == 84
    assert contract["assets"]["hard_negative_family_split_counts"] == {
        "train": 52,
        "val": 16,
        "test": 16,
    }
    assert contract["negative_prior"]["negative_only_ratio_range"] == [0.25, 0.35]
    assert contract["negative_prior"]["cross_split_ratio_delta_max"] == 0.10
    assert contract["data_contract"]["test_used_for_model_selection"] is False
    assert contract["data_contract"]["G4_dataset_gate_pass"] is False
    assert contract["data_contract"]["full_capture_executed"] is False


def test_g4_capture_script_uses_g4_modules_and_resume_skip():
    script = (REPO / "scripts" / "auto05r_g4_capture_all.sh").read_text(
        encoding="utf-8"
    )
    assert "auto05r_generate_g4_worlds" in script
    assert "auto05r_randomize_g4_scene" in script
    assert "g4_world_manifest.json" in script
    assert "resume-skip" in script
    assert "GZ_SIM_RESOURCE_PATH" in script
    assert "AUTO05R_MAX_WORLDS" in script
    assert "AUTO05R_START_WORLD_INDEX" in script
    assert "AUTO05R_ONLY_SCENES" in script
    assert "AUTO05R_FORCE_SCENES" in script
    assert "AUTO05R_DIAGNOSTIC_ROLE" in script
    assert "AUTO05R_SCENE_SEED_OFFSET" in script
    assert "START_WORLD_INDEX + local_world_index" in script
    assert "/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge" in script
    assert "auto05r_v5_retracted_primary_perception_v1" in script
    assert '--camera-xyz "${CAMERA_X}" "${CAMERA_Y}" "${CAMERA_Z}"' in script
    assert 'camera_pitch_rad:="${CAMERA_PITCH_RAD}"' in script
    wrapper = (REPO / "scripts" / "run_auto05r_g4_capture_docker.ps1").read_text(
        encoding="utf-8"
    )
    assert "ROS_DOMAIN_ID=$RosDomainId" in wrapper
    assert "GZ_PARTITION=$GzPartition" in wrapper
    assert "IGN_PARTITION=$GzPartition" in wrapper
    assert "100 + $StartWorldIndex" in wrapper
