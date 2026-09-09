import json
from pathlib import Path

import numpy as np
import prepare_trcrv10_g10_coco as prepare
import pytest


def route_profile() -> dict:
    return {
        "route_id": prepare.EXPECTED_ROUTE_ID,
        "name": "g10_straight_then_reverse_v1",
        "control_mode": "latched_world_x_switch",
        "switch_world_x_m": -1.95,
        "straight_linear_x_mps": 0.05,
        "orbit_linear_x_mps": -0.05,
        "orbit_angular_z_rad_s": 0.0,
        "post_switch_phase_name": "straight_reverse_after_candidate",
        "route_contract": (
            "drive straight using odometry only until world x reaches -1.95 m, "
            "then latch a straight reverse command for all remaining frames"
        ),
        "route_config_sha256": prepare.EXPECTED_ROUTE_CONFIG_SHA256,
    }


def test_annotations_preserve_three_class_gt_for_offline_use() -> None:
    mask = np.zeros((20, 30), dtype=np.uint8)
    mask[2:8, 3:12] = 1
    mask[10:19, 15:25] = 3
    rows, next_id = prepare.annotations(mask, image_id=4, next_id=9)
    assert [row["category_id"] for row in rows] == [1, 3]
    assert rows[0]["bbox"] == [3, 2, 9, 6]
    assert rows[0]["bbox_short_side_px"] == 6
    assert next_id == 11


def test_categories_are_fixed() -> None:
    assert [row["name"] for row in prepare.CATEGORIES] == [
        "plastic_bottle", "metal_can", "paper_litter"
    ]


def test_coco_index_binds_camera_intrinsics_path() -> None:
    source = Path(prepare.__file__).read_text(encoding="utf-8")
    assert '"camera_file_name"' in source
    assert '"semantic_file_name"' in source
    assert '"instance_file_name"' in source
    assert '"capture_report"' in source


def test_declared_holdout_requires_explicit_val_capture(tmp_path) -> None:
    scenes = tmp_path / "capture" / "g4_screening_native" / "scenes"
    scene = scenes / "scene_0001"
    scene.mkdir(parents=True)
    (scene / "scene_manifest.json").write_text(
        '{"split":"train","world_id":"w","scene_seed":1,'
        '"negative_only":false,"trcrv10_g10_approach_sequence":'
        '{"gt_runtime_forbidden":true}}',
        encoding="utf-8",
    )
    (scene / "capture_report.json").write_text(
        '{"capture_pass":true,"captured_frames":1}', encoding="utf-8"
    )

    with pytest.raises(RuntimeError, match="explicit val capture"):
        prepare.build(scenes, declared_source_split="G10_HOLDOUT")


def test_only_fixed_g10_train_or_holdout_can_be_declared(tmp_path) -> None:
    with pytest.raises(ValueError, match="must be G10_TRAIN or G10_HOLDOUT"):
        prepare.build(tmp_path, declared_source_split="G10_DEV_VAL_SEALED")


def test_explicit_holdout_declaration_locks_domain_and_relative_paths(
    tmp_path, monkeypatch
) -> None:
    domain = tmp_path / "domain.json"
    domain.write_text("{}", encoding="utf-8")
    domain_sha256 = prepare.sha256(domain)
    monkeypatch.setattr(prepare, "EXPECTED_DOMAIN_MANIFEST_SHA256", domain_sha256)
    capture = tmp_path / "capture"
    scenes = capture / "g4_screening_native" / "scenes"
    for index, world_id in enumerate(sorted(prepare.HOLDOUT_WORLD_IDS), start=1):
        scene = scenes / f"scene_{index:04d}"
        for name in ("rgb", "depth", "camera", "semantic", "instance"):
            (scene / name).mkdir(parents=True, exist_ok=True)
        profile = route_profile()
        (scene / "scene_manifest.json").write_text(
            json.dumps(
                {
                    "split": "val",
                    "world_id": world_id,
                    "world_sha256": str(index) * 64,
                    "scene_seed": index,
                    "trajectory_id": (
                        f"{world_id}_{prepare.EXPECTED_ROUTE_ID}_trajectory_{index:04d}"
                    ),
                    "negative_only": False,
                    "trcrv10_g10_approach_sequence": {
                        "gt_runtime_forbidden": True,
                        "route_id": prepare.EXPECTED_ROUTE_ID,
                        "route_config_sha256": prepare.EXPECTED_ROUTE_CONFIG_SHA256,
                        "source_domain_manifest_sha256": domain_sha256,
                    },
                    "oprv3_motion_profile": profile,
                }
            ),
            encoding="utf-8",
        )
        (scene / "capture_report.json").write_text(json.dumps({
            "capture_pass": True,
            "captured_frames": 1,
            "oprv3_motion_profile": profile,
        }), encoding="utf-8")
        mask = np.zeros((8, 10), dtype=np.uint8)
        mask[2:5, 3:7] = index
        np.save(scene / "semantic" / "frame_00.npy", mask)
    payload = prepare.build(
        scenes,
        declared_source_split="G10_HOLDOUT",
        domain_manifest=domain,
    )

    assert payload["info"]["g10_domain_manifest_sha256"] == prepare.sha256(domain)
    assert payload["info"]["holdout_world_ids"] == sorted(prepare.HOLDOUT_WORLD_IDS)
    assert payload["info"]["capture_source_splits"] == ["val"]
    assert payload["info"]["path_contract"] == "capture_root_relative_posix"
    assert payload["info"]["route_id"] == prepare.EXPECTED_ROUTE_ID
    assert payload["info"]["route_config_sha256"] == prepare.EXPECTED_ROUTE_CONFIG_SHA256
    assert all(row["source_split"] == "G10_HOLDOUT" for row in payload["images"])
    assert all(not row["file_name"].startswith(("/", "\\")) for row in payload["images"])
    assert all("\\" not in row["file_name"] for row in payload["images"])


def test_explicit_train_declaration_locks_six_worlds_and_domain(
    tmp_path, monkeypatch
) -> None:
    domain = tmp_path / "domain.json"
    domain.write_text("{}", encoding="utf-8")
    domain_sha256 = prepare.sha256(domain)
    monkeypatch.setattr(prepare, "EXPECTED_DOMAIN_MANIFEST_SHA256", domain_sha256)
    scenes = tmp_path / "capture" / "g4_screening_native" / "scenes"
    for index, world_id in enumerate(sorted(prepare.TRAIN_WORLD_IDS), start=1):
        scene = scenes / f"scene_{index:04d}"
        scene.mkdir(parents=True)
        profile = route_profile()
        (scene / "scene_manifest.json").write_text(json.dumps({
            "split": "train",
            "world_id": world_id,
            "world_sha256": str(index) * 64,
            "scene_seed": index,
            "trajectory_id": (
                f"{world_id}_{prepare.EXPECTED_ROUTE_ID}_trajectory_{index:04d}"
            ),
            "negative_only": False,
            "trcrv10_g10_approach_sequence": {
                "gt_runtime_forbidden": True,
                "route_id": prepare.EXPECTED_ROUTE_ID,
                "route_config_sha256": prepare.EXPECTED_ROUTE_CONFIG_SHA256,
                "source_domain_manifest_sha256": domain_sha256,
            },
            "oprv3_motion_profile": profile,
        }), encoding="utf-8")
        (scene / "capture_report.json").write_text(json.dumps({
            "capture_pass": True,
            "captured_frames": 0,
            "oprv3_motion_profile": profile,
        }), encoding="utf-8")

    payload = prepare.build(
        scenes,
        declared_source_split="G10_TRAIN",
        domain_manifest=domain,
    )

    assert payload["info"]["declared_source_split"] == "G10_TRAIN"
    assert payload["info"]["capture_source_splits"] == ["train"]
    assert payload["info"]["approved_world_ids"] == sorted(prepare.TRAIN_WORLD_IDS)
    assert payload["info"]["g10_domain_manifest_sha256"] == domain_sha256
