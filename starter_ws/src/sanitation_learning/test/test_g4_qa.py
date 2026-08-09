from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys

import cv2
import numpy as np
import pytest


_PACKAGE_DIR = Path(__file__).resolve().parents[1]
if str(_PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_DIR))


from sanitation_learning import g4_scene
from sanitation_learning.g4_assets import write_g4_assets
from sanitation_learning.g4_qa import (
    finalize_g4_dataset,
    perceptual_near_duplicate,
    phash,
)
from sanitation_learning.gazebo_g4 import write_g4_worlds


g4_scene.set_poses = lambda world_id, poses: None

ROOT = Path(__file__).resolve().parents[2]
REGISTRY = ROOT / "sanitation_learning" / "config" / "g4_asset_registry.yaml"
XACRO = (
    ROOT
    / "sanitation_vehicle_description"
    / "urdf"
    / "sanitation_vehicle.urdf.xacro"
)
CONTRACT = ROOT / "sanitation_learning" / "config" / "auto05r_g4_contract.yaml"
SCENES_PER_WORLD = 25


def _find_repository_root() -> Path:
    for candidate in Path(__file__).resolve().parents:
        if (candidate / "scripts" / "auto05r_g4_finalize_dataset.py").is_file():
            return candidate
    raise RuntimeError("could not locate repository root")


REPO = _find_repository_root()


def _gradient_frame(split: str, seed: int) -> np.ndarray:
    base = {
        "train": (70, 122, 84),
        "val": (70, 96, 150),
        "test": (150, 76, 66),
    }[split]
    yy, xx = np.mgrid[0:480, 0:640]
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    for channel in range(3):
        phase = (seed + channel) % 3
        rgb[:, :, channel] = np.clip(
            base[channel]
            + ((xx * (1 + phase) + yy * phase) % 64)
            - 32,
            0,
            255,
        ).astype(np.uint8)
    return rgb


def _write_frame(
    scene_dir: Path,
    index: int,
    split: str,
    seed: int,
    objects: list[dict] | None = None,
) -> dict:
    stem = f"frame_{index:02d}"
    for name in ("rgb", "depth", "semantic", "instance", "camera", "tf"):
        (scene_dir / name).mkdir(parents=True, exist_ok=True)
    rgb = _gradient_frame(split, seed)
    cv2.imwrite(
        str(scene_dir / "rgb" / f"{stem}.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    )
    semantic = np.zeros((480, 640), dtype=np.uint8)
    instance = np.zeros((480, 640), dtype=np.uint16)
    targets = [item for item in (objects or []) if int(item["semantic_label"]) > 0]
    for object_index, item in enumerate(targets, start=1):
        left = 30 + (object_index % 10) * 45
        top = 180 + (object_index // 10) * 30
        semantic[top : top + 12, left : left + 12] = int(item["semantic_label"])
        instance[top : top + 12, left : left + 12] = object_index
    cv2.imwrite(str(scene_dir / "semantic" / f"{stem}.png"), semantic)
    cv2.imwrite(str(scene_dir / "instance" / f"{stem}.png"), instance)
    (scene_dir / "depth" / f"{stem}.npy").write_bytes(b"\x00")
    (scene_dir / "camera" / f"{stem}.json").write_text(
        json.dumps(
            {
                "width": 640,
                "height": 480,
                "k": [320.0] * 9,
                "p": [320.0] * 12,
                "frame_id": "camera_depth_link",
            }
        )
        + "\n",
        encoding="utf-8",
    )
    x = -8.0 + 0.3 * index
    (scene_dir / "tf" / f"{stem}.json").write_text(
        json.dumps(
            {
                "world_to_base_xy": [x, 0.0],
                "base_to_camera_xyz_m": [0.53, 0.0, 0.22],
            }
        )
        + "\n",
        encoding="utf-8",
    )
    return {
        "frame_index": index,
        "timestamp_ns": 1000 + index,
        "vehicle_xy_m": [x, 0.0],
        "exact_four_sensor_timestamp": True,
        "paths": {
            "rgb": f"rgb/{stem}.png",
            "depth": f"depth/{stem}.npy",
            "semantic": f"semantic/{stem}.png",
            "instance": f"instance/{stem}.png",
            "camera": f"camera/{stem}.json",
            "tf": f"tf/{stem}.json",
        },
    }


def _build_scene(
    data_root: Path,
    manifest_path: Path,
    world_id: str,
    world_index: int,
    scene_index: int,
) -> None:
    seed = world_index * SCENES_PER_WORLD + scene_index
    scene_dir = data_root / "scenes" / f"scene_{seed:04d}"
    scene_dir.mkdir(parents=True, exist_ok=True)
    scene = g4_scene.randomize(
        manifest_path,
        world_id,
        seed,
        scene_index,
        scene_dir / "scene_manifest.json",
    )
    records = [
        _write_frame(scene_dir, index, scene["split"], seed, scene["objects"])
        for index in range(10)
    ]
    (scene_dir / "capture_report.json").write_text(
        json.dumps({"schema_version": 1, "capture_pass": True, "records": records})
        + "\n",
        encoding="utf-8",
    )


@pytest.fixture(scope="session")
def good_dataset(tmp_path_factory, g4_assets_dir):
    """75 scenes (25 per world) across one train/val/test world each."""
    root = tmp_path_factory.mktemp("g4_good")
    write_g4_worlds(REGISTRY, g4_assets_dir, XACRO, root / "worlds")
    manifest_path = root / "worlds" / "g4_world_manifest.json"
    for world_id, world_index in (
        ("world_g4_01_asphalt_campus", 0),
        ("world_g4_09_light_paver_pedestrian", 8),
        ("world_g4_11_brick_market_street", 10),
    ):
        for scene_index in range(SCENES_PER_WORLD):
            _build_scene(root, manifest_path, world_id, world_index, scene_index)
    return root


def test_good_smoke_reports_expected_actual_and_quality_pass(
    good_dataset, tmp_path
):
    output = tmp_path / "qa"
    report = finalize_g4_dataset(
        good_dataset, output, contract_path=str(CONTRACT)
    )
    assert report["schema_version"] == 2
    assert report["formal_scale"]["expected"]["worlds"] == 12
    assert report["formal_scale"]["expected"]["scenes"] == 300
    assert report["formal_scale"]["expected"]["frames"] == 3000
    assert report["formal_scale"]["actual"]["worlds"] == 3
    assert report["formal_scale"]["actual"]["scenes"] == 75
    assert report["formal_scale"]["actual"]["frames"] == 750
    assert report["test_used_for_model_selection"] is False
    assert report["G4_dataset_gate_pass"] is False
    assert report["quality_gates_pass"] is True
    for name in (
        "negative_only_ratio_in_25_to_35_percent",
        "negative_only_cross_split_delta_at_most_10pp",
        "annotation_completeness_100_percent",
        "four_sensor_sync_100_percent",
        "camera_info_valid_100_percent",
        "tf_valid_100_percent",
        "semantic_instance_error_zero",
        "scene_pose_reset_contract_100_percent",
        "manifest_pixel_target_consistency_100_percent",
        "declared_target_sequence_visibility_100_percent",
        "asset_split_leakage_zero",
        "world_split_leakage_zero",
        "trajectory_split_leakage_zero",
        "exact_duplicate_zero",
        "cross_split_phash_duplicate_zero",
        "distance_bucket_coverage_all",
        "size_bucket_coverage_all",
        "test_used_for_model_selection_false",
    ):
        assert report["gates"][name] is True
    for name in (
        "worlds_12_and_8_2_2",
        "scenes_300_and_frames_3000",
        "scenes_per_world_25",
        "train_negative_only_frames_at_least_500",
        "train_paper_like_hard_negative_frames_at_least_300",
    ):
        assert report["gates"][name] is False
    for filename in (
        "g4_dataset_qa.json",
        "split_manifest.json",
        "leakage_report.json",
        "g4_frame_manifest.jsonl",
        "g4_instance_records.jsonl",
    ):
        assert (output / filename).is_file()
    assert report["errors"] == []
    assert report["negative_only_ratio_by_split"] == {
        "train": 0.28,
        "val": 0.32,
        "test": 0.28,
    }


def _tiny_scene(root: Path, manifest_path: Path, world_id: str, seed: int, split: str) -> None:
    scene_dir = root / "scenes" / f"scene_{seed:04d}"
    scene_dir.mkdir(parents=True, exist_ok=True)
    scene = g4_scene.randomize(
        manifest_path, world_id, seed, seed % 25, scene_dir / "scene_manifest.json"
    )
    records = [
        _write_frame(scene_dir, index, split, seed, scene["objects"])
        for index in range(10)
    ]
    (scene_dir / "capture_report.json").write_text(
        json.dumps({"schema_version": 1, "capture_pass": True, "records": records})
        + "\n",
        encoding="utf-8",
    )


@pytest.fixture(scope="session")
def g4_assets_dir(tmp_path_factory):
    assets = tmp_path_factory.mktemp("g4_assets")
    write_g4_assets(REGISTRY, assets)
    return assets


@pytest.fixture()
def tiny_worlds(tmp_path, g4_assets_dir):
    root = tmp_path / "data"
    write_g4_worlds(REGISTRY, g4_assets_dir, XACRO, root / "worlds")
    return root, root / "worlds" / "g4_world_manifest.json"


def test_leakage_and_sync_gates_fail_on_injected_errors(
    tiny_worlds, tmp_path
):
    root, manifest_path = tiny_worlds
    train_dir = root / "scenes" / "scene_0005"
    val_dir = root / "scenes" / "scene_0209"
    _tiny_scene(root, manifest_path, "world_g4_01_asphalt_campus", 5, "train")
    _tiny_scene(root, manifest_path, "world_g4_09_light_paver_pedestrian", 209, "val")
    # Inject asset leakage: a val scene object points at a train-only asset.
    train_scene = json.loads((train_dir / "scene_manifest.json").read_text(encoding="utf-8"))
    train_asset = next(
        item["asset_id"] for item in train_scene["objects"] if item["semantic_label"]
    )
    val_scene_path = val_dir / "scene_manifest.json"
    val_scene = json.loads(val_scene_path.read_text(encoding="utf-8"))
    target = next(
        item for item in val_scene["objects"] if item["semantic_label"]
    )
    target["asset_id"] = train_asset
    val_scene_path.write_text(json.dumps(val_scene) + "\n", encoding="utf-8")
    # Inject a four-sensor sync violation in one frame.
    capture_path = val_dir / "capture_report.json"
    capture = json.loads(capture_path.read_text(encoding="utf-8"))
    capture["records"][3]["exact_four_sensor_timestamp"] = False
    capture_path.write_text(json.dumps(capture) + "\n", encoding="utf-8")
    report = finalize_g4_dataset(
        root, tmp_path / "qa", contract_path=str(CONTRACT)
    )
    assert report["gates"]["asset_split_leakage_zero"] is False
    assert report["gates"]["four_sensor_sync_100_percent"] is False
    assert report["gates"]["cross_split_phash_duplicate_zero"] is True


def test_negative_ratio_gate_fails_on_injected_prior_error(
    tiny_worlds, tmp_path
):
    root, manifest_path = tiny_worlds
    scene_dir = root / "scenes" / "scene_0000"
    _tiny_scene(root, manifest_path, "world_g4_01_asphalt_campus", 0, "train")
    scene_path = scene_dir / "scene_manifest.json"
    scene = json.loads(scene_path.read_text(encoding="utf-8"))
    assert scene["negative_only"] is True
    scene["negative_only"] = False
    scene_path.write_text(json.dumps(scene) + "\n", encoding="utf-8")
    report = finalize_g4_dataset(
        root, tmp_path / "qa", contract_path=str(CONTRACT)
    )
    assert report["gates"]["negative_only_ratio_in_25_to_35_percent"] is False
    assert report["quality_gates_pass"] is False


def test_manifest_pixel_target_gate_rejects_stale_positive(
    tiny_worlds, tmp_path
):
    root, manifest_path = tiny_worlds
    _tiny_scene(root, manifest_path, "world_g4_01_asphalt_campus", 0, "train")
    scene_dir = root / "scenes" / "scene_0000"
    scene = json.loads(
        (scene_dir / "scene_manifest.json").read_text(encoding="utf-8")
    )
    assert scene["negative_only"] is True
    instance_path = scene_dir / "instance" / "frame_00.png"
    semantic_path = scene_dir / "semantic" / "frame_00.png"
    instance = np.zeros((480, 640), dtype=np.uint16)
    semantic = np.zeros((480, 640), dtype=np.uint8)
    instance[200:220, 200:220] = 1
    semantic[200:220, 200:220] = 1
    cv2.imwrite(str(instance_path), instance)
    cv2.imwrite(str(semantic_path), semantic)
    report = finalize_g4_dataset(
        root, tmp_path / "qa", contract_path=str(CONTRACT)
    )
    assert report["gates"]["manifest_pixel_target_consistency_100_percent"] is False
    assert report["manifest_pixel_target_consistency_rate"] == 0.9
    assert any(
        error["reason"] == "undeclared_pixel_target_count_exceeded"
        for error in report["errors"]
    )


def test_sequence_visibility_gate_rejects_declared_target_never_seen(
    tiny_worlds, tmp_path
):
    root, manifest_path = tiny_worlds
    _tiny_scene(root, manifest_path, "world_g4_01_asphalt_campus", 1, "train")
    scene_dir = root / "scenes" / "scene_0001"
    scene = json.loads((scene_dir / "scene_manifest.json").read_text())
    label = next(
        int(item["semantic_label"])
        for item in scene["objects"]
        if int(item.get("semantic_label") or 0) > 0
    )
    for semantic_path in sorted((scene_dir / "semantic").glob("*.png")):
        instance_path = scene_dir / "instance" / semantic_path.name
        semantic = cv2.imread(str(semantic_path), cv2.IMREAD_UNCHANGED)
        instance = cv2.imread(str(instance_path), cv2.IMREAD_UNCHANGED)
        hidden = semantic == label
        semantic[hidden] = 0
        instance[hidden] = 0
        cv2.imwrite(str(semantic_path), semantic)
        cv2.imwrite(str(instance_path), instance)
    report = finalize_g4_dataset(root, tmp_path / "qa", contract_path=str(CONTRACT))
    assert report["gates"]["manifest_pixel_target_consistency_100_percent"] is True
    assert report["gates"]["declared_target_sequence_visibility_100_percent"] is False
    assert any(
        error["reason"] == "declared_target_sequence_visibility_failed"
        for error in report["errors"]
    )


def test_contract_enforces_test_used_for_model_selection_false(
    tiny_worlds, tmp_path
):
    root, manifest_path = tiny_worlds
    import yaml

    broken = tmp_path / "broken_contract.yaml"
    payload = yaml.safe_load(CONTRACT.read_text(encoding="utf-8"))
    payload["data_contract"]["test_used_for_model_selection"] = True
    broken.write_text(
        yaml.safe_dump(payload, sort_keys=False), encoding="utf-8"
    )
    with pytest.raises(ValueError, match="test_used_for_model_selection"):
        finalize_g4_dataset(root, tmp_path / "qa", contract_path=str(broken))


def test_phash_collision_requires_independent_pixel_confirmation(tmp_path):
    dark = tmp_path / "dark.png"
    light = tmp_path / "light.png"
    near = tmp_path / "near.png"
    cv2.imwrite(str(dark), np.full((480, 640, 3), 40, dtype=np.uint8))
    cv2.imwrite(str(light), np.full((480, 640, 3), 100, dtype=np.uint8))
    cv2.imwrite(str(near), np.full((480, 640, 3), 42, dtype=np.uint8))
    assert phash(dark) == phash(light) == phash(near)
    assert perceptual_near_duplicate(dark, light) is False
    assert perceptual_near_duplicate(dark, near) is True


def test_finalize_cli_strict_and_non_strict(good_dataset, tmp_path):
    script = REPO / "scripts" / "auto05r_g4_finalize_dataset.py"
    non_strict = subprocess.run(
        [
            sys.executable,
            str(script),
            "--data-root",
            str(good_dataset),
            "--output-dir",
            str(tmp_path / "qa_non_strict"),
            "--contract",
            str(CONTRACT),
        ],
        capture_output=True,
        text=True,
    )
    assert non_strict.returncode == 0
    strict = subprocess.run(
        [
            sys.executable,
            str(script),
            "--data-root",
            str(good_dataset),
            "--output-dir",
            str(tmp_path / "qa_strict"),
            "--contract",
            str(CONTRACT),
            "--strict",
        ],
        capture_output=True,
        text=True,
    )
    assert strict.returncode == 2
    payload = json.loads(strict.stdout)
    assert payload["G4_dataset_gate_pass"] is False
