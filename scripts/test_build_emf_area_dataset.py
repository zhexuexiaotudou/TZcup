import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import pytest
from build_emf_area_dataset import (
    DatasetContractError,
    build_manifest,
    main,
    write_manifest,
)


def _make_scene(
    root: Path,
    *,
    split: str,
    world_id: str,
    scene_id: str,
    frame_index: int,
    semantic: np.ndarray,
    source_split: str | None = None,
    negative_only: bool = False,
) -> Path:
    scene = root / "scenes" / scene_id
    scene.mkdir(parents=True)
    (scene / "scene_manifest.json").write_text(
        json.dumps(
            {
                "split": source_split or split.lower(),
                "world_id": world_id,
                "scene_id": scene_id,
                "scene_seed": 123,
                "negative_only": negative_only,
            },
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    rgb = scene / f"rgb_{frame_index:06d}.png"
    depth = scene / f"depth_{frame_index:06d}.npy"
    semantic_path = scene / f"semantic_{frame_index:06d}.npy"
    assert cv2.imwrite(str(rgb), np.zeros((*semantic.shape, 3), dtype=np.uint8))
    np.save(depth, np.ones(semantic.shape, dtype=np.float32))
    np.save(semantic_path, semantic)
    (scene / "capture_report.json").write_text(
        json.dumps(
            {
                "records": [
                    {
                        "frame_index": frame_index,
                        "timestamp_ns": 1000 + frame_index,
                        "paths": {
                            "rgb": rgb.name,
                            "depth": depth.name,
                            "semantic": semantic_path.name,
                        },
                    }
                ]
            },
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    return scene


def _valid_roots(tmp_path: Path) -> tuple[Path, Path]:
    train = tmp_path / "training_capture"
    holdout = tmp_path / "holdout_capture"
    _make_scene(
        train,
        split="TRAIN",
        world_id="outdoor_world_a",
        scene_id="scene_train_001",
        frame_index=7,
        semantic=np.array([[0, 4], [1, 4]], dtype=np.uint8),
    )
    _make_scene(
        holdout,
        split="HOLDOUT",
        world_id="outdoor_world_b",
        scene_id="scene_holdout_001",
        frame_index=3,
        semantic=np.array([[0, 5], [5, 2]], dtype=np.uint8),
    )
    _make_scene(
        train,
        split="TRAIN",
        world_id="outdoor_world_a",
        scene_id="scene_train_negative_002",
        frame_index=11,
        semantic=np.zeros((2, 2), dtype=np.uint8),
        negative_only=True,
    )
    _make_scene(
        holdout,
        split="HOLDOUT",
        world_id="outdoor_world_b",
        scene_id="scene_holdout_negative_002",
        frame_index=13,
        semantic=np.zeros((2, 2), dtype=np.uint8),
        negative_only=True,
    )
    return train, holdout


def test_manifest_is_deterministic_sha_locked_and_preserves_identity(tmp_path: Path):
    train, holdout = _valid_roots(tmp_path)
    roots = [("HOLDOUT", holdout), ("TRAIN", train)]

    first = build_manifest(roots)
    second = build_manifest(reversed(roots))

    assert first == second
    assert first["a4_area_dataset_ready"] is True
    assert first["failure_reasons"] == []
    assert first["semantic_audit"]["observed_ids"] == [0, 1, 2, 4, 5]
    assert first["semantic_audit"]["leaf_pile_positive_frame_count"] == 1
    assert first["semantic_audit"]["puddle_positive_frame_count"] == 1
    identities = {
        (row["split"], row["world_id"], row["scene_id"], row["frame_index"])
        for row in first["frames"]
    }
    assert identities == {
        ("TRAIN", "outdoor_world_a", "scene_train_001", 7),
        ("TRAIN", "outdoor_world_a", "scene_train_negative_002", 11),
        ("HOLDOUT", "outdoor_world_b", "scene_holdout_001", 3),
        ("HOLDOUT", "outdoor_world_b", "scene_holdout_negative_002", 13),
    }
    assert {row["source_split"] for row in first["scenes"]} == {"train", "holdout"}
    for row in first["frames"]:
        assert row["mission_id"] == row["scene_id"]
        assert row["gt_source"]["type"] == "gazebo_ground_truth_semantic_image"
        assert row["gt_source"]["sha256"] == row["sha256"]["semantic"]
        for modality in ("rgb", "depth", "semantic"):
            source_root = next(
                root
                for root in first["source_roots"]
                if root["root_id"] == row["root_id"]
            )
            source = Path(source_root["path"]) / row["paths"][modality]
            assert (
                row["sha256"][modality]
                == hashlib.sha256(source.read_bytes()).hexdigest()
            )

    assert first["screening_dataset_contract"]["mission_field"] == (
        "frames[].mission_id"
    )
    contract = first["screening_dataset_contract"]
    assert contract["negative_only_scene_counts_by_split"] == {
        "TRAIN": 1,
        "HOLDOUT": 1,
    }
    assert contract["negative_only_frame_counts_by_split"] == {
        "TRAIN": 1,
        "HOLDOUT": 1,
    }
    assert contract["negative_only_world_ids_by_split"] == {
        "TRAIN": ["outdoor_world_a"],
        "HOLDOUT": ["outdoor_world_b"],
    }
    assert sum(frame["negative_only"] is True for frame in first["frames"]) == 2

    output = tmp_path / "area_manifest.json"
    write_manifest(first, output)
    first_bytes = output.read_bytes()
    write_manifest(second, output)
    assert output.read_bytes() == first_bytes


def test_missing_target_id_writes_fail_closed_manifest_and_returns_two(tmp_path: Path):
    train, holdout = _valid_roots(tmp_path)
    semantic_path = next(holdout.rglob("semantic_*.npy"))
    np.save(semantic_path, np.array([[0, 2], [2, 2]], dtype=np.uint8))
    output = tmp_path / "missing_target_manifest.json"

    result = main(
        [
            "--split-root",
            f"TRAIN={train}",
            "--split-root",
            f"HOLDOUT={holdout}",
            "--output",
            str(output),
        ]
    )

    assert result == 2
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["a4_area_dataset_ready"] is False
    assert payload["semantic_audit"]["puddle_positive_frame_count"] == 0
    assert payload["failure_reasons"] == ["missing_positive_semantic_id:5"]


@pytest.mark.parametrize(
    "marker", ["G5", "G5_V2", "G5V2", "VAL_NEW", "DEV_VAL", "SEALED"]
)
def test_forbidden_source_markers_are_rejected(tmp_path: Path, marker: str):
    train, holdout = _valid_roots(tmp_path)
    forbidden = tmp_path / marker / "capture"
    forbidden.parent.mkdir()
    train.rename(forbidden)

    with pytest.raises(DatasetContractError, match="forbidden marker"):
        build_manifest([("TRAIN", forbidden), ("HOLDOUT", holdout)])


def test_missing_pair_and_unknown_semantic_id_fail_closed(tmp_path: Path):
    train, holdout = _valid_roots(tmp_path)
    next(train.rglob("depth_*.npy")).unlink()
    with pytest.raises(DatasetContractError, match="missing paired file"):
        build_manifest([("TRAIN", train), ("HOLDOUT", holdout)])

    train, holdout = _valid_roots(tmp_path / "unknown_id_case")
    np.save(next(train.rglob("semantic_*.npy")), np.array([[0, 6]], dtype=np.uint8))
    with pytest.raises(DatasetContractError, match=r"unknown semantic IDs \[6\]"):
        build_manifest([("TRAIN", train), ("HOLDOUT", holdout)])


def test_explicit_holdout_accepts_legacy_val_but_rejects_incompatible_source_split(
    tmp_path: Path,
):
    train = tmp_path / "training_capture"
    holdout = tmp_path / "holdout_capture"
    semantic = np.array([[0, 4, 5]], dtype=np.uint8)
    _make_scene(
        train,
        split="TRAIN",
        world_id="world_train",
        scene_id="scene_train",
        frame_index=0,
        semantic=semantic,
    )
    holdout_scene = _make_scene(
        holdout,
        split="HOLDOUT",
        source_split="val",
        world_id="world_holdout",
        scene_id="scene_holdout",
        frame_index=0,
        semantic=semantic,
    )
    report = build_manifest([("TRAIN", train), ("HOLDOUT", holdout)])
    assert {scene["source_split"] for scene in report["scenes"]} == {"train", "val"}

    payload_path = holdout_scene / "scene_manifest.json"
    payload = json.loads(payload_path.read_text(encoding="utf-8"))
    payload["split"] = "train"
    payload_path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(
        DatasetContractError, match="incompatible with explicit root split"
    ):
        build_manifest([("TRAIN", train), ("HOLDOUT", holdout)])


def test_incomplete_scene_is_sha_locked_excluded_evidence(tmp_path: Path):
    train, holdout = _valid_roots(tmp_path)
    incomplete = holdout / "scenes" / "scene_incomplete"
    incomplete.mkdir()
    scene_manifest = incomplete / "scene_manifest.json"
    scene_manifest.write_text(
        json.dumps({"split": "val", "world_id": "world_incomplete"}),
        encoding="utf-8",
    )

    report = build_manifest([("TRAIN", train), ("HOLDOUT", holdout)])

    assert report["a4_area_dataset_ready"] is False
    assert report["excluded_scene_count"] == 1
    assert report["excluded_scenes"] == [
        {
            "root_id": "HOLDOUT_000",
            "split": "HOLDOUT",
            "scene_manifest_path": "scenes/scene_incomplete/scene_manifest.json",
            "scene_manifest_sha256": hashlib.sha256(
                scene_manifest.read_bytes()
            ).hexdigest(),
            "reason": "missing_capture_report",
        }
    ]
    assert "excluded_incomplete_scene_count:1" in report["failure_reasons"]


def test_capture_root_is_part_of_frame_identity(tmp_path: Path):
    train_a = tmp_path / "train_a"
    train_b = tmp_path / "train_b"
    holdout = tmp_path / "holdout"
    semantic = np.array([[0, 4, 5]], dtype=np.uint8)
    for root in (train_a, train_b):
        _make_scene(
            root,
            split="TRAIN",
            world_id="reused_world",
            scene_id="reused_scene",
            frame_index=0,
            semantic=semantic,
        )
    _make_scene(
        holdout,
        split="HOLDOUT",
        world_id="holdout_world",
        scene_id="holdout_scene",
        frame_index=0,
        semantic=semantic,
    )

    report = build_manifest(
        [("TRAIN", train_a), ("TRAIN", train_b), ("HOLDOUT", holdout)]
    )

    reused = [
        frame for frame in report["frames"] if frame["world_id"] == "reused_world"
    ]
    assert len(reused) == 2
    assert len({frame["root_id"] for frame in reused}) == 2


def test_corrupt_rgb_and_misaligned_depth_fail_closed(tmp_path: Path):
    train, holdout = _valid_roots(tmp_path)
    next(train.rglob("rgb_*.png")).write_bytes(b"not-an-image")
    with pytest.raises(DatasetContractError, match="cannot decode RGB"):
        build_manifest([("TRAIN", train), ("HOLDOUT", holdout)])

    train, holdout = _valid_roots(tmp_path / "depth_mismatch")
    np.save(next(train.rglob("depth_*.npy")), np.ones((1, 1), dtype=np.float32))
    with pytest.raises(DatasetContractError, match="dimensions differ"):
        build_manifest([("TRAIN", train), ("HOLDOUT", holdout)])


def test_depth_positive_infinity_is_audited_but_nan_fails_closed(tmp_path: Path):
    train, holdout = _valid_roots(tmp_path)
    depth_path = next(train.rglob("depth_*.npy"))
    depth = np.ones((2, 2), dtype=np.float32)
    depth[0, 0] = np.inf
    np.save(depth_path, depth)
    report = build_manifest([("TRAIN", train), ("HOLDOUT", holdout)])
    relative_depth = depth_path.resolve().relative_to(train.resolve()).as_posix()
    train_frame = next(
        frame
        for frame in report["frames"]
        if frame["split"] == "TRAIN"
        and frame["paths"]["depth"] == relative_depth
    )
    assert train_frame["modality_contract"]["depth_positive_inf_pixel_count"] == 1
    assert train_frame["modality_contract"]["depth_finite_pixel_fraction"] == 0.75

    depth[:] = np.nan
    np.save(depth_path, depth)
    with pytest.raises(
        DatasetContractError, match="depth tensor contains invalid values"
    ):
        build_manifest([("TRAIN", train), ("HOLDOUT", holdout)])


def test_negative_only_declaration_rejects_positive_objects_and_gt(tmp_path: Path):
    train, holdout = _valid_roots(tmp_path)
    scene_manifest_path = next(
        train.rglob("scene_train_negative_002/scene_manifest.json")
    )
    scene_manifest = json.loads(scene_manifest_path.read_text(encoding="utf-8"))
    scene_manifest["objects"] = [{"semantic_label": 4}]
    scene_manifest_path.write_text(json.dumps(scene_manifest), encoding="utf-8")
    with pytest.raises(DatasetContractError, match="declares target objects"):
        build_manifest([("TRAIN", train), ("HOLDOUT", holdout)])

    train, holdout = _valid_roots(tmp_path / "positive_gt")
    semantic_path = next(
        train.rglob("scene_train_negative_002/semantic_*.npy")
    )
    np.save(semantic_path, np.array([[0, 4], [0, 0]], dtype=np.uint8))
    with pytest.raises(DatasetContractError, match="contains positive semantic GT"):
        build_manifest([("TRAIN", train), ("HOLDOUT", holdout)])


def test_ready_requires_declared_negative_only_scene_in_each_split(tmp_path: Path):
    train, holdout = _valid_roots(tmp_path)
    holdout_negative = next(
        holdout.rglob("scene_holdout_negative_002/scene_manifest.json")
    )
    payload = json.loads(holdout_negative.read_text(encoding="utf-8"))
    payload.pop("negative_only")
    holdout_negative.write_text(json.dumps(payload), encoding="utf-8")

    report = build_manifest([("TRAIN", train), ("HOLDOUT", holdout)])

    assert report["a4_area_dataset_ready"] is False
    assert "missing_negative_only_scene:HOLDOUT" in report["failure_reasons"]
