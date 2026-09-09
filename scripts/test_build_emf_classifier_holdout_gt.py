import hashlib
import json
import shutil
from pathlib import Path, PurePosixPath

import build_emf_classifier_holdout_gt as builder
import cv2
import numpy as np
import pytest


def _write_scene(
    capture_root: Path,
    *,
    scene: str,
    world: str,
    seed: int,
    negative_only: bool,
    frame_count: int,
    selected_target_class: str | None,
) -> Path:
    scene_dir = capture_root / "g4_screening_native" / "scenes" / scene
    for name in ("rgb", "depth", "camera", "semantic", "instance", "tf", "capture"):
        (scene_dir / name).mkdir(parents=True, exist_ok=True)
    target_counts = {
        name: int(not negative_only and name == selected_target_class)
        for name in builder.TARGET_CLASSES.values()
    }
    objects = []
    if selected_target_class is not None:
        semantic_label = next(
            category_id
            for category_id, name in builder.TARGET_CLASSES.items()
            if name == selected_target_class
        )
        objects.append(
            {
                "asset_id": f"asset_{selected_target_class}",
                "class_id": selected_target_class,
                "semantic_label": semantic_label,
            }
        )
    (scene_dir / "scene_manifest.json").write_text(
        json.dumps(
            {
                "schema_version": 2,
                "split": "val",
                "source_world_split": "val",
                "world_id": world,
                "world_sha256": builder.HOLDOUT_WORLD_SHA256[world],
                "scene_seed": seed,
                "trajectory_id": f"{world}_trajectory_{seed}",
                "negative_only": negative_only,
                "target_count_by_class": target_counts,
                "objects": objects,
                "trcrv10_g10_approach_sequence": {
                    "enabled": True,
                    "target_classes": sorted(builder.TARGET_CLASSES.values()),
                    "selected_target_class": selected_target_class,
                    "targets_per_positive_mission": 1,
                    "gt_runtime_forbidden": True,
                },
            },
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    (scene_dir / "capture_report.json").write_text(
        json.dumps(
            {
                "schema_version": 4,
                "capture_pass": True,
                "requested_frames": frame_count,
                "captured_frames": 0,
                "sensor_odom_sync": {
                    "maximum_skew_ns": 1,
                    "gate_maximum_skew_ns": 50_000_000,
                    "pass": True,
                },
                "records": [],
            },
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    return scene_dir


def _write_frame(
    scene_dir: Path,
    frame_index: int,
    *,
    category_id: int | None,
) -> tuple[int, int, int, int] | None:
    suffix = f"frame_{frame_index:02d}"
    rgb = np.zeros((24, 32, 3), dtype=np.uint8)
    rgb[:, :, 0] = frame_index % 255
    semantic = np.zeros((24, 32), dtype=np.uint8)
    instance = np.zeros((24, 32), dtype=np.uint16)
    bbox = None
    if category_id is not None:
        semantic[5:11, 7:16] = category_id
        instance[5:11, 7:16] = 100 + frame_index
        bbox = (7, 5, 9, 6)
    assert cv2.imwrite(str(scene_dir / "rgb" / f"{suffix}.png"), rgb)
    np.save(scene_dir / "depth" / f"{suffix}.npy", np.ones((24, 32), dtype=np.float32))
    np.save(scene_dir / "semantic" / f"{suffix}.npy", semantic)
    np.save(scene_dir / "instance" / f"{suffix}.npy", instance)
    (scene_dir / "camera" / f"{suffix}.json").write_text(
        json.dumps({"k": [20.0, 0.0, 16.0, 0.0, 20.0, 12.0, 0.0, 0.0, 1.0]}),
        encoding="utf-8",
    )
    for name in ("tf", "capture"):
        (scene_dir / name / f"{suffix}.json").write_text(
            json.dumps({"frame_index": frame_index}), encoding="utf-8"
        )
    report_path = scene_dir / "capture_report.json"
    report = json.loads(report_path.read_text(encoding="utf-8"))
    rgb_path = scene_dir / "rgb" / f"{suffix}.png"
    report["records"].append(
        {
            "frame_index": frame_index,
            "timestamp_ns": 1_000_000 + frame_index,
            "odom_timestamp_ns": 1_000_000 + frame_index,
            "sensor_odom_skew_ns": 0,
            "exact_four_sensor_timestamp": True,
            "paths": {
                "rgb": f"rgb/{suffix}.png",
                "depth": f"depth/{suffix}.npy",
                "semantic": f"semantic/{suffix}.npy",
                "instance": f"instance/{suffix}.npy",
                "camera": f"camera/{suffix}.json",
                "tf": f"tf/{suffix}.json",
                "capture": f"capture/{suffix}.json",
            },
            "rgb_sha256": hashlib.sha256(rgb_path.read_bytes()).hexdigest(),
        }
    )
    report["captured_frames"] = len(report["records"])
    report_path.write_text(json.dumps(report, sort_keys=True), encoding="utf-8")
    return bbox


def _image_row(
    image_id: int,
    scene_dir: Path,
    *,
    world: str,
    seed: int,
    frame_index: int,
    negative_only: bool,
) -> dict:
    suffix = f"frame_{frame_index:02d}"
    capture_root = scene_dir.parents[2]

    def relative(path: Path) -> str:
        return path.relative_to(capture_root).as_posix()

    return {
        "id": image_id,
        "file_name": relative(scene_dir / "rgb" / f"{suffix}.png"),
        "depth_file_name": relative(scene_dir / "depth" / f"{suffix}.npy"),
        "camera_file_name": relative(scene_dir / "camera" / f"{suffix}.json"),
        "semantic_file_name": relative(scene_dir / "semantic" / f"{suffix}.npy"),
        "instance_file_name": relative(scene_dir / "instance" / f"{suffix}.npy"),
        "capture_report": relative(scene_dir / "capture_report.json"),
        "scene_manifest": relative(scene_dir / "scene_manifest.json"),
        "width": 32,
        "height": 24,
        "scene": scene_dir.name,
        "world_id": world,
        "scene_seed": seed,
        "frame_index": frame_index,
        "negative_only": negative_only,
        "mission_id": scene_dir.name,
        "source_split": builder.SOURCE_SPLIT,
    }


def _valid_fixture(tmp_path: Path) -> tuple[Path, Path]:
    capture = tmp_path / "capture_holdout"
    images = []
    annotations = []
    image_id = 1
    annotation_id = 1
    holdout_worlds = sorted(builder.HOLDOUT_WORLD_SHA256)
    holdout_world = holdout_worlds[0]
    for class_index, (category_id, class_name) in enumerate(builder.TARGET_CLASSES.items()):
        scene_name = f"scene_{class_name}"
        world = holdout_worlds[class_index]
        scene_dir = _write_scene(
            capture,
            scene=scene_name,
            world=world,
            seed=1000 + category_id,
            negative_only=False,
            frame_count=builder.POSITIVE_PER_CLASS,
            selected_target_class=class_name,
        )
        for frame_index in range(builder.POSITIVE_PER_CLASS):
            bbox = _write_frame(scene_dir, frame_index, category_id=category_id)
            images.append(
                _image_row(
                    image_id,
                    scene_dir,
                    world=world,
                    seed=1000 + category_id,
                    frame_index=frame_index,
                    negative_only=False,
                )
            )
            annotations.append(
                {
                    "id": annotation_id,
                    "image_id": image_id,
                    "category_id": category_id,
                    "bbox": list(bbox),
                    "area": bbox[2] * bbox[3],
                    "iscrowd": 0,
                    "bbox_short_side_px": min(bbox[2], bbox[3]),
                }
            )
            image_id += 1
            annotation_id += 1
    negative_scene = _write_scene(
        capture,
        scene="scene_independent_unknown",
        world=holdout_world,
        seed=2001,
        negative_only=True,
        frame_count=2,
        selected_target_class=None,
    )
    for frame_index in range(2):
        _write_frame(negative_scene, frame_index, category_id=None)
        images.append(
            _image_row(
                image_id,
                negative_scene,
                world=holdout_world,
                seed=2001,
                frame_index=frame_index,
                negative_only=True,
            )
        )
        image_id += 1
    coco = {
        "info": {
            "semantic_gt_role": "offline_evaluator_only",
            "production_runtime_gt_used": False,
            "g10_domain_manifest_sha256": builder.G10_DOMAIN_MANIFEST_SHA256,
            "holdout_world_ids": sorted(builder.HOLDOUT_WORLD_SHA256),
        },
        "categories": [
            {"id": category_id, "name": class_name}
            for category_id, class_name in builder.TARGET_CLASSES.items()
        ],
        "images": images,
        "annotations": annotations,
        # False audit keys are permitted; their string values are not data sources.
        "VAL_NEW_read": False,
        "G5_V2_read": False,
        "G10_DEV_VAL_SEALED_read": False,
    }
    coco_path = tmp_path / "g10_holdout_coco.json"
    coco_path.write_text(json.dumps(coco, sort_keys=True), encoding="utf-8")
    return coco_path, capture


def test_build_is_deterministic_sha_locked_and_gt_runtime_forbidden(tmp_path: Path):
    coco, capture = _valid_fixture(tmp_path)
    first_dir = tmp_path / "result_a"
    second_dir = tmp_path / "result_b"

    first = builder.build_dataset(coco, capture, first_dir)
    second = builder.build_dataset(coco, capture, second_dir)

    assert first == second
    assert first["pass"] is True
    assert first["source_split"] == "G10_HOLDOUT"
    assert first["g10_domain_manifest_sha256"] == builder.G10_DOMAIN_MANIFEST_SHA256
    assert first["holdout_world_ids"] == sorted(builder.HOLDOUT_WORLD_SHA256)
    assert first["counts"] == {
        "background_or_unknown": 2,
        "metal_can": builder.POSITIVE_PER_CLASS,
        "paper_litter": builder.POSITIVE_PER_CLASS,
        "plastic_bottle": builder.POSITIVE_PER_CLASS,
    }
    assert first["negative_only_scene_count"] == 1
    assert first["negative_only_frame_count"] == 2
    assert first["offline_gt_development_only"] is True
    assert first["production_runtime_gt_forbidden"] is True
    assert first["training_performed"] is False
    assert first["threshold_selected"] is False
    assert first["threshold_frozen"] is False
    assert first["identity_lock_sha256"] == builder._canonical_sha256(first["records"])
    assert len({row["record_id"] for row in first["records"]}) == len(first["records"])
    assert all(row["source_identity"]["source_split"] == "G10_HOLDOUT" for row in first["records"])
    assert all(row["production_runtime_eligible"] is False for row in first["records"])
    for row in first["records"]:
        crop_path = first_dir / row["crop_path"]
        assert hashlib.sha256(crop_path.read_bytes()).hexdigest() == row["crop_sha256"]
        assert set(row["source_sha256"]) == {
            "rgb",
            "depth",
            "camera",
            "semantic",
            "instance",
            "scene_manifest",
            "capture_report",
        }
        if row["class_name"] == "background_or_unknown":
            assert row["source_identity"]["negative_only"] is True
            assert row["source_identity"]["annotation_id"] is None

    first_bytes = (first_dir / "CLASSIFIER_HOLDOUT_GT_MANIFEST.json").read_bytes()
    second_bytes = (second_dir / "CLASSIFIER_HOLDOUT_GT_MANIFEST.json").read_bytes()
    assert first_bytes == second_bytes


@pytest.mark.parametrize(
    "marker", ["G5", "G5_V2", "G5V2", "VAL_NEW", "DEV_VAL", "SEALED"]
)
def test_forbidden_path_markers_are_rejected(tmp_path: Path, marker: str):
    coco, capture = _valid_fixture(tmp_path)
    forbidden_output = tmp_path / marker / "result"
    with pytest.raises(builder.HoldoutContractError, match="forbidden marker"):
        builder.build_dataset(coco, capture, forbidden_output)


def test_rejects_val_alias_and_requires_exact_holdout_split(tmp_path: Path):
    coco, capture = _valid_fixture(tmp_path)
    payload = json.loads(coco.read_text(encoding="utf-8"))
    payload["images"][0]["source_split"] = "val"
    coco.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(builder.HoldoutContractError, match="source_split must be"):
        builder.build_dataset(coco, capture, tmp_path / "result")


def test_missing_class_and_negative_scene_fail_closed_without_output(tmp_path: Path):
    coco, capture = _valid_fixture(tmp_path)
    payload = json.loads(coco.read_text(encoding="utf-8"))
    paper_image_ids = {
        row["id"] for row in payload["images"] if row["scene"] == "scene_paper_litter"
    }
    payload["annotations"] = [
        row for row in payload["annotations"] if row["image_id"] not in paper_image_ids
    ]
    coco.write_text(json.dumps(payload), encoding="utf-8")
    output = tmp_path / "missing_class_result"
    with pytest.raises(builder.HoldoutContractError, match="exactly match semantic target IDs"):
        builder.build_dataset(coco, capture, output)
    assert not output.exists()

    coco, capture = _valid_fixture(tmp_path / "no_negative")
    payload = json.loads(coco.read_text(encoding="utf-8"))
    payload["images"] = [row for row in payload["images"] if not row["negative_only"]]
    coco.write_text(json.dumps(payload), encoding="utf-8")
    shutil.rmtree(
        capture
        / "g4_screening_native"
        / "scenes"
        / "scene_independent_unknown"
    )
    with pytest.raises(builder.HoldoutContractError, match="negative-only scene/frame"):
        builder.build_dataset(coco, capture, tmp_path / "no_negative_result")


def test_negative_only_target_pixels_and_path_escape_are_rejected(tmp_path: Path):
    coco, capture = _valid_fixture(tmp_path)
    negative_semantic = next(
        (capture / "g4_screening_native" / "scenes" / "scene_independent_unknown" / "semantic").glob("*.npy")
    )
    semantic = np.load(negative_semantic, allow_pickle=False)
    semantic[0:2, 0:2] = 1
    np.save(negative_semantic, semantic)
    output = tmp_path / "negative_gt_result"
    with pytest.raises(builder.HoldoutContractError, match="contains target semantic pixels"):
        builder.build_dataset(coco, capture, output)
    assert not output.exists()

    coco, capture = _valid_fixture(tmp_path / "escape")
    payload = json.loads(coco.read_text(encoding="utf-8"))
    outside = tmp_path / "outside.png"
    assert cv2.imwrite(str(outside), np.zeros((24, 32, 3), dtype=np.uint8))
    payload["images"][0]["file_name"] = str(outside.resolve())
    coco.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(builder.HoldoutContractError, match="capture-root-relative POSIX path"):
        builder.build_dataset(coco, capture, tmp_path / "escape_result")


def test_partial_capture_and_output_reuse_are_rejected(tmp_path: Path):
    coco, capture = _valid_fixture(tmp_path)
    report = next(capture.rglob("capture_report.json"))
    payload = json.loads(report.read_text(encoding="utf-8"))
    payload["captured_frames"] -= 1
    report.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(builder.HoldoutContractError, match="partial"):
        builder.build_dataset(coco, capture, tmp_path / "partial_result")

    coco, capture = _valid_fixture(tmp_path / "reuse")
    output = tmp_path / "existing_result"
    output.mkdir()
    with pytest.raises(builder.HoldoutContractError, match="already exists"):
        builder.build_dataset(coco, capture, output)


def test_coco_must_cover_every_scene_and_every_captured_frame(tmp_path: Path):
    coco, capture = _valid_fixture(tmp_path)
    payload = json.loads(coco.read_text(encoding="utf-8"))
    removed = payload["images"].pop()
    assert removed["negative_only"] is True
    coco.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(builder.HoldoutContractError, match="every captured frame"):
        builder.build_dataset(coco, capture, tmp_path / "incomplete_coco_result")

    coco, capture = _valid_fixture(tmp_path / "extra_scene")
    orphan = capture / "g4_screening_native" / "scenes" / "scene_orphan"
    orphan.mkdir()
    with pytest.raises(builder.HoldoutContractError, match="scene set"):
        builder.build_dataset(coco, capture, tmp_path / "extra_scene_result")


def test_unselected_positive_frame_is_still_fully_decoded_and_rejected(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    coco, capture = _valid_fixture(tmp_path)
    payload = json.loads(coco.read_text(encoding="utf-8"))
    plastic = [
        (
            builder._selection_rank("plastic_bottle", image, annotation),
            image,
        )
        for image in payload["images"]
        for annotation in payload["annotations"]
        if annotation["image_id"] == image["id"] and annotation["category_id"] == 1
    ]
    _, unselected = max(plastic)
    semantic_path = capture.joinpath(*PurePosixPath(unselected["semantic_file_name"]).parts)
    semantic_path.write_bytes(b"not-a-numpy-array")
    monkeypatch.setattr(builder, "POSITIVE_PER_CLASS", builder.POSITIVE_PER_CLASS - 1)

    with pytest.raises(builder.HoldoutContractError, match="cannot safely load frame tensors"):
        builder.build_dataset(coco, capture, tmp_path / "unselected_bad_result")


def test_instance_and_capture_record_identity_fail_closed(tmp_path: Path):
    coco, capture = _valid_fixture(tmp_path)
    instance_path = next(
        (capture / "g4_screening_native" / "scenes" / "scene_plastic_bottle" / "instance").glob("*.npy")
    )
    instance = np.load(instance_path, allow_pickle=False)
    instance[:] = 0
    np.save(instance_path, instance)
    with pytest.raises(builder.HoldoutContractError, match="exactly one nonzero instance"):
        builder.build_dataset(coco, capture, tmp_path / "zero_instance_result")

    coco, capture = _valid_fixture(tmp_path / "record")
    report_path = next(capture.rglob("capture_report.json"))
    report = json.loads(report_path.read_text(encoding="utf-8"))
    report["records"][0]["exact_four_sensor_timestamp"] = False
    report_path.write_text(json.dumps(report), encoding="utf-8")
    with pytest.raises(builder.HoldoutContractError, match="exact four-sensor timestamp"):
        builder.build_dataset(coco, capture, tmp_path / "bad_record_result")


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (lambda payload: payload["categories"].append(dict(payload["categories"][0])), "exactly three"),
        (lambda payload: payload["images"][0].__setitem__("scene_seed", "1001"), "scene_seed must be an integer"),
        (lambda payload: payload["images"][0].__setitem__("negative_only", "false"), "negative_only must be a boolean"),
        (lambda payload: payload["images"][0].__setitem__("file_name", "..\\escape.png"), "capture-root-relative POSIX path"),
    ],
)
def test_strict_types_categories_and_portable_paths_fail_closed(
    tmp_path: Path, mutation, message: str
):
    coco, capture = _valid_fixture(tmp_path)
    payload = json.loads(coco.read_text(encoding="utf-8"))
    mutation(payload)
    coco.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(builder.HoldoutContractError, match=message):
        builder.build_dataset(coco, capture, tmp_path / "strict_result")


def test_concurrent_writer_lock_fails_without_partial_output(tmp_path: Path):
    coco, capture = _valid_fixture(tmp_path)
    output = tmp_path / "locked_result"
    lock = output.parent / f".{output.name}.lock"
    lock.write_text("other-writer\n", encoding="ascii")
    with pytest.raises(builder.HoldoutContractError, match="concurrent output writer lock"):
        builder.build_dataset(coco, capture, output)
    assert not output.exists()
    assert lock.read_text(encoding="ascii") == "other-writer\n"
