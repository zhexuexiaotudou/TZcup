import hashlib
import json
from pathlib import Path

import c3_holdout_native_worker as worker
import numpy as np
import pytest
from PIL import Image


def _source_hashes(token):
    return {
        field: hashlib.sha256((field + token).encode()).hexdigest()
        for field in worker.REQUIRED_SOURCE_SHA_FIELDS
    }


def _record(root, class_name, index, *, negative_only=False, scene=None):
    scene = scene or ("scene_negative" if negative_only else "scene_{}".format(class_name))
    annotation_id = None if negative_only else 10000 + index
    world_by_class = {
        "background_or_unknown": "g10v15_val_w01_07_service_road",
        "plastic_bottle": "g10v15_val_w01_07_service_road",
        "metal_can": "g10v15_val_w02_08_mixed_curb_vegetation",
        "paper_litter": "g10v15_val_w03_09_light_paver_pedestrian",
    }
    source_identity = {
        "source_split": worker.SOURCE_SPLIT,
        "image_id": index + 1,
        "annotation_id": annotation_id,
        "world_id": world_by_class[class_name],
        "scene": scene,
        "scene_seed": 2000 if negative_only else 1000 + worker.PRODUCT_CLASSES.index(class_name),
        "frame_index": index,
        "negative_only": negative_only,
    }
    source_hashes = _source_hashes("{}:{}".format(class_name, index))
    source_identity_sha = worker.canonical_sha256(
        {"identity": source_identity, "sha256": source_hashes}
    )
    bbox = [0, 0, 8, 8]
    record_id = "emf-holdout-" + worker.canonical_sha256(
        {
            "source_identity_sha256": source_identity_sha,
            "class_name": class_name,
            "crop_bbox_xyxy": bbox,
        }
    )[:24]
    relative = Path("crops") / class_name / (record_id + ".png")
    crop_path = root / relative
    crop_path.parent.mkdir(parents=True, exist_ok=True)
    image = Image.new("RGB", (8, 8), color=(index % 255, 10, 20))
    image.save(str(crop_path), format="PNG")
    return {
        "record_id": record_id,
        "class_name": class_name,
        "crop_path": relative.as_posix(),
        "crop_sha256": worker.base.sha256(crop_path),
        "crop_bbox_xyxy": bbox,
        "source_identity": source_identity,
        "source_identity_sha256": source_identity_sha,
        "source_paths": {
            field: "g4_screening_native/scenes/{}/{}.data".format(scene, field)
            for field in worker.REQUIRED_SOURCE_SHA_FIELDS
        },
        "source_sha256": source_hashes,
        "offline_gt_development_only": True,
        "production_runtime_eligible": False,
    }


def _manifest(root):
    records = []
    next_index = 0
    for class_name in worker.PRODUCT_CLASSES[1:]:
        for _ in range(worker.POSITIVE_PER_CLASS):
            records.append(_record(root, class_name, next_index))
            next_index += 1
    for negative_index in range(2):
        records.append(
            _record(
                root,
                "background_or_unknown",
                next_index,
                negative_only=True,
                scene="scene_negative",
            )
        )
        next_index += 1
    records.sort(key=lambda row: row["record_id"])
    payload = {
        "schema_version": worker.SCHEMA_VERSION,
        "protocol_id": "EMFJ6V3",
        "stage": "CLASSIFIER_HOLDOUT_OFFLINE_GT_DEVELOPMENT",
        "source_split": worker.SOURCE_SPLIT,
        "g10_domain_manifest_sha256": worker.DOMAIN_MANIFEST_SHA256,
        "holdout_world_ids": sorted(worker.HOLDOUT_WORLDS),
        "input_coco_sha256": "a" * 64,
        "all_validated_source_frames_sha256": "b" * 64,
        "selection_contract": {
            "seed": 20260824,
            "positive_per_class": 60,
            "background_per_negative_frame": 1,
            "positive_crop_scale": 4.0,
            "minimum_crop_side": 64,
            "background_crop_side": 96,
            "background_max_gt_iou_exclusive": 0.1,
        },
        "counts": {
            "background_or_unknown": 2,
            "metal_can": 60,
            "paper_litter": 60,
            "plastic_bottle": 60,
        },
        "negative_only_scene_count": 1,
        "negative_only_frame_count": 2,
        "records": records,
        "identity_lock_sha256": worker.canonical_sha256(records),
        "offline_gt_development_only": True,
        "production_runtime_gt_forbidden": True,
        "training_performed": False,
        "threshold_selected": False,
        "threshold_frozen": False,
        "formal_product_evidence": False,
        "pass": True,
        "atomic_output_contract": {
            "visibility": "same_filesystem_directory_rename",
            "concurrent_writer_policy": "exclusive_sibling_lock_required",
            "power_loss_durability_guaranteed": False,
        },
    }
    payload["canonical_manifest_sha256"] = worker.canonical_sha256(payload)
    return payload


def _write_manifest(path, payload):
    path.write_text(
        json.dumps(payload, ensure_ascii=False, sort_keys=True, indent=2) + "\n",
        encoding="utf-8",
    )
    return worker.base.sha256(path)


def _refresh(payload):
    payload["identity_lock_sha256"] = worker.canonical_sha256(payload["records"])
    payload.pop("canonical_manifest_sha256", None)
    payload["canonical_manifest_sha256"] = worker.canonical_sha256(payload)


def test_frozen_artifact_runtime_class_and_preprocess_contracts(tmp_path):
    assert worker.MODEL_SHA256 == (
        "013afdc86a673cb2354f4559c165301d5abda1c5878bb523a5995e483d4cc90a"
    )
    assert worker.RUNTIME_IMAGE_DIGEST == (
        "sha256:47aa058918aa7b09343c05ccbd23ccef976006a07b579143e9adde34a937b419"
    )
    assert worker.RUNTIME_VERSIONS == {
        "python": "3.6.9",
        "tensorflow": "1.15.5",
        "numpy": "1.18.5",
        "pillow": "8.1.0",
        "h5py": "2.10.0",
    }
    assert worker.CLASS_ORDER == (
        "cardboard",
        "glass",
        "metal",
        "paper",
        "plastic",
        "trash",
    )
    assert worker.CLASS_MAPPING == {
        "cardboard": "background_or_unknown",
        "glass": "background_or_unknown",
        "metal": "metal_can",
        "paper": "paper_litter",
        "plastic": "plastic_bottle",
        "trash": "background_or_unknown",
    }
    source = Path(worker.__file__).read_text(encoding="utf-8")
    assert "import prediction" not in source
    assert "from prediction" not in source
    assert "validate_runtime_isolation" in source

    crop = tmp_path / "rgba.png"
    source_image = Image.new("RGBA", (2, 2))
    source_image.putdata(
        [
            (255, 0, 0, 1),
            (0, 255, 0, 2),
            (0, 0, 255, 3),
            (255, 255, 255, 4),
        ]
    )
    source_image.save(str(crop))
    batch = worker.preprocess_crop(crop, np, Image)
    assert batch.shape == (1, 300, 300, 3)
    assert batch.dtype == np.float64
    np.testing.assert_array_equal(batch[0, 0, 0], np.array([1.0, 0.0, 0.0]))
    np.testing.assert_array_equal(batch[0, -1, -1], np.array([1.0, 1.0, 1.0]))


def test_manifest_and_every_crop_identity_are_verified(tmp_path):
    data_root = tmp_path / "holdout"
    manifest = _manifest(data_root)
    manifest_path = data_root / "CLASSIFIER_HOLDOUT_GT_MANIFEST.json"
    manifest_sha = _write_manifest(manifest_path, manifest)

    loaded = worker.load_manifest(manifest_path, manifest_sha)
    validated = worker.validate_manifest(loaded, data_root.resolve())

    assert len(validated) == 182
    assert [item["record"]["record_id"] for item in validated] == sorted(
        row["record_id"] for row in manifest["records"]
    )
    assert {item["record"]["source_identity"]["source_split"] for item in validated} == {
        "G10_HOLDOUT"
    }
    assert sum(
        item["record"]["source_identity"]["negative_only"] for item in validated
    ) == 2


def test_canonical_manifest_file_and_identity_locks_fail_closed(tmp_path):
    data_root = tmp_path / "holdout"
    manifest = _manifest(data_root)
    manifest_path = data_root / "CLASSIFIER_HOLDOUT_GT_MANIFEST.json"
    _write_manifest(manifest_path, manifest)

    with pytest.raises(worker.WorkerError, match="file SHA-256 mismatch"):
        worker.load_manifest(manifest_path, "0" * 64)

    manifest["threshold_selected"] = True
    changed_sha = _write_manifest(manifest_path, manifest)
    with pytest.raises(worker.WorkerError, match="canonical SHA-256 mismatch"):
        worker.load_manifest(manifest_path, changed_sha)

    manifest = _manifest(data_root)
    manifest["identity_lock_sha256"] = "0" * 64
    with pytest.raises(worker.WorkerError, match="identity-lock"):
        worker.validate_manifest(manifest, data_root.resolve())


def test_wrong_domain_counts_and_runtime_flags_fail_closed(tmp_path):
    data_root = tmp_path / "holdout"
    manifest = _manifest(data_root)

    manifest["source_split"] = "val"
    with pytest.raises(worker.WorkerError, match="source_split"):
        worker.validate_manifest(manifest, data_root.resolve())

    manifest = _manifest(data_root)
    manifest["production_runtime_gt_forbidden"] = False
    with pytest.raises(worker.WorkerError, match="production_runtime_gt_forbidden"):
        worker.validate_manifest(manifest, data_root.resolve())

    manifest = _manifest(data_root)
    manifest["counts"]["paper_litter"] = 59
    with pytest.raises(worker.WorkerError, match="target class count differs"):
        worker.validate_manifest(manifest, data_root.resolve())


def test_record_crop_source_and_negative_only_tampering_fail_closed(tmp_path):
    data_root = tmp_path / "holdout"
    manifest = _manifest(data_root)
    background = next(
        row for row in manifest["records"] if row["class_name"] == "background_or_unknown"
    )
    background["source_identity"]["negative_only"] = False
    source_hashes = background["source_sha256"]
    background["source_identity_sha256"] = worker.canonical_sha256(
        {"identity": background["source_identity"], "sha256": source_hashes}
    )
    _refresh(manifest)
    with pytest.raises(worker.WorkerError, match="negative-only frame"):
        worker.validate_manifest(manifest, data_root.resolve())

    manifest = _manifest(data_root)
    first = manifest["records"][0]
    crop = data_root / first["crop_path"]
    crop.write_bytes(b"changed")
    with pytest.raises(worker.WorkerError, match="crop SHA-256 mismatch"):
        worker.validate_manifest(manifest, data_root.resolve())

    manifest = _manifest(data_root)
    first = manifest["records"][0]
    first["crop_path"] = "../escape.png"
    _refresh(manifest)
    with pytest.raises(worker.WorkerError, match="escapes"):
        worker.validate_manifest(manifest, data_root.resolve())


@pytest.mark.parametrize(
    "marker", ["G5", "G5_V2", "G5V2", "VAL_NEW", "DEV_VAL", "SEALED"]
)
def test_forbidden_markers_fail_closed(marker):
    with pytest.raises(worker.WorkerError, match="forbidden"):
        worker.base.reject_forbidden("C:/data/{}/manifest.json".format(marker), "test")


def test_probability_vector_mapping_and_metrics_are_bounded():
    probabilities, source_class, predicted = worker.probabilities_and_prediction(
        np.array([[0.05, 0.05, 0.1, 0.2, 0.55, 0.05]], dtype=np.float32),
        np,
    )
    assert source_class == "plastic"
    assert predicted == "plastic_bottle"
    assert set(probabilities) == set(worker.CLASS_ORDER)

    with pytest.raises(worker.WorkerError, match="normalized"):
        worker.probabilities_and_prediction(np.ones((1, 6)), np)
    with pytest.raises(worker.WorkerError, match="shape"):
        worker.probabilities_and_prediction(np.ones((1, 5)), np)

    rows = [
        {
            "actual_product_class": actual,
            "predicted_product_class": predicted,
        }
        for actual, predicted in (
            ("background_or_unknown", "background_or_unknown"),
            ("plastic_bottle", "plastic_bottle"),
            ("metal_can", "background_or_unknown"),
            ("paper_litter", "paper_litter"),
        )
    ]
    metrics = worker.classification_metrics(rows)
    assert metrics["background_specificity"] == 1.0
    assert metrics["per_class"]["metal_can"]["recall"] == 0.0
