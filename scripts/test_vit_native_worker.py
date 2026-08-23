import hashlib
import json
from pathlib import Path

import pytest
import vit_native_worker as worker


def test_frozen_model_contracts_are_complete_and_many_to_one_mapping_is_exact():
    assert set(worker.MODEL_CONTRACTS) == {
        "c5_dima806_garbage_types_vit",
        "c6_giecom_recycling_vit",
    }
    for contract in worker.MODEL_CONTRACTS.values():
        assert set(contract["artifacts"]) == {
            "model.safetensors",
            "config.json",
            "preprocessor_config.json",
            "README.md",
            next(
                name
                for name in contract["artifacts"]
                if name.startswith("transformers_v")
            ),
        }
        assert all(len(value) == 64 for value in contract["artifacts"].values())

    c6 = worker.MODEL_CONTRACTS["c6_giecom_recycling_vit"]
    probabilities = {name: 0.0 for name in c6["class_order"]}
    probabilities.update(
        {"aluminium": 0.2, "paper": 0.1, "hard plastic": 0.3, "soft plastics": 0.25}
    )
    assert worker.mapped_target_probabilities(probabilities, c6) == {
        "metal_can": 0.2,
        "paper_litter": 0.1,
        "plastic_bottle": 0.55,
    }
    assert "takeaway cups" not in {
        source for sources in c6["target_sources"].values() for source in sources
    }


@pytest.mark.parametrize(
    "marker", ["G5", "g5-v2", "G5V2", "VAL_NEW", "dev-val", "SEALED_FINAL"]
)
def test_forbidden_dataset_markers_fail_closed(marker):
    with pytest.raises(worker.WorkerError, match="forbidden"):
        worker.reject_forbidden(f"root/{marker}/train", field="fixture")


def _record(record_id="G10_TRAIN:scene:0:background"):
    return {
        "record_id": record_id,
        "source_split": "G10_TRAIN",
        "class_id": "background",
        "rgb_path": "OLD/root/frame.png",
        "proposal_bbox_native_xyxy": [1, 2, 10, 12],
        "crop_source": "offline_gt_box_development_only",
        "gt_role": "offline_training_label_only",
        "production_runtime_gt_used": False,
    }


def _lock_row(record_id="G10_TRAIN:scene:0:background"):
    return {
        "record_id": record_id,
        "relative_file": "root/frame.png",
        "source_image_sha256": "a" * 64,
        "bbox_xyxy": [1.0, 2.0, 10.0, 12.0],
        "actual_product_class": "background",
    }


def _manifest(records):
    return {
        "schema_version": 1,
        "protocol": "CLOSE-RANGE-RGBD-RECOVERY-V12-DEV-ONLY",
        "stage": "CRRGBDV12-DEV-ONLY-GT-CROPS",
        "source_split": "G10_TRAIN",
        "development_only": True,
        "formal_eligible": False,
        "production_runtime_gt_used": False,
        "pass": True,
        "records": records,
    }


def _identity(rows):
    return {
        "protocol_id": "EMFJ6V3",
        "model_id": "c4_prithiv_trash_net_siglip2",
        "source_manifest_sha256": worker.EXPECTED_MANIFEST_SHA256,
        "rows": rows,
    }


def test_manifest_and_identity_lock_require_exact_ids_hashes_bboxes_and_gt_boundary():
    by_id = worker.validate_dataset_payloads(
        _manifest([_record()]),
        _identity([_lock_row()]),
        expected_count=1,
        expected_class_counts={"background": 1},
    )
    assert set(by_id) == {"G10_TRAIN:scene:0:background"}

    bad = _lock_row()
    bad["bbox_xyxy"] = [1, 2, 10, 13]
    with pytest.raises(worker.WorkerError, match="bboxes differ"):
        worker.validate_dataset_payloads(
            _manifest([_record()]),
            _identity([bad]),
            expected_count=1,
            expected_class_counts={"background": 1},
        )

    leaked = _record()
    leaked["production_runtime_gt_used"] = True
    with pytest.raises(worker.WorkerError, match="production runtime GT"):
        worker.validate_dataset_payloads(
            _manifest([leaked]),
            _identity([_lock_row()]),
            expected_count=1,
            expected_class_counts={"background": 1},
        )


def test_rebase_rejects_prefix_and_parent_escape(tmp_path: Path):
    data_root = tmp_path / "data"
    data_root.mkdir()
    actual, relative = worker.rebase_source_path("OLD/root/frame.png", "OLD", data_root)
    assert actual == data_root / "root" / "frame.png"
    assert relative == "root/frame.png"
    with pytest.raises(worker.WorkerError, match="source prefix"):
        worker.rebase_source_path("OTHER/root/frame.png", "OLD", data_root)
    with pytest.raises(worker.WorkerError, match="escapes"):
        worker.rebase_source_path("OLD/../frame.png", "OLD", data_root)


def test_artifact_hashes_and_unsafe_weight_formats_fail_closed(tmp_path: Path):
    config = {
        "architectures": ["ViTForImageClassification"],
        "id2label": {"0": "zero"},
    }
    files = {
        "model.safetensors": b"safe header fixture",
        "config.json": json.dumps(config).encode(),
        "preprocessor_config.json": b"{}",
        "README.md": b"license fixture",
        "transformers_v1_image_processing_vit.py": b"source fixture",
    }
    for name, payload in files.items():
        (tmp_path / name).write_bytes(payload)
    contract = {
        "artifacts": {
            name: hashlib.sha256(payload).hexdigest() for name, payload in files.items()
        },
        "class_order": ("zero",),
    }
    worker.verify_artifacts(tmp_path, contract)
    (tmp_path / "pytorch_model.bin").write_bytes(b"unsafe")
    with pytest.raises(worker.WorkerError, match="non-safetensors"):
        worker.verify_artifacts(tmp_path, contract)


def test_mount_parser_uses_longest_read_only_mount():
    mounts = worker.parse_mounts(
        "overlay / overlay ro,relatime 0 0\n/dev/sda /model ext4 ro,relatime 0 0\n"
    )
    assert worker.mount_is_read_only(Path("/model/config.json"), mounts)
    assert worker.mount_is_read_only(Path("/tmp"), mounts)
