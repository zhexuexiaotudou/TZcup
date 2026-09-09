import json
from pathlib import Path

import c3_native_worker as worker
import numpy as np
import pytest
from PIL import Image


def _record(record_id="G10_TRAIN:scene_1:1:background", class_id="background"):
    return {
        "record_id": record_id,
        "source_split": "G10_TRAIN",
        "class_id": class_id,
        "rgb_path": "F:\\Project\\TZcup-product-evidence\\capture\\frame.png",
        "proposal_bbox_native_xyxy": [0.0, 0.0, 2.0, 2.0],
        "crop_source": "offline_gt_box_development_only",
        "gt_role": "offline_training_label_only",
        "production_runtime_gt_used": False,
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


def _identity(record):
    return {
        "protocol_id": "EMFJ6V3",
        "model_id": "c4_prithiv_trash_net_siglip2",
        "source_manifest_sha256": worker.MANIFEST_SHA256,
        "rows": [
            {
                "record_id": record["record_id"],
                "actual_product_class": record["class_id"],
                "bbox_xyxy": record["proposal_bbox_native_xyxy"],
                "relative_file": "capture/frame.png",
                "source_image_sha256": "a" * 64,
            }
        ],
    }


def test_frozen_c3_runtime_artifact_and_class_contracts():
    assert worker.MODEL_SHA256 == "013afdc86a673cb2354f4559c165301d5abda1c5878bb523a5995e483d4cc90a"
    assert worker.RUNTIME_IMAGE_DIGEST == (
        "sha256:47aa058918aa7b09343c05ccbd23ccef976006a07b579143e9adde34a937b419"
    )
    assert worker.CLASS_ORDER == ("cardboard", "glass", "metal", "paper", "plastic", "trash")
    assert worker.CLASS_ORDER_SOURCE_SHA256 == (
        "1922931ce576f39ff47cf5fbef5c48efcddcde49ef2631550364113d6ac6b0b8"
    )
    assert worker.CLASS_MAPPING == {
        "metal": "metal_can",
        "paper": "paper_litter",
        "plastic": "plastic_bottle",
    }
    source = Path(worker.__file__).read_text(encoding="utf-8")
    assert "import prediction" not in source
    assert "from prediction" not in source


def test_dataset_identity_bbox_and_image_sha_lock():
    record = _record()
    identities = worker.validate_dataset_payloads(
        _manifest([record]),
        _identity(record),
        expected_count=1,
        expected_class_counts={"background": 1},
    )
    assert identities[record["record_id"]]["source_image_sha256"] == "a" * 64

    changed = json.loads(json.dumps(_identity(record)))
    changed["rows"][0]["bbox_xyxy"][2] = 3.0
    with pytest.raises(worker.WorkerError, match="bboxes differ"):
        worker.validate_dataset_payloads(
            _manifest([record]),
            changed,
            expected_count=1,
            expected_class_counts={"background": 1},
        )


def test_rebase_requires_prefix_and_containment(tmp_path):
    data_root = tmp_path / "data"
    data_root.mkdir()
    actual, relative = worker.rebase_source_path(
        "F:\\Project\\TZcup-product-evidence\\capture\\frame.png",
        "F:\\Project\\TZcup-product-evidence",
        data_root,
    )
    assert actual == (data_root / "capture" / "frame.png").resolve()
    assert relative == "capture/frame.png"

    with pytest.raises(worker.WorkerError, match="outside the explicit source prefix"):
        worker.rebase_source_path("C:\\elsewhere\\frame.png", "C:\\fixed", data_root)
    with pytest.raises(worker.WorkerError, match="escapes"):
        worker.rebase_source_path("C:\\fixed\\..\\frame.png", "C:\\fixed", data_root)


@pytest.mark.parametrize("marker", ["G5", "G5_V2", "VAL_NEW", "DEV_VAL", "SEALED"])
def test_forbidden_markers_fail_closed(marker):
    with pytest.raises(worker.WorkerError, match="forbidden"):
        worker.reject_forbidden(f"C:/data/{marker}/input.json", "test")


def test_preprocess_is_rgb_nearest_nhwc_uint8_div_255():
    source = Image.new("RGBA", (2, 2))
    source.putdata(
        [
            (255, 0, 0, 1),
            (0, 255, 0, 2),
            (0, 0, 255, 3),
            (255, 255, 255, 4),
        ]
    )
    batch = worker.preprocess_crop(source, (0.0, 0.0, 2.0, 2.0), np, Image)
    assert batch.shape == (1, 300, 300, 3)
    assert batch.dtype == np.float64
    np.testing.assert_array_equal(batch[0, 0, 0], np.array([1.0, 0.0, 0.0]))
    np.testing.assert_array_equal(batch[0, 0, -1], np.array([0.0, 1.0, 0.0]))
    np.testing.assert_array_equal(batch[0, -1, 0], np.array([0.0, 0.0, 1.0]))
    np.testing.assert_array_equal(batch[0, -1, -1], np.array([1.0, 1.0, 1.0]))


def test_metrics_include_background_specificity():
    rows = [
        {"actual_product_class": actual, "predicted_product_class": predicted}
        for actual, predicted in (
            ("background", "background"),
            ("plastic_bottle", "plastic_bottle"),
            ("metal_can", "background"),
            ("paper_litter", "paper_litter"),
        )
    ]
    metrics = worker.classification_metrics(rows)
    assert metrics["background_specificity"] == 1.0
    assert metrics["per_class"]["metal_can"]["recall"] == 0.0


def test_mount_resolution_uses_most_specific_read_only_mount():
    mounts = worker.parse_mounts("/dev/sda / ext4 ro 0 0\n/dev/sdb /data ext4 ro 0 0\n")
    assert worker.mount_is_read_only(Path("/data/a.png"), mounts) is True
    writable = worker.parse_mounts("/dev/sda / ext4 ro 0 0\n/dev/sdb /data ext4 rw 0 0\n")
    assert worker.mount_is_read_only(Path("/data/a.png"), writable) is False
