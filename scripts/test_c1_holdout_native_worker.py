import hashlib
import json
import shutil
from pathlib import Path, PurePosixPath
from types import SimpleNamespace

import build_emf_classifier_holdout_gt as holdout_builder
import c1_holdout_native_worker as worker
import cv2
import numpy as np
import pytest
from test_build_emf_classifier_holdout_gt import _valid_fixture


@pytest.fixture(scope="module")
def holdout_dataset(tmp_path_factory):
    root = tmp_path_factory.mktemp("c1_holdout_worker")
    coco, capture = _valid_fixture(root)
    output = root / "holdout_output"
    holdout_builder.build_dataset(coco, capture, output)
    manifest = output / "CLASSIFIER_HOLDOUT_GT_MANIFEST.json"
    return output, manifest


def _resign_manifest(payload: dict) -> None:
    unsigned = dict(payload)
    unsigned.pop("canonical_manifest_sha256", None)
    payload["canonical_manifest_sha256"] = worker.canonical_sha256(unsigned)


def test_valid_holdout_manifest_and_crops_are_fully_locked(holdout_dataset):
    crop_root, manifest = holdout_dataset
    manifest_sha = hashlib.sha256(manifest.read_bytes()).hexdigest()

    payload, rows = worker.load_dataset(manifest, manifest_sha, crop_root)

    assert len(rows) == 182
    assert payload["counts"] == {
        "background_or_unknown": 2,
        "metal_can": 60,
        "paper_litter": 60,
        "plastic_bottle": 60,
    }
    assert {row["source_identity"]["world_id"] for row in rows} == worker.HOLDOUT_WORLDS
    assert all(hashlib.sha256(row["crop_bytes"]).hexdigest() == row["crop_sha256"] for row in rows)


@pytest.mark.parametrize(
    ("mutate", "message"),
    [
        (lambda payload: payload.__setitem__("threshold_selected", True), "unsafe manifest flag"),
        (
            lambda payload: payload["counts"].__setitem__("plastic_bottle", 59),
            "positive HOLDOUT class",
        ),
        (
            lambda payload: payload["selection_contract"].__setitem__("seed", 1),
            "selection contract changed",
        ),
    ],
)
def test_manifest_flags_counts_and_selection_fail_closed(
    holdout_dataset, mutate, message
):
    _, manifest = holdout_dataset
    payload = json.loads(manifest.read_text(encoding="utf-8"))
    mutate(payload)
    _resign_manifest(payload)
    with pytest.raises(worker.WorkerError, match=message):
        worker.validate_manifest(payload)


def test_internal_hash_and_forbidden_source_fail_closed(holdout_dataset):
    _, manifest = holdout_dataset
    payload = json.loads(manifest.read_text(encoding="utf-8"))
    payload["records"][0]["crop_bbox_xyxy"][0] += 1
    _resign_manifest(payload)
    with pytest.raises(worker.WorkerError, match="identity lock"):
        worker.validate_manifest(payload)

    payload = json.loads(manifest.read_text(encoding="utf-8"))
    payload["records"][0]["source_paths"]["rgb"] = "SEALED/frame.png"
    with pytest.raises(worker.WorkerError, match="forbidden marker"):
        worker.validate_manifest(payload)


def test_crop_sha_and_manifest_file_sha_fail_closed(holdout_dataset, tmp_path: Path):
    crop_root, manifest = holdout_dataset
    manifest_sha = hashlib.sha256(manifest.read_bytes()).hexdigest()
    with pytest.raises(worker.WorkerError, match="manifest file SHA-256"):
        worker.load_dataset(manifest, "0" * 64, crop_root)

    copied = tmp_path / "copied"
    shutil.copytree(crop_root, copied)
    payload = json.loads(manifest.read_text(encoding="utf-8"))
    first_crop = copied.joinpath(*payload["records"][0]["crop_path"].split("/"))
    first_crop.write_bytes(b"corrupted-crop")
    with pytest.raises(worker.WorkerError, match="crop SHA-256 mismatch"):
        worker.load_dataset(manifest, manifest_sha, copied)


class _FakeSession:
    def __init__(self, probabilities=None, names=None):
        self.probabilities = probabilities or [0.01, 0.01, 0.01, 0.01, 0.9, 0.02, 0.02, 0.02]
        self.names = names or {index: name for index, name in enumerate(worker.CLASS_ORDER)}

    def get_inputs(self):
        return [SimpleNamespace(name="images", shape=[1, 3, 224, 224], type="tensor(float)")]

    def get_outputs(self):
        return [SimpleNamespace(name="output0", shape=[1, 8], type="tensor(float)")]

    def get_modelmeta(self):
        return SimpleNamespace(
            custom_metadata_map={
                "names": repr(self.names),
                "task": "classify",
                "imgsz": "[224, 224]",
            }
        )

    def run(self, output_names, inputs):
        assert output_names == ["output0"]
        assert inputs["images"].shape == (1, 3, 224, 224)
        return [np.asarray([self.probabilities], dtype=np.float32)]


def test_model_metadata_preprocess_and_native_probability_mapping(holdout_dataset):
    crop_root, manifest = holdout_dataset
    payload = json.loads(manifest.read_text(encoding="utf-8"))
    first = payload["records"][0]
    crop_bytes = crop_root.joinpath(*first["crop_path"].split("/")).read_bytes()
    tensor = worker.preprocess_crop(crop_bytes, cv2, np)
    assert tensor.shape == (1, 3, 224, 224)
    assert tensor.dtype == np.float32
    assert 0.0 <= float(tensor.min()) <= float(tensor.max()) <= 1.0

    session = _FakeSession()
    contract = worker.validate_model_session(session)
    assert contract["embedded_class_order_verified"] is True
    item = {
        "record_id": first["record_id"],
        "class_name": first["class_name"],
        "crop_relative": worker.canonical_relative_path(first["crop_path"], "crop"),
        "crop_sha256": first["crop_sha256"],
        "source_identity": first["source_identity"],
        "source_identity_sha256": first["source_identity_sha256"],
        "crop_bytes": crop_bytes,
    }
    rows = worker.run_inference([item], session, cv2, np)
    assert rows[0]["source_class"] == "metal"
    assert rows[0]["predicted_product_class"] == "metal_can"
    assert set(rows[0]["probabilities"]) == set(worker.CLASS_ORDER)

    bad_names = {index: name for index, name in enumerate(reversed(worker.CLASS_ORDER))}
    with pytest.raises(worker.WorkerError, match="embedded class order"):
        worker.validate_model_session(_FakeSession(names=bad_names))


def test_non_probability_output_and_mount_contract_fail_closed(holdout_dataset):
    crop_root, manifest = holdout_dataset
    payload = json.loads(manifest.read_text(encoding="utf-8"))
    first = payload["records"][0]
    item = {
        "record_id": first["record_id"],
        "class_name": first["class_name"],
        "crop_relative": worker.canonical_relative_path(first["crop_path"], "crop"),
        "crop_sha256": first["crop_sha256"],
        "source_identity": first["source_identity"],
        "crop_bytes": crop_root.joinpath(*first["crop_path"].split("/")).read_bytes(),
    }
    with pytest.raises(worker.WorkerError, match="probability distribution"):
        worker.run_inference([item], _FakeSession(probabilities=[1.0] * 8), cv2, np)

    mounts = worker.parse_mounts(
        "/dev/root / ext4 ro,relatime 0 0\n"
        "/dev/input /input ext4 ro,relatime 0 0\n"
        "/dev/output /output ext4 rw,relatime 0 0\n"
    )
    assert "ro" in worker.mount_options_for_resolved(PurePosixPath("/input/data"), mounts)
    assert "rw" in worker.mount_options_for_resolved(PurePosixPath("/output"), mounts)


def test_atomic_output_refuses_reuse(tmp_path: Path):
    output = tmp_path / "result.json"
    worker.write_json_atomic(output, {"selected": False, "frozen": False})
    assert json.loads(output.read_text(encoding="utf-8"))["selected"] is False
    with pytest.raises(worker.WorkerError, match="already exists"):
        worker.write_json_atomic(output, {"selected": True})
