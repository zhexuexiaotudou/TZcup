from __future__ import annotations

import hashlib
import importlib.util
import json
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "materialize_s100p_calibration_corpus.py"
SPEC = importlib.util.spec_from_file_location("materialize_s100p_calibration_corpus", SCRIPT)
assert SPEC and SPEC.loader
SUBJECT = importlib.util.module_from_spec(SPEC); sys.modules[SPEC.name] = SUBJECT; SPEC.loader.exec_module(SUBJECT)


def _sha(path: Path) -> str: return hashlib.sha256(path.read_bytes()).hexdigest()


def _capture(root: Path, values: list[int]) -> Path:
    frames = root / "frames"; frames.mkdir(parents=True)
    for index, value in enumerate(values):
        frame = frames / f"frame-{index:04d}"; frame.mkdir()
        with (frame / "arrays.npz").open("wb") as stream:
            np.savez_compressed(stream, rgb=np.full((3, 4, 3), value, dtype=np.uint8))
        metadata = {"schema_version": 1, "sensor": "front", "rgb_stamp_s": float(index + 1),
                    "camera_info": {"frame_id": "front_camera"},
                    "detections": [{"class_id": "puddle"}]}
        (frame / "metadata.json").write_text(json.dumps(metadata), encoding="utf-8")
        manifest = {"schema_version": 1, "files": {"arrays.npz": _sha(frame / "arrays.npz"), "metadata.json": _sha(frame / "metadata.json")}}
        (frame / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    return root


def test_real_capture_shape_smoke_is_resumable_but_not_formal_without_oracle(tmp_path: Path) -> None:
    capture = _capture(tmp_path / "actual_product_capture", [3, 70])
    receipt_path = SUBJECT.materialize(capture_root=capture, output=tmp_path / "out", scenario_id="yard-a")
    receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    assert receipt["status"] == "BLOCKED" and receipt["record_count"] == 2 and receipt["remaining_to_500"] == 498
    assert "official_preprocessing_oracle_not_verified" in receipt["blockers"]
    state = json.loads((tmp_path / "out" / SUBJECT.STATE_NAME).read_text(encoding="utf-8"))
    assert len(state["records"]) == 2
    tensor = np.load(tmp_path / "out" / state["records"][0]["relative_path"], allow_pickle=False)
    assert tensor.shape == (1, 3, 640, 640) and tensor.dtype == np.float32
    assert SUBJECT.materialize(capture_root=capture, output=tmp_path / "out", scenario_id="yard-a") == receipt_path
    assert len(json.loads((tmp_path / "out" / SUBJECT.STATE_NAME).read_text(encoding="utf-8"))["records"]) == 2


def test_duplicate_source_or_tensor_never_fills_count(tmp_path: Path) -> None:
    capture = _capture(tmp_path / "capture", [7, 7])
    receipt = json.loads(SUBJECT.materialize(capture_root=capture, output=tmp_path / "out", scenario_id="yard-a").read_text(encoding="utf-8"))
    assert receipt["record_count"] == 1


def test_tampered_product_capture_and_resume_configuration_are_rejected(tmp_path: Path) -> None:
    capture = _capture(tmp_path / "capture", [7])
    (capture / "frames" / "frame-0000" / "metadata.json").write_text("{}", encoding="utf-8")
    with pytest.raises(SUBJECT.CalibrationRejected, match="hash_mismatch"):
        SUBJECT.materialize(capture_root=capture, output=tmp_path / "out", scenario_id="yard-a")
    capture = _capture(tmp_path / "good", [8])
    SUBJECT.materialize(capture_root=capture, output=tmp_path / "out2", scenario_id="yard-a")
    with pytest.raises(SUBJECT.CalibrationRejected, match="resume_configuration"):
        SUBJECT.materialize(capture_root=capture, output=tmp_path / "out2", scenario_id="yard-b")
    state = json.loads((tmp_path / "out2" / SUBJECT.STATE_NAME).read_text(encoding="utf-8"))
    (tmp_path / "out2" / state["records"][0]["relative_path"]).write_bytes(b"tampered")
    with pytest.raises(SUBJECT.CalibrationRejected, match="resume_sample_drift"):
        SUBJECT.materialize(capture_root=capture, output=tmp_path / "out2", scenario_id="yard-a")


def test_holdout_overlap_fails_closed(tmp_path: Path) -> None:
    capture = _capture(tmp_path / "capture", [9])
    rgb = np.full((3, 4, 3), 9, dtype=np.uint8)
    source_sha = hashlib.sha256(rgb.tobytes(order="C")).hexdigest()
    with pytest.raises(SUBJECT.CalibrationRejected, match="holdout_overlap"):
        SUBJECT.materialize(capture_root=capture, output=tmp_path / "out", scenario_id="yard-a", holdout_source_hashes={source_sha})
