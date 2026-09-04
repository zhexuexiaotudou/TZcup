from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import pytest
import validate_dosod_s100p_hbm_compile_contract as compile_contract


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "collect_formal_s100_calibration_frames.py"
SPEC = importlib.util.spec_from_file_location("s100_calibration_collector", SCRIPT)
assert SPEC and SPEC.loader
SUBJECT = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = SUBJECT
SPEC.loader.exec_module(SUBJECT)


def _contract() -> dict:
    contract = copy.deepcopy(json.loads((ROOT / "config" / "dosod_s100p_hbm_compile_contract.json").read_text(encoding="utf-8")))
    contract["calibration"]["minimum_sample_count"] = 2
    return contract


def _frame(value: int, *, width: int = 3, height: int = 2) -> object:
    pixels = bytes([value, value + 1, value + 2]) * (width * height)
    return SUBJECT.ImageFrame(pixels, width, height, width * 3, "rgb8", 123, "front_camera")


def _authorization(root: Path, topic: str = "/camera/front/image", target_count: int = 500) -> dict:
    contract = json.loads((ROOT / "config" / "dosod_s100p_hbm_compile_contract.json").read_text(encoding="utf-8"))
    return {
        "operator": "fixture-operator",
        "authorized_action": "capture_real_s100p_calibration",
        "timestamp": datetime.now(timezone.utc).isoformat(), "consent": True,
        "privacy": "private calibration only", "private_output_root": str(root.resolve()),
        "retention_policy": "delete after approved calibration review",
        "privacy_policy": "retain private NPY tensors only", "processed_tensor_retention_approved": True,
        "source_kind": "physical_camera", "no_replay": True, "topic": topic,
        "board": "RDK S100P", "scope": "tzcup_s100p_dosod_calibration_capture_v1",
        "contract_id": contract["contract_id"], "target_count": target_count,
        "override_collection_status": "PAUSED_NO_NEW_CAPTURE",
    }


def _freeze(store, root: Path, authorization: dict | None = None) -> Path:
    return store.freeze(
        authorization=authorization or _authorization(root),
        hardware={"attested": True, "architecture": "aarch64", "board": "RDK S100P", "soc": "Journey 6P"},
        publisher_identity=[{"node_name": "camera_driver", "node_namespace": "/", "topic_type": SUBJECT.IMAGE_TYPE}],
        camera_frame_id="front_camera",
    )


def test_normal_frames_are_dosod_nchw_and_freeze_compatible_manifest(tmp_path: Path) -> None:
    store = SUBJECT.CalibrationStore(tmp_path / "calibration", _contract(), "/camera/front/image", set())
    assert store.add(_frame(1)) is True
    assert store.add(_frame(20)) is True
    manifest_path = _freeze(store, tmp_path)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["status"] == "FROZEN"
    assert manifest["authorization_record"]["source_kind"] == "physical_camera"
    assert manifest["camera_frame_id"] == "front_camera"
    assert not (tmp_path / "calibration" / SUBJECT.RECEIPT_NAME).exists()
    assert manifest["records_sha256"] == SUBJECT.canonical_sha256(manifest["records"])
    assert all(row["source_role"] == "calibration_only" for row in manifest["records"])
    blockers: list[str] = []
    compile_contract.audit_calibration(tmp_path / "calibration", _contract(), blockers)
    assert blockers == []
    tensor = np.load(tmp_path / "calibration" / manifest["records"][0]["relative_path"], allow_pickle=False)
    assert tensor.shape == (1, 3, 640, 640)
    assert tensor.dtype == np.float32
    assert np.isfinite(tensor).all() and 0.0 <= float(tensor.min()) <= float(tensor.max()) <= 1.0


def test_duplicate_source_and_duplicate_tensor_are_not_collected(tmp_path: Path) -> None:
    store = SUBJECT.CalibrationStore(tmp_path / "calibration", _contract(), "/camera/front/image", set())
    assert store.add(_frame(1)) is True
    assert store.add(_frame(1)) is False
    # Row padding changes the raw source SHA but not the RGB pixels/tensor.
    padded = bytearray()
    for _ in range(2):
        padded.extend(bytes([1, 2, 3]) * 3)
        padded.extend(b"\x00\x01\x02")
    frame = SUBJECT.ImageFrame(bytes(padded), 3, 2, 12, "rgb8", 124, "front_camera")
    assert store.add(frame) is False


def test_simulator_truth_and_non_image_topics_are_rejected() -> None:
    with pytest.raises(SUBJECT.CalibrationRejected, match="topic_forbidden"):
        SUBJECT.validate_topic("/gazebo/camera/image", [SUBJECT.IMAGE_TYPE])
    with pytest.raises(SUBJECT.CalibrationRejected, match="topic_not_exact"):
        SUBJECT.validate_topic("/camera/front/image", ["sensor_msgs/msg/CompressedImage"])


def test_bad_shape_nonfinite_and_holdout_are_rejected(tmp_path: Path, monkeypatch) -> None:
    with pytest.raises(SUBJECT.CalibrationRejected, match="rgb_image_shape_or_dtype"):
        SUBJECT.preprocess_dosod_rgb(np.zeros((2, 2), dtype=np.uint8))
    original = SUBJECT.np.isfinite
    monkeypatch.setattr(SUBJECT.np, "isfinite", lambda value: np.zeros_like(value, dtype=bool))
    with pytest.raises(SUBJECT.CalibrationRejected, match="preprocessed_tensor_invalid"):
        SUBJECT.preprocess_dosod_rgb(np.zeros((2, 2, 3), dtype=np.uint8))
    monkeypatch.setattr(SUBJECT.np, "isfinite", original)
    held = _frame(1)
    store = SUBJECT.CalibrationStore(tmp_path / "calibration", _contract(), "/camera/front/image", {hashlib.sha256(held.data).hexdigest()})
    with pytest.raises(SUBJECT.CalibrationRejected, match="evaluation_holdout_overlap"):
        store.add(held)


def test_partial_or_nonempty_output_is_rejected(tmp_path: Path) -> None:
    output = tmp_path / "partial"
    output.mkdir()
    (output / "samples").mkdir()
    with pytest.raises(SUBJECT.CalibrationRejected, match="output_must_be_empty"):
        SUBJECT.CalibrationStore(output, _contract(), "/camera/front/image", set())


def test_paused_contract_needs_explicit_capture_flag_and_structured_authorization(tmp_path: Path) -> None:
    with pytest.raises(SUBJECT.CalibrationRejected, match="collection_paused"):
        SUBJECT.load_contract(ROOT / "config" / "dosod_s100p_hbm_compile_contract.json")
    record_path = tmp_path / "authorization.json"
    record_path.write_text(json.dumps(_authorization(tmp_path)), encoding="utf-8")
    with pytest.raises(SUBJECT.CalibrationRejected, match="operator_authorization_flag_required"):
        SUBJECT.authorization_record(record_path, accepted=False, output=tmp_path / "out", topic="/camera/front/image", contract=json.loads((ROOT / "config" / "dosod_s100p_hbm_compile_contract.json").read_text(encoding="utf-8")), target_count=500)
    accepted = SUBJECT.authorization_record(record_path, accepted=True, output=tmp_path / "out", topic="/camera/front/image", contract=json.loads((ROOT / "config" / "dosod_s100p_hbm_compile_contract.json").read_text(encoding="utf-8")), target_count=500)
    assert accepted["board"] == "RDK S100P"


def test_fake_board_and_replay_publisher_are_rejected() -> None:
    with pytest.raises(SUBJECT.CalibrationRejected, match="hardware_identity"):
        SUBJECT.require_s100p_hardware(lambda: {"attested": False})
    with pytest.raises(SUBJECT.CalibrationRejected, match="publisher_replay"):
        SUBJECT.validate_publishers([{"node_name": "rosbag2_replay", "node_namespace": "/", "topic_type": SUBJECT.IMAGE_TYPE}])


def test_timeout_partial_receipt_is_terminal_and_not_reusable(tmp_path: Path) -> None:
    output = tmp_path / "partial"
    output.mkdir()
    (output / "samples").mkdir()
    SUBJECT.write_receipt(output, {"status": "BLOCKED", "partial_data_not_reusable": True}, allow_partial=True)
    receipt = json.loads((output / SUBJECT.RECEIPT_NAME).read_text(encoding="utf-8"))
    assert receipt["status"] == "BLOCKED" and receipt["partial_data_not_reusable"] is True
    with pytest.raises(SUBJECT.CalibrationRejected, match="output_must_be_empty"):
        SUBJECT.reject_unsafe_output(output)


def test_collect_uses_injected_board_gate_and_writes_blocked_receipt(tmp_path: Path) -> None:
    output = tmp_path / "private" / "capture"
    record_path = tmp_path / "authorization.json"
    record_path.write_text(json.dumps(_authorization(tmp_path / "private")), encoding="utf-8")
    with pytest.raises(SUBJECT.CalibrationRejected, match="hardware_identity"):
        SUBJECT.collect(
            output=output, topic="/camera/front/image", target_count=500,
            contract_path=ROOT / "config" / "dosod_s100p_hbm_compile_contract.json",
            holdout_paths=[], holdout_values=[], timeout_s=0.0,
            accept_operator_authorized_real_capture=True, authorization_path=record_path,
            hardware_probe=lambda: {"attested": False},
        )
    receipt = json.loads((output / SUBJECT.RECEIPT_NAME).read_text(encoding="utf-8"))
    assert receipt["status"] == "BLOCKED" and receipt["hardware"] is None


@pytest.mark.parametrize(
    ("field", "value", "error"),
    [
        ("scope", "other", "authorization_scope_invalid"),
        ("override_collection_status", "READY", "authorization_collection_status_override_invalid"),
        ("target_count", 501, "authorization_target_count_mismatch"),
        ("contract_id", "other-contract", "authorization_contract_id_mismatch"),
        ("timestamp", "2026-09-03T00:00:00+00:00", "authorization_timestamp_outside_window"),
        ("privacy_policy", "no raw pixels retained", "authorization_privacy_policy"),
        ("processed_tensor_retention_approved", False, "authorization_consent_or_privacy_missing"),
    ],
)
def test_authorization_schema_conflicts_fail_closed(tmp_path: Path, field: str, value: object, error: str) -> None:
    contract = json.loads((ROOT / "config" / "dosod_s100p_hbm_compile_contract.json").read_text(encoding="utf-8"))
    record = _authorization(tmp_path)
    record["timestamp"] = "2026-09-05T00:00:00+00:00"
    record[field] = value
    path = tmp_path / "authorization.json"
    path.write_text(json.dumps(record), encoding="utf-8")
    with pytest.raises(SUBJECT.CalibrationRejected, match=error):
        SUBJECT.authorization_record(
            path, accepted=True, output=tmp_path / "output", topic="/camera/front/image",
            contract=contract, target_count=500,
            now=datetime(2026, 9, 5, 6, tzinfo=timezone.utc),
        )


def test_manifest_commit_window_never_creates_blocked_second_terminal(tmp_path: Path, monkeypatch) -> None:
    output = tmp_path / "committed"
    output.mkdir()
    (output / "calibration_manifest.json").write_text("{}\n", encoding="utf-8")
    monkeypatch.setattr(SUBJECT, "write_receipt", lambda *_args, **_kwargs: pytest.fail("receipt after manifest"))
    SUBJECT.write_blocked_receipt_if_manifest_absent(
        output, {"calibration": {"manifest_name": "calibration_manifest.json"}},
        {"status": "BLOCKED"}, allow_partial=True,
    )
    assert not (output / SUBJECT.RECEIPT_NAME).exists()
