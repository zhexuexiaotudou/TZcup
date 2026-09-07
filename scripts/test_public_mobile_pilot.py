import json
import os
import subprocess
import tempfile
from pathlib import Path
import pytest


ROOT = Path(__file__).resolve().parents[1]


def test_pilot_and_full_outputs_are_explicitly_separate() -> None:
    collector = (ROOT / "scripts/public_gazebo_dosod_calibration.py").read_text(encoding="utf-8")
    runner = (ROOT / "scripts/run_public_mobile_gazebo_dosod_calibration.sh").read_text(encoding="utf-8")
    assert '"pilot_manifest.json"' in collector
    assert 'self.contract["calibration"]["manifest_name"]' in collector
    assert 'EXPECTED_MANIFEST="pilot_manifest.json"' in runner
    assert 'EXPECTED_MANIFEST="calibration_manifest.json"' in runner
    assert "NON_FORMAL_PILOT_CAPTURED" in collector and "NON_FORMAL_PILOT_CAPTURED" in runner


def test_full_mode_requires_hash_bound_review_fields() -> None:
    collector = (ROOT / "scripts/public_gazebo_dosod_calibration.py").read_text(encoding="utf-8")
    for field in ("pilot_manifest_sha256", "contact_sheet_sha256", "record_sha256", '"reviews"'):
        assert field in collector
    runner = (ROOT / "scripts/run_public_mobile_gazebo_dosod_calibration.sh").read_text(encoding="utf-8")
    assert runner.index('preflight_args+=(--review-receipt') < runner.index('ros2 launch sanitation_formal_campus_integration')
    assert '--validate-pilot-manifest "$DATASET/pilot_manifest.json"' in runner
    assert 'VALIDATION_SNAPSHOT="$RUN_ROOT/full_review_validation.json"' in runner
    assert "review_validation_snapshot_sha256" in runner


def test_review_and_provenance_path_type_gates_are_fail_closed() -> None:
    collector = (ROOT / "scripts/public_gazebo_dosod_calibration.py").read_text(encoding="utf-8")
    assert 'any(_unsafe(path) or not path.is_file() for path in (receipt, pilot_manifest))' in collector
    assert 'isinstance(stamp, bool) or stamp <= 0' in collector


def test_runner_receipt_binds_pilot_artifacts_and_blocks_drift() -> None:
    source = (ROOT / "scripts/run_public_mobile_gazebo_dosod_calibration.sh").read_text(encoding="utf-8")
    start = source.index("write_receipt() {")
    write_receipt = source[start:source.index("\nstop_verified()", start)]
    write_receipt = "export PUBLIC_GAZEBO_CALIBRATION_TIMEOUT_SEC=900\n" + write_receipt
    work = ROOT / ".work"; work.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="pilot-receipt-", dir=work) as raw:
        root = Path(raw); dataset = root / "dataset"; dataset.mkdir(); sheet = dataset / "pilot_contact_sheet.png"
        from PIL import Image
        Image.new("RGB", (800, 800)).save(sheet)
        import hashlib
        sheet_hash = hashlib.sha256(sheet.read_bytes()).hexdigest()
        manifest = {"status":"NON_FORMAL_PILOT_CAPTURED","formal_passed":False,"pilot_scene":"map-0-mission-0","record_count":25,"record_sha256":"a"*64,"contact_sheet":{"relative_path":sheet.name,"sha256":sheet_hash,"byte_size":sheet.stat().st_size}}
        (dataset / "pilot_manifest.json").write_text(json.dumps(manifest))
        fixture = root / "fixture.sh"
        fixture.write_bytes(("#!/usr/bin/env bash\nset -u\ncd \"$(dirname \"${BASH_SOURCE[0]}\")\"\nRECEIPT=receipt.json\nMODE=pilot\nDATASET=dataset\nEXPECTED_MANIFEST=pilot_manifest.json\nPUBLIC_GAZEBO_CALIBRATION_REVIEW_RECEIPT=''\nPUBLIC_GAZEBO_CALIBRATION_PILOT_MANIFEST=''\nPUBLIC_GAZEBO_CALIBRATION_TIMEOUT_SEC=900\nPRIMARY_ERROR=''\n" + write_receipt + "\nset +e\nwrite_receipt NON_FORMAL_PILOT_CAPTURED 0 true\necho $?\n").encode("utf-8"))
        result = subprocess.run(["bash", fixture.relative_to(ROOT).as_posix()], cwd=ROOT, env={**os.environ, "PUBLIC_GAZEBO_CALIBRATION_TIMEOUT_SEC":"900"}, text=True, capture_output=True, check=True, timeout=20)
        assert result.stdout.strip() == "0", result.stderr
        receipt = json.loads((root / "receipt.json").read_text()); assert receipt["status"] == "NON_FORMAL_PILOT_CAPTURED" and receipt["artifact"]["contact_sheet"]["sha256"] == sheet_hash
        sheet.unlink()
        result = subprocess.run(["bash", fixture.relative_to(ROOT).as_posix()], cwd=ROOT, text=True, capture_output=True, check=True, timeout=20)
        assert result.stdout.strip() == "1"
        assert json.loads((root / "receipt.json").read_text())["status"] == "BLOCKED"


@pytest.mark.parametrize("mutation", ("review", "pilot", "contact_sheet"))
def test_runner_full_receipt_blocks_snapshot_bound_mutation(mutation: str) -> None:
    source = (ROOT / "scripts/run_public_mobile_gazebo_dosod_calibration.sh").read_text(encoding="utf-8")
    start = source.index("write_receipt() {"); write_receipt = source[start:source.index("\nstop_verified()", start)]
    write_receipt = "export PUBLIC_GAZEBO_CALIBRATION_TIMEOUT_SEC=14400\n" + write_receipt
    work = ROOT / ".work"; work.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=f"full-receipt-{mutation}-", dir=work) as raw:
        root = Path(raw); dataset = root / "dataset"; dataset.mkdir(); (dataset / "calibration_manifest.json").write_text('{"status":"FROZEN"}')
        from PIL import Image
        import hashlib
        sheet = root / "pilot_contact_sheet.png"; Image.new("RGB", (800, 800)).save(sheet); sheet_hash = hashlib.sha256(sheet.read_bytes()).hexdigest()
        pilot = root / "pilot.json"; pilot.write_text(json.dumps({"record_sha256":"a"*64,"contact_sheet":{"relative_path":sheet.name,"sha256":sheet_hash}}))
        review = root / "review.json"; review.write_text('{"approved":true}')
        digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
        snapshot = root / "snapshot.json"; snapshot.write_text(json.dumps({"status":"NON_FORMAL_REVIEW_APPROVED","review_receipt_sha256":digest(review),"pilot_manifest_sha256":digest(pilot),"contact_sheet_sha256":sheet_hash,"record_sha256":"a"*64}))
        fixture = root / "fixture.sh"
        fixture.write_bytes(("#!/usr/bin/env bash\nset -u\ncd \"$(dirname \"${BASH_SOURCE[0]}\")\"\nRECEIPT=receipt.json\nMODE=full\nDATASET=dataset\nEXPECTED_MANIFEST=calibration_manifest.json\nPUBLIC_GAZEBO_CALIBRATION_REVIEW_RECEIPT=review.json\nPUBLIC_GAZEBO_CALIBRATION_PILOT_MANIFEST=pilot.json\nPUBLIC_GAZEBO_CALIBRATION_TIMEOUT_SEC=14400\nVALIDATION_SNAPSHOT=snapshot.json\nPRIMARY_ERROR=''\n" + write_receipt + "\nset +e\nwrite_receipt NON_FORMAL_CALIBRATION_FROZEN 0 true\necho $?\n").encode("utf-8"))
        run = lambda: subprocess.run(["bash", fixture.relative_to(ROOT).as_posix()], cwd=ROOT, env={**os.environ, "PUBLIC_GAZEBO_CALIBRATION_TIMEOUT_SEC":"14400"}, text=True, capture_output=True, check=True, timeout=20)
        assert run().stdout.strip() == "0"
        receipt = json.loads((root / "receipt.json").read_text()); assert receipt["artifact"]["contact_sheet_sha256"] == sheet_hash and receipt["artifact"]["record_sha256"] == "a"*64
        if mutation == "review": review.write_text('{"approved":false}')
        elif mutation == "pilot": pilot.write_text('{"record_sha256":"b"}')
        else: Image.new("RGB", (800, 800), "red").save(sheet)
        assert run().stdout.strip() == "1"
        blocked = json.loads((root / "receipt.json").read_text()); assert blocked["status"] == "BLOCKED" and blocked["exit_code"] == 125
