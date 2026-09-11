import json
import os
import sys
from pathlib import Path

import pytest

import a20_release_replay_receipt as receipt_module
import formal_product_mcap_replay as replay_module
from a20_release_replay_receipt import (
    BLOCKER,
    _assert_root_binding,
    _open_bound_input,
    _open_root_directory,
    _output_in_root,
    _require_posix_descriptor_api,
    _read_bound_json,
    _regular_in_root,
    _write_fresh_output,
    validate_receipt,
)


ROOT = Path(__file__).resolve().parents[1]
CONTRACT = ROOT / "config/high_fidelity_vehicle/a20_release_replay_receipt_contract.json"


def test_a20_contract_names_canonical_formal_product_replay_producer() -> None:
    contract = json.loads(CONTRACT.read_text(encoding="utf-8"))
    report = validate_receipt({"handwritten": "PASS"})

    assert contract["canonical_product_replay_producer"] == "scripts/formal_product_mcap_replay.py"
    assert report["valid"] is False
    assert report["status"] == "A20_RECEIPT_STATIC_BLOCKED"
    assert "repository root is required" in report["errors"][0]
    assert report["release_runtime_pass"] is False


def test_a20_accepts_five_distinct_current_canonical_replays_and_real_release_files(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(replay_module, "mcap_semantic_sha256", lambda bag: __import__("hashlib").sha256(next(Path(bag).glob("*.mcap")).read_bytes()).hexdigest())
    monkeypatch.setattr(receipt_module, "_verify_complete_session_evidence", lambda *args, **kwargs: None)
    root = tmp_path.resolve()
    producer = root / "scripts/formal_product_mcap_replay.py"
    producer.parent.mkdir()
    producer.write_text("# canonical producer\n", encoding="utf-8")
    producer_hash = __import__("hashlib").sha256(producer.read_bytes()).hexdigest()
    source_hash = "a" * 64
    hashes = {"source": source_hash, "model": "b" * 64, "config": "c" * 64, "dataset": "d" * 64, "dependency": "e" * 64}
    snapshot_file = root / "snapshot.json"
    snapshot_file.write_text('{"snapshot":"fixture"}', encoding="utf-8")
    snapshot_file_hash = __import__("hashlib").sha256(snapshot_file.read_bytes()).hexdigest()
    snapshot = {"snapshot_manifest_sha256": snapshot_file_hash, "source_inventory_sha256": source_hash, "expanded_urdf_sha256": "2" * 64}
    ros2_identity = {"path": str(producer), "sha256": producer_hash}
    closure = {"status": "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED", "closure_sha256": "3" * 64, "ros2_executable": ros2_identity}
    session = root / "session.json"
    session.write_text(json.dumps({
        "report_id": "tzcup_formal_final_acceptance_session_v1",
        "status": "FORMAL_FINAL_ACCEPTANCE_SESSION_COMPLETE",
        "started_epoch_ns": 123,
        "finished_epoch_ns": 124,
        "failures": {},
        "evidence": {f"gate-{index}": {"sha256": "a" * 64, "status": "PASS"} for index in range(25)},
        "snapshot": snapshot,
        "runtime_closure_binding": closure,
    }), encoding="utf-8")
    runtime_binding = root / "runtime-binding.json"
    runtime_binding.write_text(json.dumps({
        "status": "FORMAL_RUNTIME_GATE_BOUND",
        "runtime_closure_binding": closure,
        "acceptance_session_binding": {"snapshot": snapshot, "session_started_epoch_ns": 123},
    }), encoding="utf-8")

    def ref(path: Path) -> dict[str, str]:
        return {"path": str(path), "sha256": __import__("hashlib").sha256(path.read_bytes()).hexdigest()}

    replay_refs = []
    raw_results = {}
    provenance_refs = {}
    for name in ("model", "config", "dataset", "dependency"):
        path = root / f"provenance-{name}"
        path.write_text(name, encoding="utf-8")
        hashes[name] = __import__("hashlib").sha256(path.read_bytes()).hexdigest()
        provenance_refs[name] = {"path": str(path), "sha256": hashes[name]}
    for index in range(5):
        bag = root / f"bag-{index}"
        bag.mkdir()
        (bag / "capture.mcap").write_bytes(f"bag-{index}".encode())
        (bag / "metadata.yaml").write_text("""rosbag2_bagfile_information:
  storage_identifier: mcap
  relative_file_paths: [capture.mcap]
  files: [{path: capture.mcap}]
  topics_with_message_count: []
""", encoding="utf-8")
        from formal_product_mcap_replay import mcap_sha256, mcap_semantic_sha256
        raw_path = root / f"raw-{index}.json"
        raw_path.write_text(json.dumps({
            "schema": "tzcup.a12.raw_capture_receipt.v1",
            "capture_id": f"{index + 6000:064x}",
            "run_root": str(root),
            "mcap": {"path": str(bag), "sha256": mcap_sha256(bag)},
        }), encoding="utf-8")
        raw_ref = ref(raw_path)
        raw_result = {**raw_ref, "capture_id": f"{index + 6000:064x}", "run_root": str(root), "scenario_id": f"scenario-{index}", "seed": index, "mission_id": f"mission-{index}", "mcap": {"path": str(bag), "sha256": mcap_sha256(bag), "semantic_sha256": mcap_semantic_sha256(bag)}}
        raw_results[str(raw_path)] = raw_result
        replay = root / f"replay-{index}.json"
        replay.write_text(json.dumps({
            "schema": "tzcup.formal_product_mcap_replay.v1",
            "status": "FORMAL_PRODUCT_MCAP_REPLAY_PASS",
            "pass": True,
            "producer": {"id": "scripts/formal_product_mcap_replay.py", "sha256": producer_hash},
                "formal_context": {
                    "session": {"path": str(session), "started_epoch_ns": 123},
                    "snapshot": snapshot,
                    "snapshot_manifest": ref(snapshot_file),
                    "runtime_closure_binding": closure,
                    "runtime_gate_binding": ref(runtime_binding),
                },
            "input_hashes": {"model": hashes["model"], "config": hashes["config"], "dataset": hashes["dataset"], "dependency": hashes["dependency"], "container": "f" * 64},
            "input_artifacts": provenance_refs,
            "checks": {name: True for name in receipt_module.REPLAY_CHECKS},
            "playback": {"exit_code": 0, "timed_out": False, "ros_domain_id": 215, "ros_localhost_only": True, "rmw_implementation": "rmw_cyclonedds_cpp", "automatic_discovery_range": "LOCALHOST", "ros2_executable": ros2_identity, "process_group_cleanup": {"process_group_id": index + 100, "process_group_isolated": True, "cleanup_attempted": True, "sigterm_attempted": False, "sigkill_attempted": False, "signals_sent": [], "surviving_group_processes": 0, "zero_survivor": True}},
            "bag": {"path": str(bag), "sha256": mcap_sha256(bag), "semantic_sha256": mcap_semantic_sha256(bag)},
            "run_root": str(root),
            "raw_capture": raw_result,
        }), encoding="utf-8")
        replay_refs.append(ref(replay))
    monkeypatch.setattr(receipt_module, "validate_raw_capture_receipt", lambda **kwargs: raw_results[str(kwargs["raw_capture_receipt_path"])])
    execution_refs = []
    for index, replay_ref in enumerate(replay_refs):
        replay_payload = json.loads(Path(replay_ref["path"]).read_text(encoding="utf-8"))
        execution = root / f"execution-{index}.json"
        execution.write_text(json.dumps({"replay": replay_ref, "formal_context": replay_payload["formal_context"]}), encoding="utf-8")
        execution_refs.append(ref(execution))
    ledger = root / "a12-ledger.json"
    ledger.write_text(json.dumps({"run_root": str(root), "formal_context": json.loads(Path(replay_refs[0]["path"]).read_text(encoding="utf-8"))["formal_context"], "executions": execution_refs}), encoding="utf-8")
    monkeypatch.setattr(receipt_module, "validate_auto15_execution_evidence", lambda *args, **kwargs: {"execution_evidence_pass": True})
    release_refs = {}
    for name in ("archive", "sha256sums", "sbom", "dependency_lock", "licenses"):
        path = root / name
        path.write_text(name, encoding="utf-8")
        release_refs[name] = ref(path)
    rollback_report = root / "rollback.json"
    rollback_report.write_text('{"verified":true}', encoding="utf-8")
    receipt = {
        "schema": "tzcup.a20_release_replay_receipt.v1",
        "a12_ledger": ref(ledger),
        "input_hashes": hashes,
        "sealed_final_session": ref(session),
        "product_replays": replay_refs,
        "container_sha256": "f" * 64,
        "release_artifact": {
            "status": "RELEASE_PACKAGE_ARTIFACT_RECORDED",
            "main_commit": "1" * 40,
            "rollback_commit": "2" * 40,
            "container_sha256": "f" * 64,
            **release_refs,
        },
        "verified_rollback_exercise": {
            "status": "ROLLBACK_EXERCISE_VERIFIED",
            "verified": True,
            "rollback_commit": "2" * 40,
            "verification_report": ref(rollback_report),
        },
    }
    report = validate_receipt(receipt, root, root, session)
    assert report["valid"] is True, report
    assert report["release_runtime_pass"] is True
    wrong_session = root / "wrong-session.json"
    wrong_session.write_text(session.read_text(encoding="utf-8"), encoding="utf-8")
    mismatch = validate_receipt(receipt, root, root, wrong_session)
    assert mismatch["valid"] is False
    assert "session path" in mismatch["errors"][0]
    foreign_root = root / "foreign-run"
    foreign_root.mkdir()
    cross_root = validate_receipt(receipt, root, foreign_root, session)
    assert cross_root["valid"] is False
    assert "replay" in cross_root["errors"][0] or "run root" in cross_root["errors"][0]
    # A20 reopens the retained raw receipt rather than trusting the replay's
    # embedded copy; a swapped capture chain must invalidate release intake.
    first_raw = root / "raw-0.json"
    first_raw.write_text('{"schema":"forged"}', encoding="utf-8")
    blocked = validate_receipt(receipt, root, root, session)
    assert blocked["valid"] is False
    assert "raw capture receipt" in blocked["errors"][0]


def test_cli_paths_reject_escape_symlink_and_existing_output(tmp_path: Path) -> None:
    root = tmp_path / "root"
    root.mkdir()
    receipt = root / "receipt.json"
    receipt.write_text("{}", encoding="utf-8")
    assert _regular_in_root(root, receipt, "receipt") == receipt

    with pytest.raises(ValueError, match="escapes"):
        _regular_in_root(root, tmp_path / "outside.json", "receipt")
    with pytest.raises(ValueError, match="escapes"):
        _regular_in_root(root, root / ".." / "outside.json", "receipt")
    with pytest.raises(ValueError, match="fresh"):
        _output_in_root(root, receipt)
    try:
        link = root / "link.json"
        link.symlink_to(receipt)
    except OSError as exc:  # pragma: no cover - Windows without symlink privilege
        pytest.skip(f"symbolic links unavailable: {exc}")
    with pytest.raises(ValueError, match="symbolic-link"):
        _regular_in_root(root, link, "receipt")
    directory = root / "real"
    directory.mkdir()
    (directory / "nested.json").write_text("{}", encoding="utf-8")
    linked_directory = root / "linked"
    linked_directory.symlink_to(directory, target_is_directory=True)
    with pytest.raises(ValueError, match="symbolic-link ancestor"):
        _regular_in_root(root, linked_directory / "nested.json", "receipt")


def test_secure_output_rejects_pending_symlink_and_commit_race(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    root = tmp_path / "root"
    root.mkdir()
    output = root / "result.json"
    outside = tmp_path / "outside.json"
    pending = root / ".result.json.pending.fixed"
    try:
        pending.symlink_to(outside)
    except OSError as exc:  # pragma: no cover - Windows without symlink privilege
        pytest.skip(f"symbolic links unavailable: {exc}")
    root_descriptor = _open_root_directory(root)
    try:
        with pytest.raises(FileExistsError):
            _write_fresh_output(root, root_descriptor, output, {"blocked": True}, token="fixed")
    finally:
        os.close(root_descriptor)
    assert not outside.exists()
    assert pending.is_symlink()
    pending.unlink()

    real_link = os.link

    def create_target_then_link(*args, **kwargs):
        output.write_text("attacker", encoding="utf-8")
        return real_link(*args, **kwargs)

    monkeypatch.setattr(receipt_module.os, "link", create_target_then_link)
    root_descriptor = _open_root_directory(root)
    try:
        with pytest.raises(ValueError, match="appeared during commit"):
            _write_fresh_output(root, root_descriptor, output, {"blocked": True}, token="race")
    finally:
        os.close(root_descriptor)
    assert output.read_text(encoding="utf-8") == "attacker"


@pytest.mark.skipif(os.name == "nt", reason="secure dir_fd traversal is POSIX-only")
def test_secure_output_reopens_bound_root_after_parent_swap(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    root = tmp_path / "root"
    parent = root / "evidence"
    parent.mkdir(parents=True)
    output = parent / "result.json"
    retained = root / "retained-evidence"
    real_link = os.link

    def link_then_swap_parent(*args, **kwargs):
        result = real_link(*args, **kwargs)
        parent.rename(retained)
        parent.mkdir()
        output.write_text("attacker", encoding="utf-8")
        return result

    monkeypatch.setattr(receipt_module.os, "link", link_then_swap_parent)
    root_descriptor = _open_root_directory(root)
    try:
        with pytest.raises(ValueError, match="identity mismatch"):
            _write_fresh_output(root, root_descriptor, output, {"blocked": True})
    finally:
        os.close(root_descriptor)
    assert output.read_text(encoding="utf-8") == "attacker"
    assert (retained / "result.json").is_file()


def test_windows_cli_is_explicitly_blocked_before_path_processing(tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]) -> None:
    if os.name != "nt":
        monkeypatch.setattr(receipt_module.os, "name", "nt")
        with pytest.raises(ValueError, match="POSIX/WSL-only"):
            _require_posix_descriptor_api()
        return

    root = tmp_path / "root"
    root.mkdir()
    monkeypatch.setattr(sys, "argv", [
        "a20_release_replay_receipt.py",
        "--repository-root", str(root),
        "--run-root", str(root),
        "--session", str(root / "session.json"),
        "--receipt", str(root / "receipt.json"),
        "--output", str(root / "report.json"),
    ])

    assert receipt_module.main() == 2
    report = json.loads(capsys.readouterr().out)
    assert report["status"] == "A20_RECEIPT_STATIC_BLOCKED"
    assert "POSIX/WSL-only" in report["error"]


@pytest.mark.skipif(os.name == "nt", reason="Windows is rejected before POSIX capability checks")
def test_cli_fails_closed_without_no_follow_or_link_dirfd(tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]) -> None:
    root = tmp_path / "root"
    root.mkdir()
    monkeypatch.setattr(sys, "argv", [
        "a20_release_replay_receipt.py",
        "--repository-root", str(root),
        "--run-root", str(root),
        "--session", str(root / "session.json"),
        "--receipt", str(root / "receipt.json"),
        "--output", str(root / "report.json"),
    ])
    monkeypatch.delattr(receipt_module.os, "O_NOFOLLOW")

    assert receipt_module.main() == 2
    report = json.loads(capsys.readouterr().out)
    assert report["status"] == "A20_RECEIPT_STATIC_BLOCKED"
    assert "requires O_NOFOLLOW" in report["error"]

    monkeypatch.undo()
    monkeypatch.setattr(
        receipt_module.os,
        "supports_dir_fd",
        frozenset(operation for operation in os.supports_dir_fd if operation is not os.link),
    )
    with pytest.raises(ValueError, match="os.link dir_fd"):
        _require_posix_descriptor_api()


@pytest.mark.skipif(os.name == "nt", reason="secure dir_fd traversal is POSIX-only")
def test_bound_input_rejects_mutation_after_read(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    receipt = tmp_path / "receipt.json"
    receipt.write_text('{"receipt": "before"}', encoding="utf-8")
    root_descriptor = _open_root_directory(tmp_path)
    try:
        descriptor, identity = _open_bound_input(tmp_path, root_descriptor, receipt)
    finally:
        os.close(root_descriptor)
    actual_fstat = os.fstat

    def mutate_before_fstat(fd: int):
        receipt.write_text('{"receipt": "after after after"}', encoding="utf-8")
        return actual_fstat(fd)

    monkeypatch.setattr(receipt_module.os, "fstat", mutate_before_fstat)
    with pytest.raises(ValueError, match="changed while being read"):
        _read_bound_json(descriptor, identity)


@pytest.mark.skipif(os.name == "nt", reason="secure dir_fd traversal is POSIX-only")
def test_root_replacement_after_fd_open_is_rejected(tmp_path: Path) -> None:
    root = tmp_path / "root"
    root.mkdir()
    descriptor = _open_root_directory(root)
    retained = tmp_path / "retained"
    root.rename(retained)
    outside = tmp_path / "outside"
    outside.mkdir()
    root.symlink_to(outside, target_is_directory=True)
    try:
        with pytest.raises(ValueError, match="root changed"):
            _assert_root_binding(root, descriptor)
    finally:
        os.close(descriptor)
