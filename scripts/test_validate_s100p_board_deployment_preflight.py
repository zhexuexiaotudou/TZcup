import hashlib
import importlib.util
import json
import stat
from pathlib import Path

import pytest

PATH = Path(__file__).with_name("validate_s100p_board_deployment_preflight.py")
SPEC = importlib.util.spec_from_file_location("s100_preflight", PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class Stat:
    def __init__(self, mode, dev=1):
        self.st_mode, self.st_dev = mode, dev


def _digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _entry(root, relative):
    path = root / relative
    return {"relative_path": relative, "sha256": _digest(path), "byte_size": path.stat().st_size}


def _write(root, relative, content):
    path = root / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(content if isinstance(content, bytes) else content.encode())
    return path


def _refresh_manifest_entry(root, role):
    manifest_path = root / "handoff/manifest.json"
    manifest = json.loads(manifest_path.read_text())
    relative = manifest["entries"][role]["relative_path"]
    manifest["entries"][role] = _entry(root, relative)
    manifest_path.write_text(json.dumps(manifest, sort_keys=True))


def _fixture(tmp_path):
    for relative, content in {
        "proc/device-tree/model": "D-Robotics RDK S100P V1P0",
        "proc/device-tree/compatible": "drobot,s100-rdk",
        "proc/modules": "bpu_cores 1 0\nbpu_framework 1 0\n",
        "opt/tros/humble/setup.bash": "# setup\n",
    }.items():
        _write(tmp_path, relative, content)
    (tmp_path / "opt/tzcup/s100p").mkdir(parents=True)
    (tmp_path / "opt/tzcup/stages").mkdir(parents=True)
    (tmp_path / "opt/tzcup/rollback").mkdir(parents=True)
    closure_path = _write(tmp_path, "handoff/closure.json", json.dumps({
        "kind": "tzcup_formal_final_runtime_closure", "status": "FORMAL_FINAL_RUNTIME_CLOSURE_FROZEN",
        "closure_sha256": "c" * 64,
    }))
    closure_binding = {"runtime_closure_manifest_sha256": _digest(closure_path), "runtime_closure_sha256": "c" * 64}
    session_path = _write(tmp_path, "handoff/session.json", json.dumps({
        "runtime_closure_binding": {"manifest_sha256": closure_binding["runtime_closure_manifest_sha256"],
                                    "closure_sha256": closure_binding["runtime_closure_sha256"]},
    }))
    payloads = {}
    handoff_payloads = {}
    for index, (role, relative) in enumerate(MODULE.EXPECTED_PAYLOAD_PATHS.items(), 1):
        source_relative = f"handoff/payloads/{role}.bin"
        _write(tmp_path, source_relative, (role * index).encode())
        payloads[role] = {"target_relative_path": relative, "sha256": _digest(tmp_path / source_relative),
                          "byte_size": len((role * index).encode())}
        handoff_payloads[role] = _entry(tmp_path, source_relative)
    model_receipt = {"schema_version": 1, "receipt_id": MODULE.MODEL_RECEIPT_ID, "status": "VERIFIED", "payloads": payloads}
    model_path = _write(tmp_path, "handoff/model_payload.json", json.dumps(model_receipt, sort_keys=True))
    final_identity = {"session_sha256": _digest(session_path), "session_byte_size": session_path.stat().st_size,
                      "runtime_closure_binding": closure_binding}
    final = {"schema_version": 1, "report_id": MODULE.FINAL_REPORT_ID, "operation_boundary": MODULE.FINAL_BOUNDARY,
             "status": "PREDEPLOY_READY_NOT_DEPLOYED", "ready_to_deploy": True,
             "checks": {"receipt_chain": True, "thermal_external": True}, "blockers": [],
             "pc_session_runtime_identity": final_identity, "board_handoff_binding": dict(final_identity),
             "receipt_requirements": {"receipts": {"model_payload": {"present": True, "sha256": _digest(model_path),
                                                                     "byte_size": model_path.stat().st_size}}}}
    _write(tmp_path, "handoff/final.json", json.dumps(final, sort_keys=True))
    entries = {"final_predeploy": _entry(tmp_path, "handoff/final.json"),
               "model_payload_receipt": _entry(tmp_path, "handoff/model_payload.json"),
               "acceptance_session": _entry(tmp_path, "handoff/session.json"),
               "runtime_closure": _entry(tmp_path, "handoff/closure.json"), "payloads": handoff_payloads}
    _write(tmp_path, "handoff/manifest.json", json.dumps({"schema_version": 1, "entries": entries}, sort_keys=True))
    return Path("handoff/manifest.json")


def _stat(path, split=False, bpu_mode=stat.S_IFCHR):
    text = str(path).replace("\\", "/")
    if text.endswith("bpu_core0"):
        return Stat(bpu_mode, 1)
    return Stat(stat.S_IFDIR, 2 if split and text.endswith("/rollback") else 1)


def _run(tmp_path, **overrides):
    manifest = overrides.pop("handoff_manifest", None) or _fixture(tmp_path)
    values = {"handoff_manifest": manifest, "board_root": tmp_path,
              "candidate": "/opt/tzcup/stages/v1", "retained_old": "/opt/tzcup/rollback/v0",
              "stat_path": _stat, "disk_usage": lambda _: type("D", (), {"free": 10**12})(),
              "platform_machine": lambda: "aarch64"}
    values.update(overrides)
    return MODULE.validate(**values)


def test_realistic_board_handoff_is_ready_only_for_controlled_stage(tmp_path):
    result = _run(tmp_path)
    assert result["status"] == "READY_FOR_CONTROLLED_STAGE"
    assert result["payload_copy_performed"] is False and result["node_started"] is False
    manifest = json.loads((tmp_path / "handoff/manifest.json").read_text())
    assert manifest["entries"]["payloads"]["dosod_hbm"]["relative_path"] != MODULE.EXPECTED_PAYLOAD_PATHS["dosod_hbm"]


def test_platform_identity_is_injected_not_fake_proc_arch(tmp_path):
    result = _run(tmp_path, platform_machine=lambda: "x86_64")
    assert "board_identity_not_exact_s100p_aarch64" in result["blockers"]


def test_handwritten_ready_final_is_not_sufficient(tmp_path):
    manifest = _fixture(tmp_path)
    final = tmp_path / "handoff/final.json"
    final.write_text(json.dumps({"status": "PREDEPLOY_READY_NOT_DEPLOYED", "ready_to_deploy": True}))
    _refresh_manifest_entry(tmp_path, "final_predeploy")
    result = MODULE.validate(handoff_manifest=manifest, board_root=tmp_path, candidate="/opt/tzcup/stages/v1",
                             retained_old="/opt/tzcup/rollback/v0", stat_path=_stat,
                             disk_usage=lambda _: type("D", (), {"free": 10**12})(), platform_machine=lambda: "aarch64")
    assert "final_predeploy_not_ready_not_deployed" in result["blockers"]


def test_receipt_or_payload_drift_blocks(tmp_path):
    _fixture(tmp_path)
    (tmp_path / "handoff/payloads/dosod_hbm.bin").write_bytes(b"drift")
    result = MODULE.validate(handoff_manifest=Path("handoff/manifest.json"), board_root=tmp_path,
                             candidate="/opt/tzcup/stages/v1", retained_old="/opt/tzcup/rollback/v0", stat_path=_stat,
                             disk_usage=lambda _: type("D", (), {"free": 10**12})(), platform_machine=lambda: "aarch64")
    assert "board_handoff_payload_dosod_hbm_digest_or_size_mismatch" in result["blockers"]


def test_board_local_model_receipt_hash_drift_blocks(tmp_path):
    _fixture(tmp_path)
    receipt = tmp_path / "handoff/model_payload.json"
    receipt.write_text("{\"drift\":true}")
    result = MODULE.validate(handoff_manifest=Path("handoff/manifest.json"), board_root=tmp_path,
                             candidate="/opt/tzcup/stages/v1", retained_old="/opt/tzcup/rollback/v0", stat_path=_stat,
                             disk_usage=lambda _: type("D", (), {"free": 10**12})(), platform_machine=lambda: "aarch64")
    assert "board_handoff_model_payload_receipt_digest_or_size_mismatch" in result["blockers"]


def test_existing_candidate_cross_filesystem_and_space_block(tmp_path):
    _fixture(tmp_path)
    (tmp_path / "opt/tzcup/stages/v1").mkdir()
    result = _run(tmp_path, handoff_manifest=Path("handoff/manifest.json"), stat_path=lambda path: _stat(path, split=True),
                  disk_usage=lambda _: type("D", (), {"free": 0})())
    assert {"candidate_target_not_fresh_absent", "active_candidate_retained_old_parents_not_same_filesystem",
            "same_filesystem_free_space_below_payload_plus_margin"}.issubset(result["blockers"])


def test_missing_handoff_and_bad_bpu_block(tmp_path):
    _fixture(tmp_path)
    (tmp_path / "handoff/manifest.json").unlink()
    result = _run(tmp_path, handoff_manifest=Path("handoff/manifest.json"), stat_path=lambda path: _stat(path, bpu_mode=stat.S_IFREG))
    assert {"board_handoff_manifest_missing_or_linked", "bpu_core0_not_nonlink_character_device"}.issubset(result["blockers"])


def test_manifest_and_role_paths_reject_traversal(tmp_path):
    _fixture(tmp_path)
    result = _run(tmp_path, handoff_manifest=Path("handoff/../handoff/manifest.json"), candidate="/opt/../escape")
    assert {"board_handoff_manifest_path_invalid", "active_candidate_or_retained_old_path_invalid"}.issubset(result["blockers"])


def test_handoff_intermediate_symlink_is_rejected(tmp_path):
    _fixture(tmp_path)
    link = tmp_path / "handoff-link"
    try:
        link.symlink_to(tmp_path / "handoff", target_is_directory=True)
    except OSError as exc:
        pytest.skip(f"symlink fixture unsupported: {exc}")
    result = _run(tmp_path, handoff_manifest=Path("handoff-link/manifest.json"))
    assert "board_handoff_manifest_missing_or_linked" in result["blockers"]


@pytest.mark.parametrize("margin", [0, -1])
def test_nonpositive_safety_margin_is_fail_closed(tmp_path, margin):
    result = _run(tmp_path, safety_margin_bytes=margin)
    assert result["status"] == "BLOCKED"
    assert "safety_margin_bytes_not_positive_integer" in result["blockers"]


def test_runtime_closure_binding_must_match_handoff_session(tmp_path):
    _fixture(tmp_path)
    final_path = tmp_path / "handoff/final.json"
    final = json.loads(final_path.read_text())
    final["pc_session_runtime_identity"]["runtime_closure_binding"]["runtime_closure_sha256"] = "d" * 64
    final["board_handoff_binding"] = dict(final["pc_session_runtime_identity"])
    final_path.write_text(json.dumps(final, sort_keys=True))
    _refresh_manifest_entry(tmp_path, "final_predeploy")
    result = _run(tmp_path, handoff_manifest=Path("handoff/manifest.json"))
    assert "board_handoff_session_or_runtime_closure_binding_mismatch" in result["blockers"]


def test_final_handoff_duplicate_cannot_drift_from_pc_identity(tmp_path):
    _fixture(tmp_path)
    final_path = tmp_path / "handoff/final.json"
    final = json.loads(final_path.read_text())
    final["board_handoff_binding"]["session_sha256"] = "d" * 64
    final_path.write_text(json.dumps(final, sort_keys=True))
    _refresh_manifest_entry(tmp_path, "final_predeploy")
    result = _run(tmp_path, handoff_manifest=Path("handoff/manifest.json"))
    assert "final_predeploy_not_ready_not_deployed" in result["blockers"]


def test_role_paths_and_active_mount_filesystem_are_strict(tmp_path):
    _fixture(tmp_path)
    manifest = Path("handoff/manifest.json")
    invalid_path = _run(tmp_path, handoff_manifest=manifest, candidate="/opt/tzcup/stages/release/nested")
    assert "active_candidate_retained_old_roles_not_exact" in invalid_path["blockers"]
    mounted_active = _run(tmp_path, handoff_manifest=manifest,
                          stat_path=lambda path: Stat(stat.S_IFDIR, 2 if str(path).replace("\\", "/").endswith("/s100p") else 1))
    assert "active_candidate_retained_old_parents_not_same_filesystem" in mounted_active["blockers"]


def test_ci_and_documented_observed_board_boundary_remain_fail_closed():
    root = PATH.parents[1]
    ci_fast = (root / "scripts/ci_fast.py").read_text(encoding="utf-8")
    assert 'ROOT / "scripts" / "test_collect_s100p_g0_inventory.py"' in ci_fast
    assert 'ROOT / "scripts" / "test_validate_s100p_board_deployment_preflight.py"' in ci_fast
    document = (root / "docs/s100p-board-deployment-preflight.md").read_text(encoding="utf-8")
    for required in ("`/opt/tzcup/s100p` as a\ndirectory", "`/opt/tzcup/rollback` as a directory",
                     "`/opt/tzcup/stages` as\nabsent", "`27,037,792 KiB`", "must therefore currently return `BLOCKED`"):
        assert required in document
