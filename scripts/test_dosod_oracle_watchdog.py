from __future__ import annotations

import importlib.util
import json
import signal
import subprocess
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).parent


def mod(name: str):
    spec = importlib.util.spec_from_file_location(name, HERE / f"{name}.py")
    module = importlib.util.module_from_spec(spec); assert spec.loader
    sys.modules[name] = module; spec.loader.exec_module(module)
    return module


common = mod("hbm_evidence_common")
candidate = mod("execute_dosod_nonformal_oracle_candidate_compile")
supervisor = mod("run_dosod_single_frame_preprocessing_oracle_supervised")


def _report(*, status="FORMAL_MEMORY_WATCHDOG_COMPLETED", survivors=0):
    return {"status": status, "target_pgid": 44, "surviving_group_processes": survivors,
            "breach_exit_code": 86,
            "thresholds_kib": {"min_mem_available": 3145728, "max_swap_used": 1048576, "max_group_rss": 9437184}}


def test_owned_child_hang_uses_term_then_kill_and_no_survivor(monkeypatch):
    class Child:
        pid = 44; returncode = 0
        def __init__(self): self.calls = 0
        def communicate(self, timeout=None):
            self.calls += 1
            if self.calls < 3: raise subprocess.TimeoutExpired(["hang"], timeout)
            return "", ""
    child = Child(); alive = {"value": True}; signals = []
    monkeypatch.setattr(common.os, "name", "posix", raising=False)
    monkeypatch.setattr(common.signal, "SIGKILL", 9, raising=False)
    monkeypatch.setattr(common.subprocess, "Popen", lambda *_, **__: child)
    monkeypatch.setattr(common.os, "getpgid", lambda _: 44, raising=False)
    def killpg(_, sig):
        if sig == 0:
            if not alive["value"]: raise ProcessLookupError
        else:
            signals.append(sig)
            if sig == common.signal.SIGKILL: alive["value"] = False
    monkeypatch.setattr(common.os, "killpg", killpg, raising=False)
    _, _, _, details = common.run_owned_process(["hang"], timeout_seconds=1)
    assert details["timed_out"] and details["zero_survivor"]
    assert signals[:2] == [signal.SIGTERM, common.signal.SIGKILL]


def test_watchdog_start_failure_contains_exact_child_group(monkeypatch):
    class Child:
        pid = 44
    alive = {"value": True}; calls = []; launches = {"count": 0}
    monkeypatch.setattr(common.os, "name", "posix", raising=False)
    monkeypatch.setattr(common.os, "getpgid", lambda _: 44, raising=False)
    def popen(*_, **__):
        launches["count"] += 1
        if launches["count"] == 1: return Child()
        raise OSError("watchdog")
    monkeypatch.setattr(common.subprocess, "Popen", popen)
    def killpg(_, sig):
        calls.append(sig)
        if sig == 0:
            if not alive["value"]: raise ProcessLookupError
        elif sig == signal.SIGTERM: alive["value"] = False
    monkeypatch.setattr(common.os, "killpg", killpg, raising=False)
    with pytest.raises(OSError, match="watchdog"):
        common.run_owned_process(["child"], timeout_seconds=1, memory_watchdog={"script": HERE / "formal_memory_watchdog.sh", "json": Path("/tmp/x.json"), "log": Path("/tmp/x.log")})
    assert signal.SIGTERM in calls


@pytest.mark.parametrize("status,survivors", [("FORMAL_MEMORY_LIMIT_BREACHED", 0), ("FORMAL_MEMORY_WATCHDOG_COMPLETED", 1)])
def test_candidate_watchdog_breach_or_survivor_is_blocked(tmp_path, status, survivors):
    report, log = tmp_path / "memory_watchdog.json", tmp_path / "memory_watchdog.log"
    report.write_text(json.dumps(_report(status=status, survivors=survivors))); log.write_text("watchdog")
    evidence = {"json": {"relative_path": report.name, "sha256": common.sha256_file(report), "byte_size": report.stat().st_size},
                "log": {"relative_path": log.name, "sha256": common.sha256_file(log), "byte_size": log.stat().st_size},
                "returncode": 86 if status != "FORMAL_MEMORY_WATCHDOG_COMPLETED" else 0, "status": status, "pgid": 44}
    with pytest.raises(ValueError, match="memory_watchdog_failed"):
        common.require_completed_memory_watchdog(tmp_path, evidence, "candidate_memory_watchdog")


def test_wrapper_retains_stale_watchdog_failure_and_handoff_htr_has_no_watcher(tmp_path, monkeypatch):
    candidate_receipt, capture, model, hrt = [tmp_path / name for name in ("candidate.json", "capture.json", "model.onnx", "hrt")]
    for path in (candidate_receipt, capture, model, hrt): path.write_text("x")
    def fake_run(*_, **kwargs):
        root = Path(kwargs["memory_watchdog"]["json"]).parent
        return 125, "", "", {"pgid": 44, "deadline_seconds": 180, "term_grace_seconds": 10,
                                 "timed_out": False, "term_sent": True, "kill_sent": True, "zero_survivor": True,
                                 "elapsed_seconds": 0.1, "memory_watchdog": {"json_path": str(root / "missing.json"),
                                 "log_path": str(root / "missing.log"), "returncode": 125, "status": None}}
    monkeypatch.setattr(supervisor, "run_owned_process", fake_run)
    result = supervisor.supervise(candidate_receipt=candidate_receipt, official_capture_receipt=capture, onnx_model=model, hrt=hrt, output=tmp_path / "out")
    assert result["status"] == "BLOCKED" and result["memory_watchdog"]["json"] is None
    observed = {}
    monkeypatch.setenv("TZCUP_ORACLE_OUTER_SUPERVISED", "1")
    monkeypatch.setattr(supervisor, "run_owned_process", fake_run)
    monkeypatch.setattr(common, "run_owned_process", lambda command, **kwargs: observed.update(kwargs) or (0, "", "", {"direct_process_reaped": True}))
    collector = mod("collect_dosod_single_frame_preprocessing_oracle")
    monkeypatch.setattr(collector, "run_owned_process", common.run_owned_process)
    collector._run(["hrt"], 1)
    assert observed["start_new_session"] is False and "memory_watchdog" not in observed
