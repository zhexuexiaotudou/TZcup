from __future__ import annotations

import json
from pathlib import Path

import pytest

import formal_final_runtime_closure as closure
import formal_native_linux_cold_start_evidence as native


def _write_build_start(runtime: Path) -> Path:
    path = runtime / native.SOURCE_NAME
    path.parent.mkdir(parents=True)
    path.write_text(
        json.dumps(
            {
                "report_id": native.SOURCE_REPORT_ID,
                "status": "FORMAL_FINAL_BUILD_LINUX_MEMORY_START_PASSED",
                "passed": True,
                "sample_epoch_ns": 123,
                "thresholds_kib": {
                    "min_mem_available": native.MIN_MEMORY_KIB,
                    "max_swap_used": native.MAX_SWAP_KIB,
                },
                "observed_kib": {
                    "mem_available": native.MIN_MEMORY_KIB + 1,
                    "swap_total": 100,
                    "swap_free": 80,
                    "swap_used": 20,
                },
                "checks": {
                    "mem_available_at_least_configured_minimum": True,
                    "swap_used_at_most_configured_maximum": True,
                },
                "signals": {
                    "exact_pgid_only": True,
                    "docker_signalled_or_stopped": False,
                },
            }
        )
        + "\n",
        encoding="utf-8",
    )
    return path


def test_bind_and_validate_native_linux_evidence(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    runtime = tmp_path / "runtime"
    _write_build_start(runtime)
    monkeypatch.setenv("FORMAL_NATIVE_LINUX_RUNTIME", "1")
    monkeypatch.setattr(native.os, "name", "posix")
    monkeypatch.setattr(native.platform, "release", lambda: "6.8.0-native")
    target = native.bind(runtime)
    payload = native.validate_bound(target, runtime.resolve())
    assert payload["status"] == "FORMAL_NATIVE_LINUX_COLD_START_GATE_PASSED"
    identity = closure._windows_cold_start_evidence_identity(runtime.resolve())
    assert identity["bound"] is True
    assert identity["mode"] == "native_linux_not_wsl"
    assert identity["report_id"] == native.BOUND_REPORT_ID


def test_build_start_rejects_inconsistent_swap(tmp_path: Path) -> None:
    runtime = tmp_path / "runtime"
    path = _write_build_start(runtime)
    payload = json.loads(path.read_text(encoding="utf-8"))
    payload["observed_kib"]["swap_used"] = 19
    path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(native.NativeEvidenceError, match="swap sample is inconsistent"):
        native.validate_build_start(path)
