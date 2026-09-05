from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
WRAPPER = ROOT / "scripts/build_formal_final_runtime_windows.ps1"


def test_windows_builder_refuses_wsl_before_the_cold_memory_gate() -> None:
    source = WRAPPER.read_text(encoding="utf-8")
    probe = source.index("formal_windows_memory_probe.py")
    refusal = source.index("Cold Windows memory gate refused WSL startup")
    wsl = source.index("& wsl.exe")
    assert probe < refusal < wsl
    assert "ColdMinCommitAvailableBytes = 13421772800" in source
    assert "[ValidateRange(13421772800, [UInt64]::MaxValue)]" in source
    assert "MaxDockerPrivateBytes = 4294967296" in source
    assert "[ValidateRange(0, 4294967296)]" in source
    assert "--check-start --output $preflightJson" in source
    assert '$env:FORMAL_WINDOWS_START_REQUIRE_WSL_STOPPED = "1"' in source
    assert '$env:FORMAL_WINDOWS_START_REQUIRE_WSL_RUNNING = "0"' in source
    assert '"FORMAL_WINDOWS_COLD_GATE_EVIDENCE=$wslPreflightJson"' in source


def test_windows_start_gate_has_a_fixed_bounded_commit_recovery_wait() -> None:
    source = (ROOT / "scripts/formal_windows_memory_probe.py").read_text(
        encoding="utf-8"
    )
    assert "START_COMMIT_RECOVERY_TIMEOUT_S = 60.0" in source
    assert "START_COMMIT_RECOVERY_INTERVAL_S = 5.0" in source
    assert "caller-controlled thresholds" in source
    assert "_commit_only_shortfall(checks)" in source
    assert "sample_count > 1 and passed" in source


def test_windows_builder_keeps_workers_bounded_and_runs_the_inner_gate() -> None:
    source = WRAPPER.read_text(encoding="utf-8")
    shell_builder = (ROOT / "scripts/build_formal_final_runtime.sh").read_text(
        encoding="utf-8"
    )
    memory_probe = (ROOT / "scripts/formal_windows_memory_probe.py").read_text(
        encoding="utf-8"
    )
    assert "[ValidateRange(1, 1)]" in source
    assert "[int]$Workers = 1" in source
    assert '"FORMAL_COLCON_PARALLEL_WORKERS=$Workers"' in source
    assert "must be exactly 1 for formal serial recovery" in shell_builder
    assert '"bash", "scripts/build_formal_final_runtime.sh"' in source
    assert "formal_runtime_memory_preflight" in shell_builder
    assert "FORMAL_WINDOWS_START_MIN_COMMIT_AVAILABLE_BYTES" in memory_probe
    assert "WSL final builds require the Windows cold-start wrapper evidence" in shell_builder
    assert "validate_formal_windows_cold_gate_evidence.py" in shell_builder


def test_windows_builder_never_reuses_evidence_or_build_logs() -> None:
    source = WRAPPER.read_text(encoding="utf-8")
    assert "Refusing to reuse Windows build-guard evidence" in source
    assert "Refusing to overwrite build log" in source
    assert "FORMAL_FINAL_RUNTIME_WINDOWS_GUARD_PASSED" in source
    assert source.index("if ($buildRc -ne 0)") < source.index(
        "FORMAL_FINAL_RUNTIME_WINDOWS_GUARD_PASSED"
    )
    assert "Write-Error" not in source
    assert "[Console]::Error.WriteLine" in source


def test_windows_builder_uses_native_exit_code_not_stderr_progress() -> None:
    source = WRAPPER.read_text(encoding="utf-8")
    invocation = source.index("& wsl.exe @wslArgs")
    capture = source.index("$buildRc = $LASTEXITCODE")
    restore = source.index("$ErrorActionPreference = $savedErrorActionPreference")
    assert '$ErrorActionPreference = "Continue"' in source
    assert invocation < capture < restore


def test_windows_fast_gate_cannot_start_wsl_for_storage_xacro() -> None:
    source = (ROOT / "scripts/ci_fast.py").read_text(encoding="utf-8")
    storage = (
        ROOT / "scripts/test_formal_storage_collision_clearance.py"
    ).read_text(encoding="utf-8")
    assert 'if os.name == "nt":' in source
    assert 'os.environ["TZCUP_STORAGE_GEOMETRY_SOURCE_ONLY"] = "1"' in source
    assert 'os.environ.pop("TZCUP_STORAGE_GEOMETRY_SOURCE_ONLY", None)' in source
    assert 'shutil.which("wsl.exe")' not in storage
    assert "[wsl," not in storage


def test_windows_builder_guard_test_is_executed_by_fast_ci() -> None:
    source = (ROOT / "scripts/ci_fast.py").read_text(encoding="utf-8")
    assert source.count('ROOT / "scripts" / "test_build_formal_final_runtime_windows_guard.py"') == 2


def test_final_acceptance_documentation_requires_the_windows_cold_start_wrapper() -> None:
    documentation = (
        ROOT / "docs/formal-final-acceptance-orchestration.md"
    ).read_text(encoding="utf-8")
    assert "build_formal_final_runtime_windows.ps1" in documentation
    assert "可用提交内存至少 12.5 GiB" in documentation
    assert "`vmmemWSL=0`" in documentation
    assert "不要先手工启动" in documentation
    assert "正式验收不得降低上述阈值或关闭" in documentation
