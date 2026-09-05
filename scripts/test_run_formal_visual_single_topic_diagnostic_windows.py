from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
WRAPPER = ROOT / "scripts/run_formal_visual_single_topic_diagnostic_windows.ps1"


def test_windows_wrapper_enforces_a_fresh_cold_gate_before_wsl() -> None:
    source = WRAPPER.read_text(encoding="utf-8")
    assert "Get-Process -Name vmmemWSL" not in source
    assert "formal_wsl_entry_memory_guard.ps1" in source
    assert "Invoke-FormalWslEntryMemoryGuard -EvidencePath $preflightJson -RequireCold" in source
    assert "require_wsl_stopped -ne $true" in source
    assert "require_wsl_running -ne $false" in source
    assert "13421772800" in source
    assert "4294967296" in source
    assert "$preflightSample = $preflight.sample" in source
    assert "preflightSample.commit_available_bytes" in source
    assert "preflightSample.vmmem_wsl_private_bytes" in source
    assert '$guardExitCode = $_.Exception.Data["ExitCode"]' in source
    assert "exit [int]$guardExitCode" in source
    assert source.index("try {") < source.index(
        "Invoke-FormalWslEntryMemoryGuard -EvidencePath $preflightJson -RequireCold"
    )
    assert source.index("exit [int]$guardExitCode") < source.index(
        "$preflight = Get-Content"
    )
    assert source.index("Invoke-FormalWslEntryMemoryGuard") < source.index(
        "& wsl.exe @wslArgs"
    )
    assert "Stop-Process" not in source
    assert "wsl.exe --shutdown" not in source


def test_windows_wrapper_runs_only_the_bounded_single_topic_diagnostic() -> None:
    source = WRAPPER.read_text(encoding="utf-8")
    assert "run_formal_visual_single_topic_diagnostic.sh" in source
    assert '"--runtime-setup", $runtimeSetup' in source
    assert '"--output-root", $OutputRoot' in source
    assert '"--domain-id", [string]$DomainId' in source
    assert "[int]$DomainId = 225" in source
    assert '"--topic", $Topic' in source
    assert "formal_vehicle_sim.launch.py" not in source
    assert "build_formal_final_runtime.sh" not in source
    assert "Refusing to reuse diagnostic guard evidence" in source
    assert "Refusing to overwrite diagnostic log" in source
    assert "DomainId intersects the Linux ephemeral UDP port range" in source
    assert "[ValidateRange(0, 231)]" in source
    assert "ValidateRange(0, 232)" not in source


def test_windows_wrapper_preserves_diagnostic_exit_status_and_evidence() -> None:
    source = WRAPPER.read_text(encoding="utf-8")
    assert "Tee-Object -FilePath $DiagnosticLog" in source
    assert "$diagnosticRc = $LASTEXITCODE" in source
    assert "exit $diagnosticRc" in source
    assert "cold_start_preflight.json" in source
    assert "FORMAL_VISUAL_SINGLE_TOPIC_WINDOWS_GUARD_PASSED" in source


def test_windows_wrapper_reraises_non_probe_guard_errors() -> None:
    source = WRAPPER.read_text(encoding="utf-8")
    guard_catch_block = source.split("catch {", 1)[1].split("$preflight =", 1)[0]

    assert "if ($null -ne $guardExitCode)" in guard_catch_block
    assert "throw" in guard_catch_block
