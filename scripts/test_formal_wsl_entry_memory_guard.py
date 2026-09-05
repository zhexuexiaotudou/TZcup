from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
GUARD = ROOT / "scripts/formal_wsl_entry_memory_guard.ps1"


def test_shared_wsl_entry_guard_selects_strict_cold_and_warm_thresholds() -> None:
    source = GUARD.read_text(encoding="utf-8")
    assert "[switch]$RequireCold" in source
    assert "Get-Process -Name vmmemWSL" in source
    assert "[UInt64]13421772800" in source
    assert "[UInt64]10737418240" in source
    assert 'FORMAL_WINDOWS_START_MAX_DOCKER_PRIVATE_BYTES = "4294967296"' in source
    assert '$requireWslStopped = if ($wslVmRunning) { "0" } else { "1" }' in source
    assert '$requireWslRunning = if ($wslVmRunning) { "1" } else { "0" }' in source
    assert "FORMAL_WINDOWS_START_REQUIRE_WSL_RUNNING" in source
    assert "--check-start --output $EvidencePath" in source
    assert "Refusing stale WSL-entry memory evidence" in source
    assert '$probeRefusal.Data["ExitCode"] = [int]$probeRc' in source
    assert "Stop-Process" not in source
    assert "docker stop" not in source.lower()
    assert "docker kill" not in source.lower()


def test_shared_wsl_entry_guard_require_cold_does_not_downgrade_from_preliminary_vmmem() -> None:
    source = GUARD.read_text(encoding="utf-8")
    require_cold_block = source.split("if ($RequireCold) {", 1)[1].split(
        "} else {", 1
    )[0]

    assert "$minimumCommit = [UInt64]13421772800" in require_cold_block
    assert '$requireWslStopped = "1"' in require_cold_block
    assert '$requireWslRunning = "0"' in require_cold_block
    assert "Get-Process -Name vmmemWSL" not in require_cold_block
    assert source.index("if ($RequireCold) {") < source.index(
        "Get-Process -Name vmmemWSL"
    )


def test_shared_wsl_entry_guard_keeps_the_10_gib_warm_path_for_legacy_calls() -> None:
    source = GUARD.read_text(encoding="utf-8")
    legacy_block = source.split("} else {", 1)[1].split("$oldCommitFloor", 1)[0]

    assert "$minimumCommit = if ($wslVmRunning) {" in legacy_block
    assert "[UInt64]10737418240" in legacy_block
    assert '$requireWslRunning = if ($wslVmRunning) { "1" } else { "0" }' in legacy_block


def test_every_legacy_wsl_launcher_runs_the_shared_guard_before_starting_a_distro() -> None:
    auto16 = (ROOT / "scripts/run_auto16_release.ps1").read_text(encoding="utf-8")
    frozen = (ROOT / "scripts/run_frozen_coverage_trial.ps1").read_text(encoding="utf-8")
    visual = (ROOT / "scripts/run_visual_demo.ps1").read_text(encoding="utf-8")
    for source in (auto16, frozen, visual):
        assert 'formal_wsl_entry_memory_guard.ps1' in source
        assert "Invoke-FormalWslEntryMemoryGuard" in source
    assert auto16.index("Invoke-FormalWslEntryMemoryGuard") < auto16.index("& wsl.exe -d")
    assert auto16.count("Invoke-FormalWslEntryMemoryGuard") == 2
    assert frozen.index("Invoke-FormalWslEntryMemoryGuard") < frozen.index(
        'Start-Process -FilePath "wsl.exe"'
    )
    assert visual.index("Invoke-FormalWslEntryMemoryGuard") < visual.index("& wsl.exe -d")
    assert visual.count("Invoke-FormalWslEntryMemoryGuard") == 5
    assert "wsl_entry_memory_prepare_recovery_preflight.json" in visual
    assert "wsl_entry_memory_recovery_preflight.json" in visual
    assert "wsl_entry_memory_launch_preflight.json" in visual
    assert "wsl_entry_memory_retry_launch_preflight.json" in visual
