from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
WRAPPER = ROOT / "scripts/run_formal_sensor_transport_probe_windows.ps1"


def test_wrapper_is_a_cold_windows_admission_gate_before_its_only_wsl_call() -> None:
    source = WRAPPER.read_text(encoding="utf-8")

    cold = source.index(
        "Invoke-FormalWslEntryMemoryGuard -EvidencePath $preflightJson -RequireCold"
    )
    wsl = source.index("& wsl.exe @wslArgs")
    assert cold < wsl
    assert source.count("& wsl.exe @wslArgs") == 1
    assert "formal_wsl_entry_memory_guard.ps1" in source
    assert "no_suspected_ndis_nonpaged_pool_leak" in source
    assert "pool.status -ne \"available\"" in source
    assert "suspected_ndis_nonpaged_pool_leak -ne $false" in source
    assert "13421772800" in source
    assert "4294967296" in source
    assert "vmmem_wsl_private_bytes -ne [UInt64]0" in source


def test_wrapper_derives_fresh_repository_attempt_and_evidence_roots() -> None:
    source = WRAPPER.read_text(encoding="utf-8")

    assert "[ValidatePattern('^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$')]" in source
    assert '$attemptRoot = "$wslRepo/.work/formal_sensor_transport_probe/$RunId"' in source
    assert "Refusing to reuse sensor transport probe evidence" in source
    assert '"--", "bash", "scripts/run_formal_sensor_transport_probe.sh"' in source
    assert '"--attempt-root", $attemptRoot' in source
    assert "RuntimeWs must not be /" in source
    assert "DomainId intersects the Linux ephemeral UDP port range" in source
    assert "[int]$DomainId = 81" in source


def test_wrapper_uses_the_existing_full_rate_session_bound_sensor_runner() -> None:
    source = WRAPPER.read_text(encoding="utf-8")

    assert "run_formal_sensor_transport_probe.sh" in source
    assert "generate_formal_vehicle_snapshot.py" not in source
    assert "formal_acceptance_session.py" not in source
    assert 'attempt_root_windows = $attemptRootWindows' in source
    assert "acceptance_session = Get-FormalEvidenceFile" in source
    assert "runtime_binding = Get-FormalEvidenceFile" in source
    assert "wsl_memory_watchdog = Get-FormalEvidenceFile" in source
    assert "cleanup_attestation = Get-FormalEvidenceFile" in source
    assert "finalize_formal_sensor_transport_probe.py" in source
    assert "FORMAL_ORCHESTRATED_STEP_SESSION" not in source
    assert "run_formal_visual_single_topic_diagnostic.sh" not in source


def test_wrapper_records_post_run_pool_in_finally_and_fails_closed() -> None:
    source = WRAPPER.read_text(encoding="utf-8")

    wsl = source.index("& wsl.exe @wslArgs")
    post = source.index("Invoke-FormalWslEntryMemoryGuard -EvidencePath $postRunPoolJson")
    summary = source.index("$summary = [ordered]@{")
    assert wsl < post < summary
    assert "finally {" in source
    assert "post_run_pool_rc = $postRc" in source
    assert "cold_pool_tags_available_and_not_suspect = $coldPoolVerified" in source
    assert "post_pool_tags_available_and_not_suspect = $postPoolVerified" in source
    assert "$childRc -eq 0" in source
    assert "$postRc -eq 0" in source
    assert "$finalizerRc -eq 0" in source
    assert "Get-FileHash -LiteralPath $Path -Algorithm SHA256" in source


def test_wrapper_never_shuts_down_wsl_stops_processes_or_changes_network() -> None:
    source = WRAPPER.read_text(encoding="utf-8")

    assert "wsl.exe --shutdown" not in source
    assert "Stop-Process" not in source
    assert "Restart-NetAdapter" not in source
    assert "Set-Net" not in source
    assert "network_configuration_changed = $false" in source
