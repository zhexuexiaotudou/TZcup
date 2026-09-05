#!/usr/bin/env python3
"""Static contract tests for the opt-in Windows NDIS isolation harness."""

from pathlib import Path
import json
import os
import subprocess
import tempfile
import unittest


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPOSITORY_ROOT / "scripts" / "prepare_windows_ndis_isolation.ps1"


class WindowsNdisIsolationHarnessTests(unittest.TestCase):
    def setUp(self) -> None:
        self.source = SCRIPT_PATH.read_text(encoding="utf-8")

    def test_execute_is_explicit_and_target_is_exactly_allowlisted(self) -> None:
        self.assertIn('[ValidateSet("Tailscale", "FSE")]', self.source)
        self.assertIn('if ($Execute -and [string]::IsNullOrWhiteSpace($Target))', self.source)
        self.assertIn('if (-not $Execute)', self.source)
        self.assertIn('Tailscale = [pscustomobject]', self.source)
        self.assertIn('FSE = [pscustomobject]', self.source)
        self.assertNotIn('VPN = [pscustomobject]', self.source)

    def test_default_route_and_physical_wlan_protections_precede_actions(self) -> None:
        self.assertIn('Get-NetRoute -AddressFamily IPv4', self.source)
        self.assertIn('DestinationPrefix -eq "0.0.0.0/0"', self.source)
        self.assertIn('Refusing protected WLAN adapter', self.source)
        self.assertIn('Refusing protected physical adapter', self.source)
        self.assertIn('Refusing', self.source)
        self.assertIn('because it owns an active IPv4 default route', self.source)
        self.assertLess(self.source.index('Assert-SafeIsolationTarget -Plan'), self.source.index('Invoke-TargetIsolation -Plan'))

    def test_state_changes_have_finally_restore_and_nonzero_recovery_failure(self) -> None:
        self.assertIn('try {', self.source)
        self.assertIn('finally {', self.source)
        self.assertIn('Stop-Service -Name $service.name', self.source)
        self.assertIn('Disable-NetAdapter -Name $adapter.name', self.source)
        self.assertIn('Start-Service -Name $serviceName', self.source)
        self.assertIn('Enable-NetAdapter -Name $DisabledAdapterNames[$index]', self.source)
        self.assertIn('$exitCode = 2', self.source)
        self.assertIn('recoveryCommands', self.source)
        self.assertIn('Test-RestoredState -Before $before -Restored $restored', self.source)
        self.assertIn('state_drift = @($stateDrift)', self.source)

    def test_execute_wait_is_bounded_and_only_reachable_in_execute_branch(self) -> None:
        self.assertIn('[ValidateRange(1, 60)]', self.source)
        self.assertIn('[int]$IsolationSeconds = 15', self.source)
        self.assertIn('Start-Sleep -Seconds $IsolationSeconds', self.source)
        self.assertLess(self.source.index('Invoke-TargetIsolation -Plan'), self.source.index('Start-Sleep -Seconds $IsolationSeconds'))
        self.assertIn('actual_elapsed_seconds = $isolationSecondsElapsed', self.source)

    def test_fse_adapters_restore_in_reverse_disable_order(self) -> None:
        self.assertIn('for ($index = $DisabledAdapterNames.Count - 1; $index -ge 0; $index--)', self.source)
        self.assertIn('Enable-NetAdapter -Name $DisabledAdapterNames[$index]', self.source)

    def test_script_uses_exact_catalog_names_not_wildcard_target_selection(self) -> None:
        self.assertIn('$_.Name -ceq $Name', self.source)
        self.assertIn('Name=\'$Name\'', self.source)
        self.assertNotIn('Get-NetAdapter -Name *', self.source)
        self.assertNotIn('Get-Service -Name *', self.source)
        self.assertNotIn('Tailscale*', self.source)
        self.assertNotIn('FSE*', self.source)

    @unittest.skipUnless(os.name == "nt", "PowerShell rejection regression is Windows-only")
    def test_execute_without_target_fails_before_any_action(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            report_path = Path(directory) / "rejected.json"
            completed = subprocess.run(
                [
                    "powershell.exe",
                    "-NoProfile",
                    "-ExecutionPolicy",
                    "Bypass",
                    "-File",
                    str(SCRIPT_PATH),
                    "-Execute",
                    "-OutputPath",
                    str(report_path),
                ],
                check=False,
                capture_output=True,
                text=True,
                timeout=30,
            )
            self.assertEqual(1, completed.returncode, completed.stderr)
            report = json.loads(report_path.read_text(encoding="utf-8-sig"))
            self.assertEqual("execute", report["mode"])
            self.assertIsNone(report["selected_target"])
            self.assertEqual([], report["applied_actions"])
            self.assertEqual([], report["recovery"]["commands"])
            self.assertEqual(1, len(report["errors"]))
            self.assertIn("requires an explicit -Target", report["errors"][0]["error"])


if __name__ == "__main__":
    unittest.main()
