#!/usr/bin/env python3
"""Static regression tests for the read-only Windows NDIS pool diagnostic."""

from pathlib import Path
import unittest


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPOSITORY_ROOT / "scripts" / "diagnose_windows_ndis_nonpaged_pool.ps1"


class WindowsNdisPoolDiagnosticContractTests(unittest.TestCase):
    def test_script_collects_required_evidence_and_candidate_classes(self) -> None:
        source = SCRIPT_PATH.read_text(encoding="utf-8")
        for required in (
            "PoolNonpagedBytes",
            "Get-NetAdapter -IncludeHidden",
            "Get-NetAdapterBinding -IncludeHidden",
            "Win32_SystemDriver",
            "Win32_PnPSignedDriver",
            "Win32_Service",
            "Microsoft-Windows-NDIS/Operational",
            "unavailable_poolmon_not_installed",
            "tailscale",
            "fse",
            "ikuuu_vpn",
            "realtek",
            "intel_wlan",
            "possible",
            "unproven",
        ):
            self.assertIn(required, source)

    def test_script_has_no_state_changing_network_or_registry_command(self) -> None:
        source = SCRIPT_PATH.read_text(encoding="utf-8").lower()
        forbidden = (
            "stop-service",
            "restart-service",
            "disable-netadapter",
            "enable-netadapter",
            "restart-netadapter",
            "set-netadapter",
            "new-itemproperty",
            "set-itemproperty",
            "remove-itemproperty",
            "reg add",
            "pnputil /add-driver",
        )
        for command in forbidden:
            self.assertNotIn(command, source)


if __name__ == "__main__":
    unittest.main()
