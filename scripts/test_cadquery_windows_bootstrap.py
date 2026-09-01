#!/usr/bin/env python3
"""Regression tests for the native CadQuery bootstrap contract."""

from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PREFLIGHT_PATH = ROOT / "scripts" / "cadquery_windows_preflight.py"
ROUNDTRIP_PATH = ROOT / "scripts" / "run_cadquery_step_roundtrip.py"
BOOTSTRAP_PATH = ROOT / "scripts" / "bootstrap_native_cadquery_windows.ps1"
LOCK_PATH = ROOT / "config" / "cadquery-windows-cp313.lock"

SPEC = importlib.util.spec_from_file_location("cadquery_windows_preflight", PREFLIGHT_PATH)
assert SPEC and SPEC.loader
PREFLIGHT = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = PREFLIGHT
SPEC.loader.exec_module(PREFLIGHT)

ROUNDTRIP_SPEC = importlib.util.spec_from_file_location("cadquery_step_roundtrip", ROUNDTRIP_PATH)
assert ROUNDTRIP_SPEC and ROUNDTRIP_SPEC.loader
ROUNDTRIP = importlib.util.module_from_spec(ROUNDTRIP_SPEC)
sys.modules[ROUNDTRIP_SPEC.name] = ROUNDTRIP
ROUNDTRIP_SPEC.loader.exec_module(ROUNDTRIP)


class _FakeShape:
    def Solids(self) -> list[object]:
        return [object()]

    def Faces(self) -> list[object]:
        return [object(), object()]

    def Edges(self) -> list[object]:
        return [object(), object(), object()]

    def Vertices(self) -> list[object]:
        return [object(), object(), object(), object()]


class CadQueryWindowsBootstrapTests(unittest.TestCase):
    def test_low_memory_blocks_without_attempting_install(self) -> None:
        report = PREFLIGHT.build_report(
            ROOT,
            host=PREFLIGHT.HostProbe(is_windows=True, free_memory_mib=1705.5, free_disk_mib=200_000),
            interpreter=PREFLIGHT.InterpreterProbe(
                executable="C:/Python313/python.exe",
                implementation="CPython",
                version=(3, 13, 7),
                bits=64,
            ),
        )
        self.assertEqual("blocked", report["outcome"])
        self.assertFalse(report["bootstrap_permitted"])
        self.assertIn("INSUFFICIENT_FREE_PHYSICAL_MEMORY", {item["code"] for item in report["blocked_reasons"]})

    def test_matching_windows_host_is_permitted(self) -> None:
        report = PREFLIGHT.build_report(
            ROOT,
            host=PREFLIGHT.HostProbe(is_windows=True, free_memory_mib=8192, free_disk_mib=16_384),
            interpreter=PREFLIGHT.InterpreterProbe(
                executable="C:/Python313/python.exe",
                implementation="CPython",
                version=(3, 13, 7),
                bits=64,
            ),
        )
        self.assertEqual("ready_to_bootstrap", report["outcome"])
        self.assertTrue(report["bootstrap_permitted"])

    def test_lock_and_bootstrap_are_hash_pinned_and_project_local(self) -> None:
        lock = LOCK_PATH.read_text(encoding="utf-8")
        bootstrap = BOOTSTRAP_PATH.read_text(encoding="utf-8")
        self.assertIn("cadquery==2.8.0 --hash=sha256:0acaddf36415551c26c3f8237907980ba86c277e211c32398121c839f719c81f", lock)
        self.assertIn("cadquery-ocp==7.9.3.1.1 --hash=sha256:425e8e5ebe351bb3dd5b1e01023737566acd28da191d3b9071e24e1b2dd14dae", lock)
        self.assertIn("--require-hashes", bootstrap)
        self.assertIn(".work", bootstrap)
        self.assertIn('$LockedPythonLauncher = "py"', bootstrap)
        self.assertIn('$LockedPythonArguments = @("-3.13")', bootstrap)
        self.assertNotIn("[string]$PythonExecutable", bootstrap)
        self.assertNotIn("[string[]]$PythonArguments", bootstrap)
        self.assertEqual(4, bootstrap.count("Invoke-LockedCadQueryPython"))
        self.assertNotIn("& wsl", bootstrap.lower())
        self.assertNotIn("& docker", bootstrap.lower())

    def test_roundtrip_keeps_outputs_under_work_and_reports_topology(self) -> None:
        allowed = ROOT / ".work" / "unit-test"
        self.assertEqual(allowed.resolve(), ROUNDTRIP.require_project_local_work_directory(ROOT, allowed))
        with self.assertRaises(ValueError):
            ROUNDTRIP.require_project_local_work_directory(ROOT, ROOT / "reports" / "engineering")
        self.assertEqual(
            {"solids": 1, "faces": 2, "edges": 3, "vertices": 4},
            ROUNDTRIP.topology_counts(_FakeShape()),
        )


if __name__ == "__main__":
    unittest.main()
