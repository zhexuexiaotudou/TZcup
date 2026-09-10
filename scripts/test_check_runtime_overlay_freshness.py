from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import unittest


SCRIPT = Path(__file__).with_name("check_runtime_overlay_freshness.py")


def _write(path: Path, text: str = "payload\n") -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def _make_workspace(root: Path) -> Path:
    for package in (
        "sanitation_campus_scenario",
        "sanitation_formal_campus_integration",
        "sanitation_hmi",
    ):
        source_package = root / "src" / package / package
        _write(source_package / "__init__.py", f"NAME = {package!r}\n")
        _write(source_package / "runtime.py", "VALUE = 1\n")
        _write(root / "src" / package / "launch" / "demo.launch.py", "LAUNCH = True\n")
        _write(root / "src" / package / "config" / "demo.yaml", "demo: true\n")
        installed_package = root / "install" / package / "lib" / "python3.12" / "site-packages" / package
        _write(installed_package / "__init__.py", f"NAME = {package!r}\n")
        _write(installed_package / "runtime.py", "VALUE = 1\n")
        _write(root / "install" / package / "share" / package / "launch" / "demo.launch.py", "LAUNCH = True\n")
        _write(root / "install" / package / "share" / package / "config" / "demo.yaml", "demo: true\n")

    vehicle = root / "src" / "sanitation_vehicle_description"
    for payload in ("launch", "config", "urdf", "worlds"):
        _write(vehicle / payload / "payload.txt", f"{payload}\n")
        _write(
            root / "install" / "sanitation_vehicle_description" / "share"
            / "sanitation_vehicle_description" / payload / "payload.txt",
            f"{payload}\n",
        )

    cpp = root / "src" / "sanitation_gazebo_control"
    _write(cpp / "CMakeLists.txt", "project(sanitation_gazebo_control)\n")
    _write(cpp / "src" / "control.cc", "int control() { return 1; }\n")
    _write(cpp / "include" / "control.hh", "int control();\n")
    _write(root / "install" / "sanitation_gazebo_control" / "lib" / "libSanitationControl.so", "binary\n")
    _write(root / "install" / "sanitation_gazebo_control" / "lib" / "sanitation_gazebo_control" / "control_node", "binary\n")
    _write(root / "build" / "sanitation_gazebo_control" / "colcon_build.rc", "0\n")
    now = time.time_ns()
    for path in root.rglob("*"):
        if path.is_file():
            os.utime(path, ns=(now + 2_000_000_000, now + 2_000_000_000))
    return root


def _run(root: Path, extra_env: dict[str, str] | None = None) -> tuple[int, dict]:
    env = os.environ.copy()
    sites = [
        root / "install" / package / "lib" / "python3.12" / "site-packages"
        for package in (
            "sanitation_campus_scenario",
            "sanitation_formal_campus_integration",
            "sanitation_hmi",
        )
    ]
    env["PYTHONPATH"] = os.pathsep.join(map(str, sites)) + os.pathsep + env.get("PYTHONPATH", "")
    if extra_env:
        env.update(extra_env)
    completed = subprocess.run(
        [sys.executable, str(SCRIPT), "--runtime-ws", str(root)],
        text=True,
        capture_output=True,
        env=env,
        check=False,
    )
    return completed.returncode, json.loads(completed.stdout)


class RuntimeOverlayFreshnessTests(unittest.TestCase):
    def test_current_copy_passes(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            code, report = _run(_make_workspace(Path(temporary)))
        self.assertEqual(code, 0, report)
        self.assertTrue(report["ok"])
        self.assertEqual(report["packages"]["sanitation_hmi"]["mode"], "installed-copy")

    def test_changed_python_source_fails(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = _make_workspace(Path(temporary))
            _write(root / "src" / "sanitation_hmi" / "sanitation_hmi" / "runtime.py", "VALUE = 2\n")
            code, report = _run(root)
        self.assertEqual(code, 1)
        self.assertIn("python_package_mismatch", {error["code"] for error in report["errors"]})

    def test_share_directory_absent_from_both_source_and_install_passes(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = _make_workspace(Path(temporary))
            shutil.rmtree(root / "src" / "sanitation_hmi" / "config")
            shutil.rmtree(
                root / "install" / "sanitation_hmi" / "share" / "sanitation_hmi" / "config"
            )
            code, report = _run(root)
        self.assertEqual(code, 0, report)
        self.assertEqual(
            report["packages"]["sanitation_hmi"]["share_payloads"]["config"]["status"],
            "not-present",
        )

    def test_newer_cpp_source_fails(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = _make_workspace(Path(temporary))
            newer = time.time_ns() + 5_000_000_000
            os.utime(root / "src" / "sanitation_gazebo_control" / "src" / "control.cc", ns=(newer, newer))
            code, report = _run(root)
        self.assertEqual(code, 1)
        self.assertIn("cpp_artifact_stale", {error["code"] for error in report["errors"]})

    def test_symlink_install_passes(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = _make_workspace(Path(temporary))
            try:
                for package in (
                    "sanitation_campus_scenario",
                    "sanitation_formal_campus_integration",
                    "sanitation_hmi",
                ):
                    source_root = root / "src" / package
                    installed_package = root / "install" / package / "lib" / "python3.12" / "site-packages" / package
                    shutil.rmtree(installed_package)
                    installed_package.symlink_to(source_root / package, target_is_directory=True)
                    for payload in ("launch", "config"):
                        installed = root / "install" / package / "share" / package / payload
                        shutil.rmtree(installed)
                        installed.symlink_to(source_root / payload, target_is_directory=True)
                code, report = _run(root)
            except OSError as exc:  # Windows without Developer Mode cannot create test symlinks.
                self.skipTest(f"symlinks unavailable: {exc}")
        self.assertEqual(code, 0, report)
        self.assertEqual(report["packages"]["sanitation_hmi"]["mode"], "symlink-install")

    def test_external_import_fails(self) -> None:
        with tempfile.TemporaryDirectory() as temporary, tempfile.TemporaryDirectory() as external_temporary:
            root = _make_workspace(Path(temporary))
            external = Path(external_temporary) / "sanitation_hmi"
            _write(external / "__init__.py", "NAME = 'external'\n")
            sites = [
                root / "install" / package / "lib" / "python3.12" / "site-packages"
                for package in (
                    "sanitation_campus_scenario",
                    "sanitation_formal_campus_integration",
                    "sanitation_hmi",
                )
            ]
            pythonpath = os.pathsep.join([str(external.parent), *map(str, sites)])
            code, report = _run(root, {"PYTHONPATH": pythonpath})
        self.assertEqual(code, 1)
        self.assertIn("external_path", {error["code"] for error in report["errors"]})


if __name__ == "__main__":
    unittest.main()
