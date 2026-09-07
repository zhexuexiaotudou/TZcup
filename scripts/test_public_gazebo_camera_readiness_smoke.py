from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.skipif(os.name == "nt" or shutil.which("bash") is None, reason="POSIX shell fixture")
def test_resource_guard_fixture() -> None:
    subprocess.run(
        ["bash", "scripts/test_run_public_gazebo_camera_readiness_smoke.sh"],
        cwd=ROOT,
        check=True,
        timeout=30,
    )
