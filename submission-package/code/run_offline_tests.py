#!/usr/bin/env python3
"""Run the source-bundle offline unit tests with the packaged ROS Python path."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.dont_write_bytecode = True

ROOT = Path(__file__).resolve().parent
for path in (
    ROOT / "ros2_ws" / "src" / "sanitation_tasks",
    ROOT / "project_scripts",
):
    sys.path.insert(0, str(path))

raise SystemExit(
    pytest.main(
        [
            "-p",
            "no:cacheprovider",
            "-v",
            str(ROOT / "project_scripts" / "test_offline_raycast_mapping.py"),
            str(ROOT / "project_scripts" / "test_verify_map_area.py"),
            str(
                ROOT
                / "project_scripts"
                / "test_run_day1_localization_stabilizer_live_contract.py"
            ),
        ]
    )
)