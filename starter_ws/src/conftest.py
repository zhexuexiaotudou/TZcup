"""Make ROS-independent source packages importable under direct pytest runs.

`scripts/ci_fast.py` inserts every package root into ``sys.path`` before
running pytest.  This conftest mirrors that setup so the same tests also pass
when pytest is invoked directly on individual test files from the repository
root (for example in AUTO-05R-0 recovery validation).
"""

from __future__ import annotations

import sys
from pathlib import Path


SRC_ROOT = Path(__file__).resolve().parent
PACKAGE_ROOTS = (
    "sanitation_coverage",
    "sanitation_tasks",
    "sanitation_gnss_sim",
    "sanitation_perception",
    "sanitation_dataset",
    "sanitation_ground_truth",
    "sanitation_spot_cleaning",
    "sanitation_learning",
    "sanitation_hmi",
    "sanitation_manipulation",
    "sanitation_debug_visualization",
    "sanitation_gazebo_visualization",
)

for name in PACKAGE_ROOTS:
    path = str(SRC_ROOT / name)
    if path not in sys.path:
        sys.path.insert(0, path)
