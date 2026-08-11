from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))

from sanitation_learning.ddrv4_boundary import (
    DDRV4BoundaryError,
    G5_V2_DATASET_ID,
    require_sealed_access,
)


def test_g5v2_denied_before_freeze():
    for freeze in (None, {}, {"protocol": "OPRV3-08", "status": "FROZEN_X86"}):
        with pytest.raises(DDRV4BoundaryError, match="denied before"):
            require_sealed_access(G5_V2_DATASET_ID, freeze)


def test_g5v2_requires_one_shot_ddrv4_contract():
    freeze = {
        "protocol": "DDRV4-07",
        "status": "FROZEN_X86_DDR_V4",
        "release_boundary": {"DDRV4_X86_DEV_PASS": True},
        "sealed_final": {"dataset": G5_V2_DATASET_ID, "maximum_accesses": 1},
    }
    require_sealed_access(G5_V2_DATASET_ID, freeze)
