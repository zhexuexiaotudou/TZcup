from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))

from sanitation_learning.ddrv4_boundary import (
    DDRV4BoundaryError,
    G5_DATASET_ID,
    require_sealed_access,
)


def test_g5_consumed_cannot_reopen():
    with pytest.raises(DDRV4BoundaryError, match="consumed and can never reopen"):
        require_sealed_access(
            G5_DATASET_ID,
            {
                "protocol": "DDRV4-07",
                "status": "FROZEN_X86_DDR_V4",
                "release_boundary": {"DDRV4_X86_DEV_PASS": True},
            },
        )
