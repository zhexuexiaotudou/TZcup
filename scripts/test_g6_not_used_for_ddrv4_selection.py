from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))

from sanitation_learning.ddrv4_boundary import (
    DDRV4BoundaryError,
    G6_DATASET_ID,
    G7_DATASET_ID,
    require_ddrv4_selection_inputs,
)


def test_g6_not_used_for_ddrv4_selection():
    with pytest.raises(DDRV4BoundaryError, match="cannot use"):
        require_ddrv4_selection_inputs([G7_DATASET_ID, G6_DATASET_ID])


def test_g7_is_the_only_selection_dataset():
    assert require_ddrv4_selection_inputs([G7_DATASET_ID]) == (G7_DATASET_ID,)
