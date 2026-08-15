from __future__ import annotations

import sys
from pathlib import Path


_PACKAGE_DIR = Path(__file__).resolve().parents[1]
if str(_PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_DIR))

from sanitation_learning.g4_split_policy import (  # noqa: E402
    stratified_row_sample,
)


def test_stratified_rows_cover_each_world_and_polarity() -> None:
    rows = []
    for world in ("w1", "w2", "w3"):
        for negative_only in (False, True):
            for index in range(10):
                rows.append(
                    {
                        "world_id": world,
                        "negative_only": negative_only,
                        "frame_index": index,
                    }
                )
    selected = stratified_row_sample(rows, 12, seed=7)
    assert len(selected) == 12
    assert {
        (row["world_id"], row["negative_only"]) for row in selected
    } == {
        (world, polarity)
        for world in ("w1", "w2", "w3")
        for polarity in (False, True)
    }
    assert stratified_row_sample(rows, 12, seed=7) == selected
