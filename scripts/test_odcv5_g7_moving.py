from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))

from sanitation_learning.g7_moving_dataset import (  # noqa: E402
    G7MovingPlan,
    REQUIRED_COVERAGE,
    build_g7_moving_dataset,
)


def test_small_pack_has_complete_capture_and_split_isolation(tmp_path: Path):
    plan = G7MovingPlan(
        missions_by_split={"MOVING_TRAIN": 2, "MOVING_HOLDOUT": 2, "MOVING_VAL": 2},
        frames_per_mission=12,
        formal=False,
    )
    qa = build_g7_moving_dataset(tmp_path / "g7-moving", plan)
    assert qa["gates"]["mission_complete_100_percent"] is True
    assert qa["gates"]["capture_contract_complete"] is True
    assert qa["G7_MOVING_PASS"] is False
    assert qa["gates"]["required_coverage_complete"] is False
    assert qa["frame_count"] == 72
    assert qa["isolation"] == {"world_overlap": 0, "seed_overlap": 0}
    assert qa["gates"]["capture_contract_complete"] is True
    assert qa["gates"]["product_manifest_has_no_gt_class_coordinates_or_instance_id"] is True
    coverage = __import__("json").loads(
        (tmp_path / "g7-moving/reports/G7_MOVING_COVERAGE_MATRIX.json").read_text(encoding="utf-8")
    )
    assert set(coverage["required"]) == set(REQUIRED_COVERAGE)


def test_nonempty_output_is_rejected(tmp_path: Path):
    output = tmp_path / "g7-moving"
    output.mkdir()
    (output / "foreign.txt").write_text("do not overwrite", encoding="utf-8")
    try:
        build_g7_moving_dataset(output, G7MovingPlan(
            missions_by_split={"MOVING_TRAIN": 1, "MOVING_HOLDOUT": 1, "MOVING_VAL": 1},
            formal=False,
        ))
    except FileExistsError:
        pass
    else:
        raise AssertionError("non-empty output must fail closed")
