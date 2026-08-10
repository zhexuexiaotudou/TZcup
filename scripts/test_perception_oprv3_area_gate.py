import importlib.util
from pathlib import Path

import pytest


SCRIPT = Path(__file__).with_name("perception_oprv3_area_gate.py")
SPEC = importlib.util.spec_from_file_location("oprv3_area_gate", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def _metrics(iou=(0.9, 0.85), boundary=(0.8, 0.76), negative=(1, 100)):
    return {
        "development_selected_postprocess": {
            "iou_by_class": {"leaf_pile": iou[0], "puddle": iou[1]},
            "postprocessed_mask_boundary_f1": sum(boundary) / 2,
            "negative_area_fp_per_frame": negative[0] / negative[1],
            "pixel_totals": {
                "intersection": [int(iou[0] * 1000), int(iou[1] * 1000)],
                "union": [1000, 1000],
                "boundary_intersection": [400, 380],
                "boundary_union": [600, 620],
                "negative_frames": negative[1],
                "negative_fp_frames": negative[0],
            },
        }
    }


def _report():
    return {
        "G5_SEALED_FINAL_read": False,
        "legacy_G4_D6_read": False,
        "models": {
            task: {
                "checkpoint_status": "training_complete",
                "sha256": str(index) * 64,
            }
            for index, task in enumerate(MODULE.MODEL_TASKS, start=1)
        },
        "development_selected_config": {"thresholds": [0.8, 0.85]},
        "splits": {name: _metrics() for name in MODULE.SPLITS},
    }


def test_area_gate_aggregates_fixed_splits_and_passes() -> None:
    result = MODULE.aggregate(_report())
    assert result["OPRV3_06_AREA_PASS"]
    assert result["cross_world_aggregate"]["area"]["negative_only_frames"] == 600
    assert result["cross_world_aggregate"]["area"]["boundary_f1"] == pytest.approx(
        0.78
    )


def test_area_gate_fails_closed_on_missing_split_or_sealed_read() -> None:
    report = _report()
    del report["splits"]["D5"]
    with pytest.raises(ValueError, match="missing fixed splits"):
        MODULE.aggregate(report)
    report = _report()
    report["G5_SEALED_FINAL_read"] = True
    with pytest.raises(ValueError, match="sealed-final"):
        MODULE.aggregate(report)


def test_area_gate_requires_completed_checkpoint_provenance() -> None:
    report = _report()
    report["models"]["puddle"]["checkpoint_status"] = "in_progress"
    with pytest.raises(ValueError, match="not training_complete"):
        MODULE.aggregate(report)
