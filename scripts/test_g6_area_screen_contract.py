from pathlib import Path
import importlib.util

import numpy as np
import pytest


pytest.importorskip("torch")
SCRIPT = Path(__file__).with_name("screen_g6_area_recovery.py")
SPEC = importlib.util.spec_from_file_location("g6_area_screen", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def _record() -> dict:
    target = np.zeros((2, 32, 32), dtype=bool)
    target[0, 8:20, 8:20] = True
    target[1, 12:26, 10:24] = True
    probabilities = np.full((2, 32, 32), 0.01, dtype=np.float16)
    probabilities[target] = 0.99
    boundaries = np.stack(
        [MODULE.mask_boundary(target[index]) for index in range(2)]
    ).astype(np.float16)
    negative = np.zeros((32, 32), dtype=bool)
    negative[2:10, 23:31] = True
    return {
        "probabilities": probabilities,
        "boundary_probabilities": boundaries,
        "targets": target,
        "negative": negative,
        "depth_m": np.full((32, 32), 3.0, dtype=np.float16),
        "taxonomy": "wet_asphalt_not_puddle",
    }


def test_holdout_selection_and_negative_region_metric_are_fail_closed() -> None:
    records = [_record(), _record()]
    leaf_config, leaf_metrics, _ = MODULE.select_task_config(records, 0)
    puddle_config, puddle_metrics, _ = MODULE.select_task_config(records, 1)
    assert leaf_metrics["iou"] == pytest.approx(1.0)
    assert puddle_metrics["negative_area_fp_per_frame"] == 0.0
    aggregate = MODULE.aggregate_selected(records, [leaf_config, puddle_config])
    assert aggregate["macro_miou"] == pytest.approx(1.0)
    assert aggregate["postprocessed_mask_boundary_f1"] == pytest.approx(1.0)
    assert aggregate["negative_area_fp_per_frame"] == 0.0

    contaminated = _record()
    contaminated["probabilities"][:, contaminated["negative"]] = 0.99
    aggregate = MODULE.aggregate_selected(
        [contaminated],
        [
            {"threshold": 0.5, "morphology": "none", "minimum_area_m2": 0.0005},
            {"threshold": 0.5, "morphology": "none", "minimum_area_m2": 0.0005},
        ],
    )
    assert aggregate["negative_area_fp_per_frame"] == 1.0
