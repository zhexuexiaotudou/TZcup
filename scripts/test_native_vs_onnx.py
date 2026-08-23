import importlib.util
from pathlib import Path

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "d1_worker_native", ROOT / "scripts" / "d1_export_worker.py"
)
worker = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(worker)

EVAL_SPEC = importlib.util.spec_from_file_location(
    "d1_eval_native", ROOT / "scripts" / "evaluate_d1_development.py"
)
evaluation = importlib.util.module_from_spec(EVAL_SPEC)
assert EVAL_SPEC.loader is not None
EVAL_SPEC.loader.exec_module(evaluation)


def test_dual_detect_native_primary_uses_official_second_head():
    auxiliary = np.zeros((1, 14, 8400), dtype=np.float32)
    primary = np.ones((1, 14, 8400), dtype=np.float32)
    assert worker.dual_detect_primary(([auxiliary, primary], object())) is primary


@pytest.mark.parametrize(
    "payload",
    [np.zeros((1, 14, 8400)), ([np.zeros((1, 14, 8400))], object())],
)
def test_dual_detect_native_primary_rejects_ambiguous_layout(payload):
    with pytest.raises(RuntimeError, match="DualDDetect"):
        worker.dual_detect_primary(payload)


def test_v3_path_guard_blocks_all_named_validation_and_sealed_splits():
    assert set(evaluation.BLOCKED_PATH_TOKENS) == {
        "DEV_VAL",
        "VAL_NEW",
        "G5",
        "G5_V2",
        "SEALED_FINAL",
    }
