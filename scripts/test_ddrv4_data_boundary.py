import json
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))

from sanitation_learning.ddrv4_boundary import DDRV4BoundaryError, validate_data_boundary


BOUNDARY = (
    ROOT
    / "artifacts/detector_data_recovery_v4_20260811T134117Z/baseline/DATA_BOUNDARY.json"
)


def test_committed_ddrv4_data_boundary_is_fail_closed():
    payload = json.loads(BOUNDARY.read_text(encoding="utf-8"))
    validate_data_boundary(payload)
    assert payload["G5_STATUS"] == "CONSUMED_FINAL"
    assert payload["G5_V2_STATUS"] == "SEALED_NOT_OPENED"


def test_boundary_rejects_tuning_reenable():
    payload = json.loads(BOUNDARY.read_text(encoding="utf-8"))
    payload["G6_CAN_BE_USED_FOR_NEW_ROUTE_TUNING"] = True
    with pytest.raises(DDRV4BoundaryError, match="G6_CAN_BE_USED"):
        validate_data_boundary(payload)
