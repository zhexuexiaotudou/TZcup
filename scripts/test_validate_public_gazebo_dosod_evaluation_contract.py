from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest


HERE = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("contract", HERE / "validate_public_gazebo_dosod_evaluation_contract.py")
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC and SPEC.loader
SPEC.loader.exec_module(MODULE)


def test_canonical_public_evaluation_contract_is_explicitly_draft_and_blocked() -> None:
    assert MODULE.load_contract()["status"] == "DRAFT_BLOCKED_UNTIL_PUBLIC_500_PLUS_100_EVIDENCE"


@pytest.mark.parametrize("path", ["classes", "ground_truth.formal_w1_w5_enabled", "postprocess.official_hobot_top_k", "tensor.operation_order", "tensor.square_padding", "tensor.official_adapter_required", "deadlines_sec.mobile_scene_generator"])
def test_contract_rejects_any_frozen_field_drift(tmp_path: Path, path: str) -> None:
    value = MODULE.load_contract()
    target = value
    parts = path.split(".")
    for part in parts[:-1]:
        target = target[part]
    target[parts[-1]] = (not target[parts[-1]]) if isinstance(target[parts[-1]], bool) else 1
    candidate = tmp_path / "contract.json"
    candidate.write_text(json.dumps(value), encoding="utf-8")
    with pytest.raises(ValueError, match="invalid_or_not_admissible"):
        MODULE.load_contract(candidate)
