from copy import deepcopy
from pathlib import Path

import pytest

from emfj6v3_contract import ContractError, load_yaml, validate_registry


REGISTRY = Path(__file__).resolve().parents[1] / "config" / "existing_model_candidates_v3.yaml"


def test_existing_model_inventory_is_fail_closed_and_within_caps():
    payload = load_yaml(REGISTRY)
    assert validate_registry(payload) == {"detector": 2, "classifier": 2, "area": 1}
    assert payload["states"]["EMF_EXISTING_MODEL_INVENTORY_READY"] is False
    assert payload["states"]["EMF_EXISTING_MODEL_SCREENING_COMPLETE"] is False
    assert payload["sealed_access_allowed"] is False


def test_candidate_cap_is_enforced():
    payload = load_yaml(REGISTRY)
    candidate = next(item for item in payload["candidates"] if item["role"] == "detector")
    expanded = deepcopy(payload)
    expanded["candidates"] = [
        {**deepcopy(candidate), "model_id": f"detector_{index}"}
        for index in range(13)
    ]
    with pytest.raises(ContractError, match="candidate cap exceeded"):
        validate_registry(expanded)
