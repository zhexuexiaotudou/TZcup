from copy import deepcopy
import json
from pathlib import Path

import pytest

from emfj6v3_contract import ContractError, build_inventory, load_yaml, validate_registry


REGISTRY = Path(__file__).resolve().parents[1] / "config" / "existing_model_candidates_v3.yaml"
INVENTORY = Path(__file__).resolve().parents[1] / "docs" / "evidence" / "emfj6v3" / "EMF_EXISTING_MODEL_INVENTORY.json"


def test_existing_model_inventory_is_fail_closed_and_within_caps():
    payload = load_yaml(REGISTRY)
    counts = validate_registry(payload)
    assert counts == {"detector": 6, "classifier": 6, "area": 3}
    assert payload["states"]["EMF_EXISTING_MODEL_INVENTORY_READY"] is True
    assert payload["states"]["EMF_EXISTING_MODEL_SCREENING_COMPLETE"] is False
    assert payload["sealed_access_allowed"] is False
    assert payload["discovery_exclusions"]


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


def test_inventory_cannot_be_ready_before_explicit_freeze():
    payload = load_yaml(REGISTRY)
    inventory = build_inventory(payload, REGISTRY)
    assert inventory["EMF_EXISTING_MODEL_INVENTORY_READY"] is True
    assert inventory["EMF_EXISTING_MODEL_SCREENING_COMPLETE"] is False
    assert inventory["registry_path"] == "config/existing_model_candidates_v3.yaml"
    unfrozen = deepcopy(payload)
    unfrozen["inventory_frozen"] = False
    unfrozen["discovery_completed_at"] = None
    unfrozen["discovery_status"] = {"detector": False, "classifier": False, "area": False}
    unfrozen["states"]["EMF_EXISTING_MODEL_INVENTORY_READY"] = False
    assert build_inventory(unfrozen, REGISTRY)["EMF_EXISTING_MODEL_INVENTORY_READY"] is False


def test_inventory_freeze_requires_all_discovery_roles():
    payload = load_yaml(REGISTRY)
    invalid = deepcopy(payload)
    invalid["discovery_status"]["area"] = False
    with pytest.raises(ContractError, match="all discovery roles"):
        validate_registry(invalid)


def test_tracked_inventory_exactly_matches_frozen_registry():
    expected = build_inventory(load_yaml(REGISTRY), REGISTRY)
    actual = json.loads(INVENTORY.read_text(encoding="utf-8"))
    assert actual == expected
