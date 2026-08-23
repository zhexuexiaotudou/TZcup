from copy import deepcopy
from pathlib import Path

import pytest

from emfj6v3_contract import ContractError, load_yaml, validate_registry


REGISTRY = Path(__file__).resolve().parents[1] / "config" / "existing_model_candidates_v3.yaml"


def test_every_candidate_has_auditable_class_order_source_and_artifact_sha():
    payload = load_yaml(REGISTRY)
    validate_registry(payload)
    for candidate in payload["candidates"]:
        assert candidate["class_order"]
        assert candidate["class_order_source"]
        assert all(len(item["sha256"]) == 64 for item in candidate["files"])


def test_anonymous_class_order_cannot_be_silently_removed():
    payload = load_yaml(REGISTRY)
    broken = deepcopy(payload)
    broken["candidates"][0]["class_order_source"] = ""
    with pytest.raises(ContractError, match="class order source"):
        validate_registry(broken)
