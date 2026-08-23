from pathlib import Path

from emfj6v3_contract import load_yaml, validate_registry


def test_v3_caps_are_frozen():
    registry = Path(__file__).resolve().parents[1] / "config" / "existing_model_candidates_v3.yaml"
    payload = load_yaml(registry)
    assert payload["candidate_limits"] == {"detector": 12, "classifier": 6, "area": 3}
    validate_registry(payload)
