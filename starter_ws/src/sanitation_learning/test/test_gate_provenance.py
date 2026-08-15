from pathlib import Path

import pytest
import yaml

from sanitation_learning.oprv3_online import validate_gate_provenance


ROOT = Path(__file__).resolve().parents[4]


def test_gate_provenance_separates_verified_official_internal_and_online_gates():
    path = ROOT / "starter_ws/src/sanitation_learning/config/oprv3_gate_provenance.yaml"
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    validate_gate_provenance(payload)
    kinds = {gate["kind"] for gate in payload["gates"]}
    assert kinds == {
        "OFFICIAL_GATE",
        "INTERNAL_DIAGNOSTIC_GATE",
        "ONLINE_PRODUCT_GATE",
    }
    audit = payload["competition_material_audit"]
    assert payload["frozen_before_moving_model_measurement"] is False
    assert payload["online_product_gates_frozen_before_moving_model_measurement"] is True
    assert payload["competition_mapping_added_after_development_measurement"] is True
    assert audit["current_official_primary_source_found"] is True
    assert audit["verified_primary_source"]["problem_id"] == "DG-202604"
    official = [gate for gate in payload["gates"] if gate["kind"] == "OFFICIAL_GATE"]
    assert {gate["metric"] for gate in official} == {
        "object_level_precision",
        "object_level_recall",
        "object_level_f1",
    }
    assert all(gate["verified"] is True for gate in official)
    assert all(gate["threshold"] == 0.95 for gate in official)


def test_unverified_gate_cannot_be_labeled_official():
    payload = {
        "schema_version": 1,
        "gates": [{
            "id": "invented", "kind": "OFFICIAL_GATE", "source_type": "competition",
            "source_paths": ["missing-primary-source"], "verified": False,
        }],
    }
    with pytest.raises(ValueError, match="unverified"):
        validate_gate_provenance(payload)
