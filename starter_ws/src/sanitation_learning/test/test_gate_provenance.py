from pathlib import Path

import pytest
import yaml

from sanitation_learning.oprv3_online import validate_gate_provenance


ROOT = Path(__file__).resolve().parents[4]


def test_gate_provenance_separates_internal_and_online_without_fake_official_gate():
    path = ROOT / "starter_ws/src/sanitation_learning/config/oprv3_gate_provenance.yaml"
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    validate_gate_provenance(payload)
    kinds = {gate["kind"] for gate in payload["gates"]}
    assert kinds == {"INTERNAL_DIAGNOSTIC_GATE", "ONLINE_PRODUCT_GATE"}
    assert payload["competition_material_audit"]["current_official_primary_source_found"] is False
    assert all(gate["source_type"] != "competition" for gate in payload["gates"])


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
