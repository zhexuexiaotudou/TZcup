import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))

from sanitation_learning.g4_data import DISCRETE_NAMES
from sanitation_learning.g4_direct_fcos import X3_ARCHITECTURE, X3_WEIGHT_SPEC


def test_x3_hypothesis_changes_failed_handoff_without_relaxing_gates():
    payload = json.loads(
        (
            ROOT
            / "artifacts/perception_product_20260809T151411Z/x3/X3_HYPOTHESIS.json"
        ).read_text(encoding="utf-8")
    )
    assert payload["route_count_after_start"] == payload["maximum_route_count"] == 3
    assert payload["x3_change"]["removed_failure_surface"] == "No proposal-crop classifier handoff"
    assert "no gate relaxation" in payload["unchanged_conditions"]
    assert payload["G5_SEALED_FINAL_read"] is False
    assert payload["legacy_G4_D6_read"] is False


def test_x3_model_contract_is_direct_three_class_official_fcos():
    assert DISCRETE_NAMES == ("plastic_bottle", "metal_can", "paper_litter")
    assert X3_ARCHITECTURE.endswith("direct_3class")
    assert X3_WEIGHT_SPEC == "fcos_resnet50_fpn_coco"


def test_x3_failure_exhausts_routes_without_opening_g5():
    payload = json.loads(
        (
            ROOT
            / "artifacts/perception_product_20260809T151411Z/x3/X3_STATIC_FAILURE.json"
        ).read_text(encoding="utf-8")
    )
    assert payload["decision"] == "FAILED_STATIC_FULL_PIPELINE"
    assert payload["route_count"] == 3
    assert payload["additional_model_routes_allowed"] is False
    assert payload["MODEL_BLOCKED_INTERNAL"] is True
    assert payload["PRODUCT_X86_PERCEPTION_READY"] is False
    assert payload["G5_SEALED_FINAL_read"] is False
