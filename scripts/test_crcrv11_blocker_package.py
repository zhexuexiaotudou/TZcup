import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "docs" / "evidence" / "crcrv11"


def _load(name: str) -> dict:
    return json.loads((EVIDENCE / name).read_text(encoding="utf-8"))


def test_crcrv11_blocker_package_is_complete_and_fail_closed():
    required = {
        "PERCEPTION_CRCRV11_FINAL_STATUS.json",
        "PERCEPTION_CRCRV11_FINAL_BLOCKERS.json",
        "PERCEPTION_CRCRV11_EVIDENCE_INDEX.md",
        "PERCEPTION_CRCRV11_MODEL_REGISTRY.json",
        "PERCEPTION_CRCRV11_RELEASE_MANIFEST.json",
        "PERCEPTION_CRCRV11_THIRD_PARTY_NOTICES.md",
        "CLOSE_RANGE_CLASSIFIER_CONTRACT_RECOVERY_V11_REPORT.md",
    }
    assert required <= {path.name for path in EVIDENCE.iterdir()}

    status = _load("PERCEPTION_CRCRV11_FINAL_STATUS.json")
    assert status["stop_condition"] == "B_R1_R2_R3_ALL_FAILED"
    assert status["CLOSE_RANGE_CLASSIFIER_CONTRACT_BLOCKED"] is True
    assert status["MODEL_BLOCKED_INTERNAL"] is True
    assert status["SIMULATION_PRODUCT_COMPLETE"] is False
    assert status["PRODUCT_X86_PERCEPTION_READY"] is False
    assert status["G10_DEV_VAL_SEALED_read"] is False
    assert status["VAL_NEW_read"] is False
    assert status["G5_V2_read"] is False
    assert set(status["downstream"].values()) == {
        "dependency_blocked_not_executed"
    }


def test_crcrv11_registry_and_release_select_nothing():
    registry = _load("PERCEPTION_CRCRV11_MODEL_REGISTRY.json")
    assert registry["selected_product_route"] is None
    assert registry["frozen_product_model"] is None
    assert {row["status"] for row in registry["models"]} == {"failed"}
    assert all(
        row["checkpoint_retained_in_active_repository"] is False
        for row in registry["models"]
    )

    release = _load("PERCEPTION_CRCRV11_RELEASE_MANIFEST.json")
    assert release["release_bundle_created"] is False
    assert release["MODEL_FREEZE_X86_created"] is False
    assert release["RELEASE_BUNDLE_PASS"] is False


def test_crcrv11_forbidden_actions_remain_locked():
    blockers = _load("PERCEPTION_CRCRV11_FINAL_BLOCKERS.json")
    assert blockers["forbidden_next_actions"] == [
        "classifier R4/R5",
        "new detector search",
        "sealed-data tuning",
        "lowering product gates",
    ]
    assert max(
        row["evidence"].get("R1_combined_macro_f1", 0.0)
        for row in blockers["primary_blockers"]
    ) == 0.6075
    target = blockers["primary_blockers"][0]["evidence"]
    assert target["R2_combined_macro_f1"] == 0.6311
    assert target["R3_combined_macro_f1"] == 0.4561
