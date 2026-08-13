from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_tgarv9_finalizer_is_fail_closed_and_preserves_seals() -> None:
    source = (ROOT / "scripts/finalize_tgarv9.py").read_text(encoding="utf-8")
    assert '"TGARV9_ALL_ROUTES_EXHAUSTED": True' in source
    assert '"MODEL_BLOCKED_INTERNAL": True' in source
    assert '"SIMULATION_PRODUCT_COMPLETE": False' in source
    assert '"VAL_NEW_read": False' in source
    assert '"G5_V2_read": False' in source
    assert "T1 did not fail" in source
    assert "T2 did not fail" in source
    assert "T3 did not fail" in source
    assert "deployability pre-screen is forbidden before a HOLDOUT pass" in source


def test_tgarv9_finalizer_emits_complete_contract() -> None:
    source = (ROOT / "scripts/finalize_tgarv9.py").read_text(encoding="utf-8")
    for name in (
        "PERCEPTION_TGARV9_FINAL_STATUS.json",
        "PERCEPTION_TGARV9_FINAL_BLOCKERS.json",
        "PERCEPTION_TGARV9_EVIDENCE_INDEX.md",
        "PERCEPTION_TGARV9_MODEL_REGISTRY.json",
        "PERCEPTION_TGARV9_RELEASE_MANIFEST.json",
        "PERCEPTION_TGARV9_THIRD_PARTY_NOTICES.md",
        "TEMPORAL_GEOMETRY_ARCHITECTURE_RECOVERY_V9_REPORT.md",
    ):
        assert name in source
    for question in range(1, 30):
        assert f"{question}. " in source
