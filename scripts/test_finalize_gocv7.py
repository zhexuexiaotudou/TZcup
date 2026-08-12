from pathlib import Path


SOURCE = Path(__file__).with_name("finalize_gocv7.py").read_text(encoding="utf-8")


def test_gocv7_finalizer_emits_mandatory_blocked_outputs():
    for name in (
        "PERCEPTION_GOCV7_FINAL_STATUS.json",
        "PERCEPTION_GOCV7_FINAL_BLOCKERS.json",
        "PERCEPTION_GOCV7_EVIDENCE_INDEX.md",
        "PERCEPTION_GOCV7_MODEL_REGISTRY.json",
        "PERCEPTION_GOCV7_RELEASE_MANIFEST.json",
        "GAZEBO_ONLINE_CLOSURE_V7_REPORT.md",
    ):
        assert name in SOURCE


def test_gocv7_finalizer_preserves_locked_boundaries():
    assert 'selection.get("existing_24_mission_read_before_selection_freeze") is not False' in SOURCE
    assert '"G5_V2_read": False' in SOURCE
    assert '"SIMULATION_PRODUCT_COMPLETE": False' in SOURCE
    assert '"PR_READY_ALLOWED": False' in SOURCE
