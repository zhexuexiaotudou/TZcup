from pathlib import Path

import trcrv10_baseline as v10


def test_v10_semantic_states_are_complete_and_unique() -> None:
    assert len(v10.STATES) == 13
    assert len(set(v10.STATES)) == len(v10.STATES)
    assert "ACTION_VERIFIED" in v10.STATES
    assert v10.STATES.index("ACTION_VERIFIED") < v10.STATES.index("CONFIRMED")


def test_v10_artifact_layout_includes_every_protocol_stage() -> None:
    assert len(v10.REQUIRED_DIRECTORIES) == 22
    assert v10.REQUIRED_DIRECTORIES[0] == "baseline"
    assert v10.REQUIRED_DIRECTORIES[-1] == "final"
    assert {"identifiability", "g10", "holdout_gate", "dev_val", "g5v2"} <= set(v10.REQUIRED_DIRECTORIES)


def test_baseline_freeze_preserves_all_sealed_boundaries() -> None:
    source = Path(v10.__file__).read_text(encoding="utf-8")
    assert '"VAL_NEW_read": False' in source
    assert '"G5_V2_read": False' in source
    assert '"G10_DEV_VAL_SEALED_read": False' in source
    assert '"formal_30_seed_read": False' in source


def test_classified_cannot_bypass_independent_action_verifier() -> None:
    source = Path(v10.__file__).read_text(encoding="utf-8")
    assert '"CLASSIFIED": False' in source
    assert '"ACTION_VERIFIED": "necessary_but_not_sufficient_until_consensus"' in source
    assert "CLASSIFIED never directly enters the scheduler clean path" in source
    assert "ACTION_VERIFIED is required before CONFIRMED" in source


def test_gate_provenance_does_not_collapse_evaluation_units() -> None:
    source = Path(v10.__file__).read_text(encoding="utf-8")
    for unit in (
        "target encounter", "four-class crop", "background crop",
        "confirmed actionable target", "negative-only proposal", "cleaning action",
    ):
        assert unit in source
    assert "must never be collapsed into one accuracy value" in source


def test_unbounded_detector_search_is_explicitly_forbidden() -> None:
    source = Path(v10.__file__).read_text(encoding="utf-8")
    assert "No T4, T5, T6" in source
    assert "unbounded detector architecture search" in source
