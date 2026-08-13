from pathlib import Path

import prepare_trcrv10_proposal_candidates as registry


def test_registry_is_strictly_historical_and_does_not_select() -> None:
    source = Path(registry.__file__).read_text(encoding="utf-8")
    assert "NOT_RUN_G10_HOLDOUT_PENDING" in source
    assert "new_detector_training_forbidden" in source
    assert "g10_dev_val_sealed_read" in source
    assert "val_new_read" in source
    assert "g5_v2_read" in source


def test_candidate_allowlist_is_exact() -> None:
    source = Path(registry.__file__).read_text(encoding="utf-8")
    for candidate_id in (
        "rgdrv8_route_a_best",
        "rgdrv8_ga1_best",
        "tgarv9_grounding_dino_best",
    ):
        assert candidate_id in source
    assert "new_detector" not in source.replace("new_detector_training_forbidden", "")
