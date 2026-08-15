from pathlib import Path

import trcrv10_finalize_asset_smoke as smoke


def test_target_semantic_labels_are_explicit_and_complete() -> None:
    assert smoke.TARGET_LABELS == {"plastic_bottle": 1, "metal_can": 2, "paper_litter": 3}


def test_smoke_is_not_misrepresented_as_identifiability_gate() -> None:
    source = Path(smoke.__file__).read_text(encoding="utf-8")
    assert "renderability_and_sensor_chain_only_not_identifiability_gate" in source
    assert "positive smoke does not contain all three target classes" in source
    assert "sensor/odom synchronization gate failed" in source


def test_gazebo_resource_failures_are_hard_errors() -> None:
    source = Path(smoke.__file__).read_text(encoding="utf-8")
    for failure in ("Unable to find file", "Error parsing XML", "Unable to load", "SDF error"):
        assert failure in source
