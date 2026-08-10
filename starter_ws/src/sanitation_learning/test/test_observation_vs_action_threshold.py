import pytest

from sanitation_learning.oprv3_online import ThresholdPolicy


def test_low_confidence_observations_never_trigger_clean_action():
    policy = ThresholdPolicy(0.20, 0.55, 0.80, 3)
    assert policy.disposition(0.30, 20) == "non_actionable_observation"
    assert policy.disposition(0.70, 3) == "confirmed_track_not_cleanable"
    assert policy.disposition(0.90, 2) == "non_actionable_observation"
    assert policy.disposition(0.90, 3) == "clean_action_eligible"


def test_thresholds_must_be_strictly_ordered():
    with pytest.raises(ValueError, match="strictly ordered"):
        ThresholdPolicy(0.50, 0.50, 0.80, 3)
