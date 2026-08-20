from types import SimpleNamespace

import pytest

from sanitation_perception.action_verifier import (
    ActionVerifierConfig,
    ActionVerdict,
    ProductActionVerifier,
)


def config(**overrides):
    values = {
        "actionable_classes": (
            "plastic_bottle", "metal_can", "paper_litter", "leaf_pile", "puddle"
        ),
        "minimum_class_confidence": 0.80,
        "maximum_background_probability": 0.10,
        "reject_background_probability": 0.90,
        "minimum_observations": 3,
        "defer_after_observations": 6,
        "maximum_covariance_trace": 0.03,
        "maximum_map_disagreement_m": 0.15,
        "minimum_view_separation_rad": 0.0,
        "maximum_reobserve_count": 2,
    }
    values.update(overrides)
    return ActionVerifierConfig(**values)


def track(**overrides):
    values = {
        "uuid": "track-1",
        "source_backend": "onnxruntime",
        "class_posterior": {
            "plastic_bottle": 0.95,
            "background": 0.05,
        },
        "class_id": "plastic_bottle",
        "class_confidence": 0.95,
        "observation_count": 3,
        "covariance_trace": 0.01,
        "x_m": 1.0,
        "y_m": 2.0,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def mapped(**overrides):
    values = {
        "map_x_m": 1.01,
        "map_y_m": 2.01,
        "observation_count": 3,
        "view_directions_rad": [0.0, 0.3],
        "reobserve_count": 0,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def test_all_action_checks_are_required_before_acceptance() -> None:
    verifier = ProductActionVerifier(config())
    result = verifier.evaluate(track(), mapped(), depth_valid=True)
    assert result.verdict == ActionVerdict.ACCEPT
    assert all(result.checks.values())


def test_reobservation_is_bounded_then_deferred() -> None:
    verifier = ProductActionVerifier(config())
    weak = track(observation_count=2, class_confidence=0.60)
    assert verifier.evaluate(weak, mapped(observation_count=2), depth_valid=True).verdict == ActionVerdict.OBSERVE_AGAIN
    assert verifier.evaluate(weak, mapped(observation_count=2), depth_valid=True).verdict == ActionVerdict.OBSERVE_AGAIN
    final = verifier.evaluate(weak, mapped(observation_count=2), depth_valid=True)
    assert final.verdict == ActionVerdict.DEFER
    assert final.reobserve_count == 2


def test_background_and_ground_truth_fail_closed() -> None:
    verifier = ProductActionVerifier(config())
    background = track(
        class_posterior={"paper_litter": 0.05, "background": 0.95},
        class_confidence=0.05,
    )
    assert verifier.evaluate(background, mapped(), depth_valid=True).verdict == ActionVerdict.REJECT
    with pytest.raises(ValueError, match="GT control violation"):
        verifier.evaluate(track(source_backend="ground_truth"), mapped(), depth_valid=True)


def test_invalid_depth_or_map_disagreement_cannot_accept() -> None:
    verifier = ProductActionVerifier(config(maximum_reobserve_count=0))
    result = verifier.evaluate(track(), mapped(map_x_m=2.0), depth_valid=False)
    assert result.verdict == ActionVerdict.DEFER
    assert not result.checks["depth_valid"]
    assert not result.checks["map_consistency"]


def test_contract_caps_reobservation_at_two() -> None:
    with pytest.raises(ValueError, match="at most two"):
        config(maximum_reobserve_count=3).validate()


def test_persisted_map_budget_survives_verifier_restart() -> None:
    verifier = ProductActionVerifier(config())
    weak = track(observation_count=2, class_confidence=0.60)
    result = verifier.evaluate(
        weak, mapped(observation_count=2, reobserve_count=2), depth_valid=True
    )
    assert result.verdict == ActionVerdict.DEFER
    assert result.reobserve_count == 2
