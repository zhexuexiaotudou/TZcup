from sanitation_tasks.localization_metrics import trial_completion_reasons


def valid_trial():
    return {
        "sample_count": 10,
        "estimate_sample_count": 10,
        "truth_sample_count": 10,
        "evaluator_exit_code": 0,
        "navigation_exit_code": 0,
        "navigation": {"success": True},
        "map_relative_localization_error": {"xy_m": {"rmse": 0.04}},
        "particle_filter": {
            "particle_instrumentation_required": True,
            "particle_instrumentation_pass": True,
        },
    }


def test_valid_trial_is_complete():
    assert trial_completion_reasons(valid_trial()) == []


def test_zero_sample_report_is_not_complete():
    trial = valid_trial()
    trial.update(sample_count=0, estimate_sample_count=0)
    reasons = trial_completion_reasons(trial)
    assert "no_synchronized_localization_samples" in reasons
    assert "no_estimate_samples" in reasons


def test_navigation_and_particle_failures_are_not_complete():
    trial = valid_trial()
    trial["navigation"] = {"success": False}
    trial["navigation_exit_code"] = 124
    trial["particle_filter"]["particle_instrumentation_pass"] = False
    reasons = trial_completion_reasons(trial)
    assert "navigation_nonzero_exit" in reasons
    assert "navigation_not_completed" in reasons
    assert "particle_instrumentation_invalid" in reasons
