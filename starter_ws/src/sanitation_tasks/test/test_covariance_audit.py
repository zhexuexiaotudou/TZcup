from sanitation_tasks.covariance_audit import covariance_summary


def test_covariance_summary_detects_all_zero_bridge_output():
    report = covariance_summary([0.0] * 36)
    assert report["all_zero"]
    assert report["has_zero_diagonal"]


def test_covariance_summary_accepts_nonzero_diagonal():
    values = [0.0] * 9
    values[0], values[4], values[8] = 1.0, 2.0, 3.0
    report = covariance_summary(values)
    assert not report["all_zero"]
    assert not report["has_zero_diagonal"]
    assert not report["unusual"]
