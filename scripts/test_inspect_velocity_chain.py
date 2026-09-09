import math

from inspect_velocity_chain import summarize


def test_summarize_uses_absolute_values_and_linear_percentiles():
    report = summarize([0.0, -1.0, 2.0, -3.0, 4.0])

    assert report["count"] == 5
    assert report["mean"] == 2.0
    assert report["p50"] == 2.0
    assert report["p95"] == 3.8
    assert report["max"] == 4.0


def test_summarize_omits_non_finite_values_and_handles_no_samples():
    report = summarize([math.nan, math.inf, -math.inf])

    assert report == {
        "count": 0,
        "mean": None,
        "p50": None,
        "p95": None,
        "max": None,
    }
