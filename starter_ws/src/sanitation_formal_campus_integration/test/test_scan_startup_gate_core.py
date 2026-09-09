import math

from sanitation_formal_campus_integration.scan_startup_gate_core import (
    has_usable_scan,
    scan_rejection_reason,
)


def test_scan_startup_gate_requires_a_frame_and_a_non_nan_ray():
    assert not has_usable_scan(frame_id="", ranges=[1.0])
    assert not has_usable_scan(frame_id="laser", ranges=[])
    assert not has_usable_scan(frame_id="laser", ranges=[math.nan, math.nan])
    assert has_usable_scan(frame_id="laser", ranges=[math.nan, math.inf])
    assert has_usable_scan(frame_id="laser", ranges=[math.nan, 2.0])


def test_scan_startup_gate_reports_the_exact_fail_closed_reason():
    assert scan_rejection_reason(frame_id="", ranges=[1.0]) == "empty_frame"
    assert scan_rejection_reason(frame_id="laser", ranges=[]) == "empty_ranges"
    assert scan_rejection_reason(frame_id="laser", ranges=[math.nan]) == "all_nan"
    assert scan_rejection_reason(frame_id="laser", ranges=[math.inf]) == ""
