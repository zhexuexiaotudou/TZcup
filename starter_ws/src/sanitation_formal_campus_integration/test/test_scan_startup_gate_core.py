import math

from sanitation_formal_campus_integration.scan_startup_gate_core import has_usable_scan


def test_scan_startup_gate_requires_a_frame_and_a_non_nan_ray():
    assert not has_usable_scan(frame_id="", ranges=[1.0])
    assert not has_usable_scan(frame_id="laser", ranges=[])
    assert not has_usable_scan(frame_id="laser", ranges=[math.nan, math.nan])
    assert has_usable_scan(frame_id="laser", ranges=[math.nan, math.inf])
    assert has_usable_scan(frame_id="laser", ranges=[math.nan, 2.0])
