"""Pure validity rule for the SLAM scan-startup gate."""

from __future__ import annotations

import math
from collections.abc import Sequence


def scan_rejection_reason(*, frame_id: str, ranges: Sequence[float]) -> str:
    """Return the fail-closed reason, or an empty string for a usable scan."""
    if not frame_id.strip():
        return "empty_frame"
    if not ranges:
        return "empty_ranges"
    if not any(not math.isnan(float(value)) for value in ranges):
        return "all_nan"
    return ""


def has_usable_scan(*, frame_id: str, ranges: Sequence[float]) -> bool:
    """Accept a physical scan once it has a frame and a non-NaN ray.

    +Inf is a valid physical no-return and must not be mistaken for an absent
    sensor frame.  A completely NaN scan is not usable by SLAM.
    """
    return not scan_rejection_reason(frame_id=frame_id, ranges=ranges)
