"""Pure validity rule for the SLAM scan-startup gate."""

from __future__ import annotations

import math
from collections.abc import Sequence


def has_usable_scan(*, frame_id: str, ranges: Sequence[float]) -> bool:
    """Accept a physical scan once it has a frame and a non-NaN ray.

    +Inf is a valid physical no-return and must not be mistaken for an absent
    sensor frame.  A completely NaN scan is not usable by SLAM.
    """
    return bool(frame_id.strip()) and bool(ranges) and any(
        not math.isnan(float(value)) for value in ranges
    )
