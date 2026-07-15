"""Pure functions used by Stage4R evaluation nodes and unit tests."""

import bisect
import math
import statistics


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def percentile(values, probability):
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * probability
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def summarize(values):
    if not values:
        return {"rmse": None, "p50": None, "p95": None, "max": None}
    return {
        "rmse": math.sqrt(statistics.fmean(value * value for value in values)),
        "p50": percentile(values, 0.50),
        "p95": percentile(values, 0.95),
        "max": max(values),
    }


def assert_comparable_frames(estimate_frame, truth_frame, allowed_pairs=None):
    """Reject the exact Stage3 error: direct map-vs-odom subtraction.

    Different frames are accepted only when the caller names an explicit,
    pre-validated alignment pair (for Stage4R this is map <-> map_gt).
    """
    if estimate_frame == truth_frame:
        return
    allowed_pairs = allowed_pairs or set()
    if (estimate_frame, truth_frame) not in allowed_pairs:
        raise ValueError(
            f"incomparable frames: {estimate_frame!r} vs {truth_frame!r}; "
            "an explicit transform/alignment is required"
        )


def synchronize_samples(estimates, truths, tolerance_sec):
    """Nearest-neighbour timestamp pairing without sample reuse."""
    truth_times = [sample[0] for sample in truths]
    pairs = []
    used = set()
    dropped = 0
    for estimate in estimates:
        index = bisect.bisect_left(truth_times, estimate[0])
        candidates = [i for i in (index - 1, index) if 0 <= i < len(truths)]
        candidates = [i for i in candidates if i not in used]
        if not candidates:
            dropped += 1
            continue
        chosen = min(candidates, key=lambda i: abs(truth_times[i] - estimate[0]))
        error = abs(truth_times[chosen] - estimate[0])
        if error > tolerance_sec:
            dropped += 1
            continue
        used.add(chosen)
        pairs.append((estimate, truths[chosen], error))
    return pairs, dropped
