"""Pure Stage4T angular transient metrics shared by ROS runners and CI."""

import math
import statistics

from .evaluation import normalize_angle


def integrate(times, values):
    total = 0.0
    for index in range(1, len(times)):
        dt = times[index] - times[index - 1]
        if 0.0 < dt <= 0.25:
            total += 0.5 * (values[index] + values[index - 1]) * dt
    return total


def unwrap(values):
    if not values:
        return []
    output = [values[0]]
    for value in values[1:]:
        output.append(output[-1] + normalize_angle(value - output[-1]))
    return output


def first_crossing(times, values, threshold):
    for stamp, value in zip(times, values):
        if value >= threshold:
            return stamp
    return None


def analyze_transient_samples(rows, target_rate):
    rows = [row for row in rows if row.get("trial_active", True)]
    if len(rows) < 3 or abs(target_rate) < 1.0e-9:
        return {"complete": False, "sample_count": len(rows)}
    sign = 1.0 if target_rate > 0.0 else -1.0
    times = [float(row["stamp_sec"]) for row in rows]
    elapsed = [stamp - times[0] for stamp in times]
    requested = [sign * float(row["cmd_requested_angular"]) for row in rows]
    output = [sign * float(row["cmd_output_angular"]) for row in rows]
    body = [sign * float(row["gt_angular_z"]) for row in rows]
    imu_rate = [sign * float(row["imu_yaw_rate"]) for row in rows]
    raw = unwrap([float(row["raw_yaw"]) for row in rows])
    truth = unwrap([float(row["gt_yaw"]) for row in rows])
    ekf = unwrap([float(row["ekf_yaw"]) for row in rows])
    request_integral = integrate(times, [float(row["cmd_requested_angular"]) for row in rows])
    output_integral = integrate(times, [float(row["cmd_output_angular"]) for row in rows])
    imu_integral = integrate(times, [float(row["imu_yaw_rate"]) for row in rows])
    requested_on = first_crossing(elapsed, requested, abs(target_rate) * 0.05)
    output_on = first_crossing(elapsed, output, abs(target_rate) * 0.05)
    body_on = first_crossing(elapsed, body, abs(target_rate) * 0.05)
    rise_10 = first_crossing(elapsed, body, abs(target_rate) * 0.10)
    rise_90 = first_crossing(elapsed, body, abs(target_rate) * 0.90)
    tolerance = max(0.01, abs(target_rate) * 0.05)
    settling = None
    for index, stamp in enumerate(elapsed):
        if all(abs(value - abs(target_rate)) <= tolerance for value in body[index:]):
            settling = stamp
            break
    tail_start = max(0, int(len(body) * 0.80))
    steady_body = statistics.fmean(body[tail_start:])
    integral_tracking_error = integrate(
        times, [abs(out - measured) for out, measured in zip(output, body)]
    )
    truth_delta = truth[-1] - truth[0]
    return {
        "complete": True,
        "sample_count": len(rows),
        "duration_sec": elapsed[-1],
        "target_yaw_rate_rad_s": target_rate,
        "requested_command_integral_rad": request_integral,
        "actual_output_command_integral_rad": output_integral,
        "ground_truth_yaw_delta_rad": truth_delta,
        "raw_odom_yaw_delta_rad": raw[-1] - raw[0],
        "imu_integrated_yaw_rad": imu_integral,
        "ekf_yaw_delta_rad": ekf[-1] - ekf[0],
        "body_yaw_error_deg": math.degrees(abs(truth_delta - request_integral)),
        "request_to_output_delay_sec": None if requested_on is None or output_on is None else max(0.0, output_on - requested_on),
        "output_to_body_delay_sec": None if output_on is None or body_on is None else max(0.0, body_on - output_on),
        "rise_time_sec": None if rise_10 is None or rise_90 is None else max(0.0, rise_90 - rise_10),
        "settling_time_sec": settling,
        "overshoot_ratio": max(0.0, max(body) / abs(target_rate) - 1.0),
        "steady_state_yaw_rate_gain": steady_body / abs(target_rate),
        "integral_tracking_error_rad": integral_tracking_error,
    }


def repeatability_summary(trials, field="ground_truth_yaw_delta_rad"):
    values = [float(trial[field]) for trial in trials if trial.get("complete") and trial.get(field) is not None]
    if len(values) < 2:
        return {"count": len(values), "mean": values[0] if values else None, "standard_deviation": None, "coefficient_of_variation": None}
    mean = statistics.fmean(values)
    deviation = statistics.stdev(values)
    return {
        "count": len(values),
        "mean": mean,
        "standard_deviation": deviation,
        "coefficient_of_variation": deviation / abs(mean) if abs(mean) > 1.0e-12 else None,
    }
