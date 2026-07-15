import math

from sanitation_tasks.transient_analysis import analyze_transient_samples, repeatability_summary


def rows(rate=0.25, count=101, step=0.1):
    output = []
    for index in range(count):
        stamp = index * step
        yaw = rate * stamp
        output.append({
            "stamp_sec": stamp,
            "trial_active": True,
            "cmd_requested_angular": rate,
            "cmd_output_angular": rate,
            "gt_yaw": yaw,
            "gt_angular_z": rate,
            "raw_yaw": yaw,
            "imu_yaw_rate": rate,
            "ekf_yaw": yaw,
        })
    return output


def test_ideal_transient_integrates_actual_output_and_sensors():
    report = analyze_transient_samples(rows(), 0.25)
    assert report["complete"]
    assert math.isclose(report["actual_output_command_integral_rad"], 2.5)
    assert math.isclose(report["ground_truth_yaw_delta_rad"], 2.5)
    assert report["body_yaw_error_deg"] < 1.0e-9
    assert report["steady_state_yaw_rate_gain"] == 1.0


def test_repeatability_reports_spread_without_dropping_trials():
    report = repeatability_summary([
        {"complete": True, "ground_truth_yaw_delta_rad": 1.0},
        {"complete": True, "ground_truth_yaw_delta_rad": 1.1},
    ])
    assert report["count"] == 2
    assert report["standard_deviation"] > 0.0
