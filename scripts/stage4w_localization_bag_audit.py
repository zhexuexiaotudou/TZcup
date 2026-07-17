#!/usr/bin/env python3
"""Compare fused and scan-refined poses with timestamp-matched Gazebo truth."""

import argparse
import bisect
import json
import math
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def stamp_seconds(message):
    stamp = message.header.stamp
    return stamp.sec + stamp.nanosec * 1.0e-9


def pose_sample(message):
    pose = message.pose.pose if hasattr(message.pose, "pose") else message.pose
    covariance = getattr(message.pose, "covariance", None)
    variance = (
        max(covariance[0], covariance[7])
        if covariance is not None and len(covariance) >= 8 else None
    )
    return stamp_seconds(message), pose.position.x, pose.position.y, variance


def gnss_sample(message, origin_latitude_deg=31.2304,
                origin_longitude_deg=121.4737):
    earth_radius_m = 6378137.0
    origin_latitude_rad = math.radians(origin_latitude_deg)
    x = math.radians(message.longitude - origin_longitude_deg) * (
        earth_radius_m * math.cos(origin_latitude_rad)
    )
    y = math.radians(message.latitude - origin_latitude_deg) * earth_radius_m
    variance = max(message.position_covariance[0], message.position_covariance[4])
    return stamp_seconds(message), x, y, variance


def summarize(samples, truths, tolerance=0.05):
    truth_times = [sample[0] for sample in truths]
    errors = []
    timed_errors = []
    for sample in samples:
        stamp, x, y = sample[:3]
        index = bisect.bisect_left(truth_times, stamp)
        candidates = [
            candidate for candidate in (index - 1, index)
            if 0 <= candidate < len(truths)
        ]
        if not candidates:
            continue
        chosen = min(candidates, key=lambda item: abs(truth_times[item] - stamp))
        truth = truths[chosen]
        if abs(truth[0] - stamp) > tolerance:
            continue
        error = math.hypot(x - truth[1], y - truth[2])
        errors.append(error)
        timed_errors.append((stamp, error))
    ordered = sorted(errors)
    if not ordered:
        return {"sample_count": 0, "windows": []}
    p95_index = min(len(ordered) - 1, math.ceil(0.95 * len(ordered)) - 1)
    start = timed_errors[0][0]
    windows = []
    window_start = start
    while window_start <= timed_errors[-1][0]:
        window_values = [
            error for stamp, error in timed_errors
            if window_start <= stamp < window_start + 30.0
        ]
        if window_values:
            windows.append({
                "start_offset_sec": window_start - start,
                "sample_count": len(window_values),
                "rmse_m": math.sqrt(
                    sum(value * value for value in window_values)
                    / len(window_values)
                ),
            })
        window_start += 30.0
    return {
        "sample_count": len(errors),
        "rmse_m": math.sqrt(sum(value * value for value in errors) / len(errors)),
        "p95_m": ordered[p95_index],
        "max_m": ordered[-1],
        "windows": windows,
    }


def simulate_reweights(fused, refined, truths, old_gnss_weight=2500.0,
                       old_scan_weight=400.0):
    """Infer the effective GNSS term and replay alternative fusion weights."""
    refined_times = [sample[0] for sample in refined]
    old_gnss_fraction = old_gnss_weight / (old_gnss_weight + old_scan_weight)
    old_scan_fraction = 1.0 - old_gnss_fraction
    inferred = []
    for fused_sample in fused:
        stamp, fused_x, fused_y = fused_sample[:3]
        index = bisect.bisect_right(refined_times, stamp) - 1
        if index < 0 or stamp - refined_times[index] > 0.5:
            continue
        refined_sample = refined[index]
        gnss_x = (fused_x - old_scan_fraction * refined_sample[1]) / old_gnss_fraction
        gnss_y = (fused_y - old_scan_fraction * refined_sample[2]) / old_gnss_fraction
        inferred.append((stamp, gnss_x, gnss_y, refined_sample[1], refined_sample[2]))
    reports = {}
    for scan_variance in (0.0016, 0.0009, 0.0004, 0.0002):
        scan_weight = 1.0 / scan_variance
        gnss_fraction = old_gnss_weight / (old_gnss_weight + scan_weight)
        replay = [
            (
                stamp,
                gnss_fraction * gnss_x + (1.0 - gnss_fraction) * refined_x,
                gnss_fraction * gnss_y + (1.0 - gnss_fraction) * refined_y,
            )
            for stamp, gnss_x, gnss_y, refined_x, refined_y in inferred
        ]
        reports[str(scan_variance)] = {
            "gnss_fraction": gnss_fraction,
            **summarize(replay, truths),
        }
    return reports


def simulate_gnss_variance_scales(fused, refined, truths,
                                  minimum_scan_variance=0.00025,
                                  maximum_scan_variance=0.0009):
    """Recover each GNSS term from the fused covariance and replay scales."""
    refined_times = [sample[0] for sample in refined]
    inferred = []
    for fused_sample in fused:
        stamp, fused_x, fused_y, fused_variance = fused_sample
        if fused_variance is None or fused_variance <= 0.0:
            continue
        index = bisect.bisect_right(refined_times, stamp) - 1
        if index < 0 or stamp - refined_times[index] > 0.5:
            continue
        refined_sample = refined[index]
        scan_variance = min(
            maximum_scan_variance,
            max(minimum_scan_variance, refined_sample[3]),
        )
        scan_weight = 1.0 / scan_variance
        gnss_weight = 1.0 / fused_variance - scan_weight
        if gnss_weight <= 1e-6:
            continue
        gnss_x = (
            fused_x * (gnss_weight + scan_weight)
            - scan_weight * refined_sample[1]
        ) / gnss_weight
        gnss_y = (
            fused_y * (gnss_weight + scan_weight)
            - scan_weight * refined_sample[2]
        ) / gnss_weight
        inferred.append((
            stamp, gnss_x, gnss_y, 1.0 / gnss_weight,
            refined_sample[1], refined_sample[2], scan_variance,
        ))
    reports = {}
    for scale in (1.0, 1.3, 1.5, 2.0, 3.0, 4.0, 8.0):
        replay = []
        for stamp, gnss_x, gnss_y, gnss_variance, refined_x, refined_y, scan_variance in inferred:
            gnss_weight = 1.0 / (gnss_variance * scale)
            scan_weight = 1.0 / scan_variance
            replay.append((
                stamp,
                (gnss_weight * gnss_x + scan_weight * refined_x)
                / (gnss_weight + scan_weight),
                (gnss_weight * gnss_y + scan_weight * refined_y)
                / (gnss_weight + scan_weight),
            ))
        reports[str(scale)] = summarize(replay, truths)
    return reports


def summarize_variance(samples):
    values = sorted(
        float(sample[3]) for sample in samples
        if len(sample) > 3 and sample[3] is not None
    )
    if not values:
        return {"sample_count": 0}
    return {
        "sample_count": len(values),
        "min_m2": values[0],
        "p50_m2": values[len(values) // 2],
        "max_m2": values[-1],
    }


def summarize_intervals(samples):
    intervals = sorted(
        samples[index][0] - samples[index - 1][0]
        for index in range(1, len(samples))
        if samples[index][0] >= samples[index - 1][0]
    )
    if not intervals:
        return {"sample_count": len(samples)}
    return {
        "sample_count": len(samples),
        "maximum_interval_sec": intervals[-1],
        "intervals_over_0_5_sec": sum(value > 0.5 for value in intervals),
        "intervals_over_1_0_sec": sum(value > 1.0 for value in intervals),
        "uncovered_after_age_limit_sec": {
            str(limit): sum(max(0.0, value - limit) for value in intervals)
            for limit in (0.5, 1.0, 2.0, 5.0, 20.0)
        },
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    selected = {
        "/ground_truth/odom": [],
        "/localization/fused_pose": [],
        "/localization/refined_pose": [],
        "/gnss/fix": [],
    }
    while reader.has_next():
        topic, data, _received_stamp = reader.read_next()
        if topic not in selected:
            continue
        message = deserialize_message(data, get_message(topic_types[topic]))
        sample = gnss_sample(message) if topic == "/gnss/fix" else pose_sample(message)
        selected[topic].append(sample)
    truths = selected["/ground_truth/odom"]
    fused = selected["/localization/fused_pose"]
    refined = selected["/localization/refined_pose"]
    gnss = selected["/gnss/fix"]
    report = {
        "schema_version": 1,
        "bag": str(args.bag),
        "truth_sample_count": len(truths),
        "fused": summarize(fused, truths),
        "fused_variance_below_scan_cap": summarize(
            [sample for sample in fused if sample[3] is not None and sample[3] < 0.0009],
            truths,
        ),
        "fused_variance_at_or_above_scan_cap": summarize(
            [sample for sample in fused if sample[3] is not None and sample[3] >= 0.0009],
            truths,
        ),
        "fused_reported_variance": summarize_variance(fused),
        "refined": summarize(refined, truths),
        "gnss_raw": summarize(gnss, truths),
        "gnss_reported_variance": summarize_variance(gnss),
        "refined_reported_variance": summarize_variance(refined),
        "refined_intervals": summarize_intervals(refined),
        "fusion_reweight_simulation": simulate_reweights(
            fused, refined, truths
        ),
        "gnss_variance_scale_simulation": simulate_gnss_variance_scales(
            fused, refined, truths
        ),
    }
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report, ensure_ascii=False))


if __name__ == "__main__":
    main()
