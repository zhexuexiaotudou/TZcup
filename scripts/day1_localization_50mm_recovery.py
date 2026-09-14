#!/usr/bin/env python3
"""Truth-isolated, causal recovery candidate for the sealed localization run.

The candidate low-pass filters only the online ``map -> odom`` TF stream and
composes it with the original ``odom -> base_footprint`` TF. Ground-truth poses
from the sealed focus result are used only after filtering, for scoring.
"""

from __future__ import annotations

import argparse
import bisect
import csv
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import statistics
from typing import Iterable, Sequence


DEFAULT_MCAP_SHA256 = (
    "8e64c0b7dd6a29247ad0d2f61ac719ba7f8c4f6b1957edf8bffc88aa38308047"
)
DEFAULT_TAU_SEC = 1.5
DEFAULT_MAX_DT_SEC = 0.1
DEFAULT_SENSITIVITY_TAU_SEC = (0.75, 1.0, 1.25, 1.5, 2.0, 3.0)
STRICT_LIMIT_M = 0.050


@dataclass(frozen=True)
class Pose2D:
    stamp_sec: float
    x_m: float
    y_m: float
    yaw_rad: float


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def pose_from_transform(transform) -> Pose2D:
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    yaw = math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
    )
    stamp = transform.header.stamp
    return Pose2D(
        stamp_sec=stamp.sec + stamp.nanosec * 1e-9,
        x_m=float(translation.x),
        y_m=float(translation.y),
        yaw_rad=yaw,
    )


def validate_pose_stream(samples: Sequence[Pose2D], name: str) -> list[Pose2D]:
    if not samples:
        raise ValueError(f"missing pose stream: {name}")
    deduplicated: list[Pose2D] = []
    for sample in samples:
        if not all(
            math.isfinite(value)
            for value in (sample.stamp_sec, sample.x_m, sample.y_m, sample.yaw_rad)
        ):
            raise ValueError(f"non-finite pose in {name}")
        if deduplicated:
            previous = deduplicated[-1]
            if sample.stamp_sec < previous.stamp_sec:
                raise ValueError(f"non-monotonic pose stream: {name}")
            if sample.stamp_sec == previous.stamp_sec:
                if sample != previous:
                    raise ValueError(f"conflicting pose at one timestamp: {name}")
                continue
        deduplicated.append(sample)
    return deduplicated


def causal_lowpass(
    samples: Sequence[Pose2D],
    tau_sec: float,
    max_dt_sec: float = DEFAULT_MAX_DT_SEC,
) -> list[Pose2D]:
    """Low-pass a pose stream using current and past samples only."""
    if tau_sec <= 0.0:
        raise ValueError("tau_sec must be positive")
    if max_dt_sec <= 0.0:
        raise ValueError("max_dt_sec must be positive")
    validated = validate_pose_stream(samples, "causal_lowpass_input")
    state = validated[0]
    output = [state]
    for sample in validated[1:]:
        dt = min(max(sample.stamp_sec - state.stamp_sec, 0.0), max_dt_sec)
        alpha = 1.0 - math.exp(-dt / tau_sec)
        yaw_delta = normalize_angle(sample.yaw_rad - state.yaw_rad)
        state = Pose2D(
            stamp_sec=sample.stamp_sec,
            x_m=state.x_m + alpha * (sample.x_m - state.x_m),
            y_m=state.y_m + alpha * (sample.y_m - state.y_m),
            yaw_rad=normalize_angle(state.yaw_rad + alpha * yaw_delta),
        )
        output.append(state)
    return output


def sample_at_or_before(samples: Sequence[Pose2D], stamp_sec: float) -> Pose2D | None:
    """Return the latest causal sample at or before ``stamp_sec``."""
    stamps = [sample.stamp_sec for sample in samples]
    index = bisect.bisect_right(stamps, stamp_sec) - 1
    return None if index < 0 else samples[index]


def compose_map_pose(map_odom: Pose2D, odom_base: Pose2D) -> tuple[float, float]:
    cosine = math.cos(map_odom.yaw_rad)
    sine = math.sin(map_odom.yaw_rad)
    return (
        map_odom.x_m + cosine * odom_base.x_m - sine * odom_base.y_m,
        map_odom.y_m + sine * odom_base.x_m + cosine * odom_base.y_m,
    )


def summarize_errors(errors_m: Iterable[float]) -> dict[str, float | int | None]:
    values = list(errors_m)
    if not values:
        return {"samples": 0, "rmse_m": None, "p95_m": None, "max_m": None}
    ordered = sorted(values)
    rank = (len(ordered) - 1) * 0.95
    lower = int(rank)
    upper = min(lower + 1, len(ordered) - 1)
    p95 = ordered[lower] + (ordered[upper] - ordered[lower]) * (rank - lower)
    return {
        "samples": len(values),
        "rmse_m": math.sqrt(sum(value * value for value in values) / len(values)),
        "p95_m": p95,
        "max_m": max(values),
    }


def metrics_with_mm(
    rows: Sequence[dict[str, float | int | None]],
) -> dict[str, float | int | None]:
    errors = [float(row["error_m"]) for row in rows]
    result = summarize_errors(errors)
    return {
        "samples": result["samples"],
        "rmse_m": result["rmse_m"],
        "p95_m": result["p95_m"],
        "max_m": result["max_m"],
        "rmse_mm": None if result["rmse_m"] is None else result["rmse_m"] * 1000.0,
        "p95_mm": None if result["p95_m"] is None else result["p95_m"] * 1000.0,
        "max_mm": None if result["max_m"] is None else result["max_m"] * 1000.0,
    }


def lagged_correlation(
    values: Sequence[float],
    stamps: Sequence[float],
    lag_sec: float,
) -> float | None:
    pairs: list[tuple[float, float]] = []
    for index, stamp in enumerate(stamps):
        target = stamp + lag_sec
        other_index = bisect.bisect_left(stamps, target)
        if other_index >= len(stamps):
            continue
        if abs(stamps[other_index] - target) <= 0.011:
            pairs.append((values[index], values[other_index]))
    if len(pairs) < 3:
        return None
    first = [pair[0] for pair in pairs]
    second = [pair[1] for pair in pairs]
    first_mean = statistics.fmean(first)
    second_mean = statistics.fmean(second)
    numerator = sum(
        (left - first_mean) * (right - second_mean) for left, right in pairs
    )
    first_energy = sum((value - first_mean) ** 2 for value in first)
    second_energy = sum((value - second_mean) ** 2 for value in second)
    denominator = math.sqrt(first_energy * second_energy)
    return None if denominator == 0.0 else numerator / denominator


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_focus_result(path: Path) -> dict:
    result = json.loads(path.read_text(encoding="utf-8"))
    samples = result.get("paired_samples")
    if not isinstance(samples, list) or not samples:
        raise ValueError("focus result has no paired_samples")
    stamps = [float(sample["sim_s"]) for sample in samples]
    if any(current <= previous for previous, current in zip(stamps, stamps[1:])):
        raise ValueError("focus paired_samples are not strictly time ordered")
    expected_accuracy = result.get("accuracy", {})
    errors = [
        math.hypot(
            float(sample["navigation_map_xy"][0]) - float(sample["gt_map_xy"][0]),
            float(sample["navigation_map_xy"][1]) - float(sample["gt_map_xy"][1]),
        )
        for sample in samples
    ]
    recomputed = summarize_errors(errors)
    for key in ("samples", "rmse_m", "p95_m", "max_m"):
        expected = expected_accuracy.get(key)
        actual = recomputed[key]
        if expected is None or actual is None:
            raise ValueError(f"focus result is missing accuracy.{key}")
        if not math.isclose(float(expected), float(actual), rel_tol=0.0, abs_tol=1e-12):
            raise ValueError(
                f"focus accuracy.{key} does not match paired_samples: "
                f"{expected!r} != {actual!r}"
            )
    return result


def decode_tf_and_odom(
    mcap_path: Path,
) -> tuple[list[Pose2D], list[Pose2D], list[tuple[float, float, float]]]:
    try:
        from mcap.reader import make_reader
        from mcap_ros2.decoder import DecoderFactory
    except ImportError as error:
        raise RuntimeError(
            "mcap and mcap-ros2-support are required for this analysis"
        ) from error

    map_odom: list[Pose2D] = []
    odom_base: list[Pose2D] = []
    odom_velocity: list[tuple[float, float, float]] = []
    with mcap_path.open("rb") as stream:
        reader = make_reader(
            stream,
            decoder_factories=[DecoderFactory()],
            validate_crcs=True,
        )
        if reader.get_summary() is None:
            raise ValueError("MCAP is not sealed: summary is missing")
        for _, channel, _, message in reader.iter_decoded_messages(
            topics=["/tf", "/odom"]
        ):
            if channel.topic == "/odom":
                stamp = message.header.stamp
                odom_velocity.append(
                    (
                        stamp.sec + stamp.nanosec * 1e-9,
                        float(message.twist.twist.linear.x),
                        float(message.twist.twist.angular.z),
                    )
                )
                continue
            for transform in message.transforms:
                parent = transform.header.frame_id.lstrip("/")
                child = transform.child_frame_id.lstrip("/")
                if parent == "map" and child == "odom":
                    map_odom.append(pose_from_transform(transform))
                elif parent == "odom" and child == "base_footprint":
                    odom_base.append(pose_from_transform(transform))
    return (
        validate_pose_stream(map_odom, "map->odom"),
        validate_pose_stream(odom_base, "odom->base_footprint"),
        odom_velocity,
    )


def motion_class(vx_mps: float, wz_radps: float) -> str:
    if abs(wz_radps) > 0.05:
        return "turning"
    if abs(vx_mps) <= 0.05:
        return "stationary"
    return "straight"


def velocity_at_or_before(
    samples: Sequence[tuple[float, float, float]], stamp_sec: float
) -> tuple[float, float, float] | None:
    stamps = [sample[0] for sample in samples]
    index = bisect.bisect_right(stamps, stamp_sec) - 1
    return None if index < 0 else samples[index]


def build_candidate_rows(
    focus_result: dict,
    map_odom_filtered: Sequence[Pose2D],
    odom_base: Sequence[Pose2D],
    odom_velocity: Sequence[tuple[float, float, float]],
) -> list[dict[str, float | int | str | None]]:
    rows: list[dict[str, float | int | str | None]] = []
    for index, sample in enumerate(focus_result["paired_samples"]):
        stamp = float(sample["sim_s"])
        map_pose = sample_at_or_before(map_odom_filtered, stamp)
        local_pose = sample_at_or_before(odom_base, stamp)
        velocity = velocity_at_or_before(odom_velocity, stamp)
        if map_pose is None or local_pose is None or velocity is None:
            raise ValueError(f"missing causal input at paired sample {index}")
        candidate_x, candidate_y = compose_map_pose(map_pose, local_pose)
        gt_x, gt_y = (float(value) for value in sample["gt_map_xy"])
        baseline_x, baseline_y = (
            float(value) for value in sample["navigation_map_xy"]
        )
        rows.append(
            {
                "sim_s": stamp,
                "motion_class": motion_class(velocity[1], velocity[2]),
                "gt_map_x_m": gt_x,
                "gt_map_y_m": gt_y,
                "baseline_x_m": baseline_x,
                "baseline_y_m": baseline_y,
                "baseline_error_m": math.hypot(baseline_x - gt_x, baseline_y - gt_y),
                "map_odom_filtered_x_m": map_pose.x_m,
                "map_odom_filtered_y_m": map_pose.y_m,
                "map_odom_filtered_yaw_rad": map_pose.yaw_rad,
                "candidate_x_m": candidate_x,
                "candidate_y_m": candidate_y,
                "candidate_error_m": math.hypot(candidate_x - gt_x, candidate_y - gt_y),
            }
        )
    return rows


def metrics_by_motion_class(
    rows: Sequence[dict[str, float | int | str | None]],
) -> dict[str, dict[str, float | int | None]]:
    result: dict[str, dict[str, float | int | None]] = {}
    classes = sorted({str(row["motion_class"]) for row in rows})
    for motion_class_name in classes:
        selected = [row for row in rows if row["motion_class"] == motion_class_name]
        result[motion_class_name] = {
            "samples": len(selected),
            "baseline": metrics_with_mm(
                [{"error_m": row["baseline_error_m"]} for row in selected]
            ),
            "candidate": metrics_with_mm(
                [{"error_m": row["candidate_error_m"]} for row in selected]
            ),
        }
    return result


def write_candidate_csv(
    path: Path,
    rows: Sequence[dict[str, float | int | str | None]],
) -> None:
    fieldnames = list(rows[0].keys())
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def run(args: argparse.Namespace) -> dict:
    mcap_path = args.mcap.resolve()
    focus_path = args.focus_result.resolve()
    output_dir = args.output_dir.resolve()
    if output_dir.exists():
        raise ValueError(f"fresh output directory required: {output_dir}")
    output_dir.mkdir(parents=True)

    mcap_sha256 = sha256_file(mcap_path)
    if mcap_sha256 != args.expected_mcap_sha256:
        raise ValueError(
            f"MCAP SHA-256 mismatch: {mcap_sha256} != {args.expected_mcap_sha256}"
        )
    focus_sha256 = sha256_file(focus_path)
    focus_result = load_focus_result(focus_path)
    recorded_hashes = set(focus_result.get("files", {}).values())
    if mcap_sha256 not in recorded_hashes:
        raise ValueError("MCAP hash is not bound by the focus result")

    map_odom, odom_base, odom_velocity = decode_tf_and_odom(mcap_path)
    candidate_rows: list[dict[str, float | int | str | None]] = []
    sensitivity: dict[str, dict[str, object]] = {}
    for tau_sec in sorted(set((*args.sensitivity_tau_sec, args.tau_sec))):
        filtered = causal_lowpass(map_odom, tau_sec, args.max_dt_sec)
        rows = build_candidate_rows(
            focus_result, filtered, odom_base, odom_velocity
        )
        metrics = metrics_with_mm(
            [{"error_m": row["candidate_error_m"]} for row in rows]
        )
        sensitivity[f"{tau_sec:g}"] = {
            "tau_sec": tau_sec,
            "metrics": metrics,
            "strict_max_pass": bool(
                metrics["max_m"] is not None
                and float(metrics["max_m"]) <= STRICT_LIMIT_M
            ),
        }
        if tau_sec == args.tau_sec:
            candidate_rows = rows

    baseline_metrics = metrics_with_mm(
        [{"error_m": row["baseline_error_m"]} for row in candidate_rows]
    )
    candidate_metrics = metrics_with_mm(
        [{"error_m": row["candidate_error_m"]} for row in candidate_rows]
    )
    manifest_samples = int(focus_result["accuracy"]["samples"])
    if baseline_metrics["samples"] != manifest_samples:
        raise ValueError("baseline denominator changed")
    if candidate_metrics["samples"] != manifest_samples:
        raise ValueError("candidate denominator changed")

    csv_path = output_dir / "candidate_paired_samples.csv"
    write_candidate_csv(csv_path, candidate_rows)
    stamps = [float(row["sim_s"]) for row in candidate_rows]
    baseline_errors = [float(row["baseline_error_m"]) for row in candidate_rows]
    candidate_errors = [float(row["candidate_error_m"]) for row in candidate_rows]
    lags = (0.1, 0.5, 1.0, 5.0)
    receipt = {
        "schema_version": 1,
        "status": "OFFLINE_CANDIDATE_PASS"
        if all(
            candidate_metrics[key] is not None
            and float(candidate_metrics[key]) <= STRICT_LIMIT_M
            for key in ("rmse_m", "p95_m", "max_m")
        )
        else "FAIL",
        "scope": "same sealed dynamic run; causal online-input replay",
        "official_boundary": (
            "Official text gives <=50 mm without defining whether RMSE, P95, "
            "or max controls acceptance. This receipt therefore reports a "
            "strict max<=50 mm offline candidate result, not official PASS."
        ),
        "candidate": {
            "name": "causal_map_odom_first_order_lowpass",
            "tau_sec": args.tau_sec,
            "max_dt_sec": args.max_dt_sec,
            "sampling": "latest map->odom and odom->base_footprint sample at or before each scored epoch",
            "uses_future": False,
            "uses_ground_truth": False,
        },
        "online_inputs": ["/tf map->odom", "/tf odom->base_footprint", "/odom velocity classification"],
        "truth_usage": "paired_samples.gt_map_xy is used only after filtering, for scoring",
        "same_session": True,
        "time_alignment": "unchanged; original sim timestamps and denominator",
        "sample_counts": {
            "dynamic_reference": int(focus_result["dynamic_reference_samples"]),
            "strict_paired_manifest": manifest_samples,
            "baseline": int(baseline_metrics["samples"]),
            "candidate": int(candidate_metrics["samples"]),
            "map_odom_raw": len(map_odom),
            "odom_base": len(odom_base),
        },
        "metrics": {
            "strict_limit_m": STRICT_LIMIT_M,
            "baseline": baseline_metrics,
            "candidate": candidate_metrics,
            "delta_mm": {
                "rmse_mm": float(candidate_metrics["rmse_mm"]) - float(baseline_metrics["rmse_mm"]),
                "p95_mm": float(candidate_metrics["p95_mm"]) - float(baseline_metrics["p95_mm"]),
                "max_mm": float(candidate_metrics["max_mm"]) - float(baseline_metrics["max_mm"]),
            },
        },
        "strict_gates": {
            "rmse_le_50mm": bool(float(candidate_metrics["rmse_m"]) <= STRICT_LIMIT_M),
            "p95_le_50mm": bool(float(candidate_metrics["p95_m"]) <= STRICT_LIMIT_M),
            "max_le_50mm": bool(float(candidate_metrics["max_m"]) <= STRICT_LIMIT_M),
            "all_strict": bool(
                float(candidate_metrics["rmse_m"]) <= STRICT_LIMIT_M
                and float(candidate_metrics["p95_m"]) <= STRICT_LIMIT_M
                and float(candidate_metrics["max_m"]) <= STRICT_LIMIT_M
            ),
        },
        "motion_class_metrics": metrics_by_motion_class(candidate_rows),
        "time_correlation": {
            "baseline_error": {
                f"{lag:g}s": lagged_correlation(baseline_errors, stamps, lag)
                for lag in lags
            },
            "candidate_error": {
                f"{lag:g}s": lagged_correlation(candidate_errors, stamps, lag)
                for lag in lags
            },
        },
        "parameter_sensitivity": sensitivity,
        "evidence": {
            "mcap_path": str(mcap_path),
            "mcap_sha256": mcap_sha256,
            "focus_result_path": str(focus_path),
            "focus_result_sha256": focus_sha256,
            "candidate_csv": str(csv_path),
            "candidate_csv_sha256": sha256_file(csv_path),
            "rollback_point": "47e3cb3a7ecc01edd82aa23a3b54cbeaffc418bc",
        },
        "caveat": (
            "This is a counterfactual same-run replay. It does not replace a "
            "live Gazebo rerun or prove general map-frame accuracy."
        ),
    }
    receipt_path = output_dir / "recovery_receipt.json"
    receipt["evidence"]["receipt_path"] = str(receipt_path)
    with receipt_path.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write(
            json.dumps(receipt, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
        )
    return receipt


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mcap", type=Path, required=True)
    parser.add_argument("--focus-result", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--tau-sec", type=float, default=DEFAULT_TAU_SEC)
    parser.add_argument("--max-dt-sec", type=float, default=DEFAULT_MAX_DT_SEC)
    parser.add_argument(
        "--expected-mcap-sha256",
        default=DEFAULT_MCAP_SHA256,
    )
    parser.add_argument(
        "--sensitivity-tau-sec",
        type=float,
        nargs="+",
        default=list(DEFAULT_SENSITIVITY_TAU_SEC),
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    requested_output_dir = args.output_dir.resolve()
    output_preexisted = requested_output_dir.exists()
    try:
        receipt = run(args)
    except Exception as error:
        failure = {
            "schema_version": 1,
            "status": "FAIL",
            "failure_type": type(error).__name__,
            "failure": str(error),
            "scope": "same sealed dynamic run; causal online-input replay",
            "strict_limit_m": STRICT_LIMIT_M,
        }
        if not output_preexisted:
            requested_output_dir.mkdir(parents=True, exist_ok=True)
            failure_path = requested_output_dir / "recovery_receipt.json"
            with failure_path.open("w", encoding="utf-8", newline="\n") as stream:
                stream.write(
                    json.dumps(failure, ensure_ascii=False, indent=2) + "\n"
                )
            failure["receipt_path"] = str(failure_path)
        print(json.dumps(failure, ensure_ascii=False))
        return 2
    print(json.dumps(receipt["metrics"], ensure_ascii=False, indent=2))
    return 0 if receipt["status"] == "OFFLINE_CANDIDATE_PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
