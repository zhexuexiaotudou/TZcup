from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import yaml
from scipy.optimize import linprog


def read_capture_requests(bag: Path) -> dict[str, dict]:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    message_type = get_message(topic_types["/auto03/capture_request"])
    captures: dict[str, dict] = {}
    while reader.has_next():
        topic, data, _timestamp = reader.read_next()
        if topic != "/auto03/capture_request":
            continue
        message = deserialize_message(data, message_type)
        payload = json.loads(message.data)
        captures[str(payload["candidate_id"])] = payload
    return captures


def corrected_center(row: dict, correction, center_offsets) -> np.ndarray:
    predicted = np.array(
        [row["predicted_center_x_px"], row["predicted_center_y_px"], 1.0]
    )
    return correction @ predicted + np.array(
        center_offsets[row["class_id"]]
    )


def short_features(row: dict) -> np.ndarray:
    return np.array(
        [
            1.0,
            (float(row["raw_center_x_px"]) - 320.0) / 200.0,
            (float(row["raw_center_y_px"]) - 240.0) / 120.0,
            float(row["predicted_short_side_px"]) / 60.0,
            float(row["target_size_m"]) / 0.30,
        ]
    )


def percentile_metrics(
    rows: list[dict],
    *,
    correction,
    center_offsets,
    short_coefficients,
) -> dict:
    center_errors = []
    short_errors = []
    for row in rows:
        center = corrected_center(row, correction, center_offsets)
        center_errors.append(
            float(
                np.hypot(
                    center[0] - row["actual_center_x_px"],
                    center[1] - row["actual_center_y_px"],
                )
            )
        )
        correction_factor = float(
            short_features(row)
            @ np.array(short_coefficients[row["class_id"]])
        )
        corrected_short = (
            row["predicted_short_side_px"] * correction_factor
        )
        short_errors.append(
            abs(corrected_short - row["actual_short_side_px"])
            / row["actual_short_side_px"]
        )
    return {
        "sample_count": len(rows),
        "center_pixel_error_p50": float(np.percentile(center_errors, 50)),
        "center_pixel_error_p95": float(np.percentile(center_errors, 95)),
        "short_side_relative_error_p50": float(
            np.percentile(short_errors, 50)
        ),
        "short_side_relative_error_p95": float(
            np.percentile(short_errors, 95)
        ),
        "center_error_over_25_count": sum(value > 25.0 for value in center_errors),
        "short_error_over_0_30_count": sum(
            value > 0.30 for value in short_errors
        ),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--raw-root", required=True)
    parser.add_argument("--matrix", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument(
        "--train-worlds",
        default=(
            "world_a_asphalt_campus,world_b_concrete_sidewalk,"
            "world_c_wet_dark_ground,world_d_mixed_curb_vegetation"
        ),
    )
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    raw_root = Path(args.raw_root)
    matrix = json.loads(Path(args.matrix).read_text(encoding="utf-8"))
    truth = {
        str(item["candidate_id"]): item for item in matrix["trials"]
    }
    rows = []
    for world_root in sorted(raw_root.glob("world_*")):
        runtime_path = world_root / "runtime_trials.json"
        bag = world_root / "auto03_runtime_bag"
        if not runtime_path.is_file() or not bag.is_dir():
            continue
        captures = read_capture_requests(bag)
        runtime = json.loads(runtime_path.read_text(encoding="utf-8"))
        for trial in runtime["trials"]:
            candidate_id = str(trial["candidate_id"])
            capture = captures.get(candidate_id)
            bbox = trial.get("actual_bbox_xywh")
            projection = trial.get("projection")
            if capture is None or bbox is None or projection is None:
                continue
            roi = capture["expected_roi_xyxy"]
            rows.append(
                {
                    "world_id": truth[candidate_id]["world_id"],
                    "candidate_id": candidate_id,
                    "class_id": truth[candidate_id]["class_id"],
                    "predicted_center_x_px": (roi[0] + roi[2]) / 2.0,
                    "predicted_center_y_px": (roi[1] + roi[3]) / 2.0,
                    "actual_center_x_px": bbox[0] + bbox[2] / 2.0,
                    "actual_center_y_px": bbox[1] + bbox[3] / 2.0,
                    "predicted_short_side_px": projection[
                        "predicted_short_side_px"
                    ],
                    "actual_short_side_px": projection[
                        "actual_short_side_px"
                    ],
                    "target_size_m": float(
                        truth[candidate_id]["oracle_candidate"][
                            "target_size_m"
                        ]
                    ),
                }
            )

    train_worlds = set(args.train_worlds.split(","))
    train = [row for row in rows if row["world_id"] in train_worlds]
    heldout = [row for row in rows if row["world_id"] not in train_worlds]
    if not train or not heldout:
        raise ValueError("calibration requires non-empty train and held-out rows")

    classes = sorted(
        {row["class_id"] for row in train + heldout}
    )
    reference_class = classes[0]
    center_design = np.array(
        [
            [
                row["predicted_center_x_px"],
                row["predicted_center_y_px"],
                1.0,
                *[
                    1.0 if row["class_id"] == class_id else 0.0
                    for class_id in classes[1:]
                ],
            ]
            for row in train
        ]
    )
    actual = np.array(
        [
            [row["actual_center_x_px"], row["actual_center_y_px"]]
            for row in train
        ]
    )
    center_model = np.linalg.lstsq(
        center_design, actual, rcond=None
    )[0]
    correction = center_model[:3].T
    center_offsets = {
        reference_class: [0.0, 0.0],
        **{
            class_id: center_model[3 + index].tolist()
            for index, class_id in enumerate(classes[1:])
        },
    }

    config = yaml.safe_load(Path(args.config).read_text(encoding="utf-8"))
    parameters = config["stage5br5_observation_pose_planner"][
        "ros__parameters"
    ]
    base_affine = np.array(
        [
            parameters["projection_center_affine"][:3],
            parameters["projection_center_affine"][3:],
            [0.0, 0.0, 1.0],
        ]
    )
    composed_affine = correction @ base_affine
    calibration_map = json.loads(
        parameters["class_projection_calibration_json"]
    )
    base_scales = {
        class_id: float(values[2])
        for class_id, values in calibration_map.items()
    }
    base_offsets = {
        class_id: np.array(values[:2], dtype=float)
        for class_id, values in calibration_map.items()
    }
    inverse_base_affine = np.linalg.inv(base_affine)
    for row in rows:
        base_offset = base_offsets[row["class_id"]]
        raw_center = inverse_base_affine @ np.array(
            [
                row["predicted_center_x_px"] - base_offset[0],
                row["predicted_center_y_px"] - base_offset[1],
                1.0,
            ]
        )
        row["raw_center_x_px"] = float(raw_center[0])
        row["raw_center_y_px"] = float(raw_center[1])
    recommended_offsets = {
        class_id: (
            correction[:, :2] @ base_offsets[class_id]
            + np.array(center_offsets[class_id])
        ).tolist()
        for class_id in classes
    }
    short_coefficients = {}
    for class_id in classes:
        class_rows = [row for row in train if row["class_id"] == class_id]
        features = np.array(
            [
                short_features(row)
                * float(row["predicted_short_side_px"])
                / float(row["actual_short_side_px"])
                for row in class_rows
            ]
        )
        coefficient_count = features.shape[1]
        inequalities = np.vstack(
            [
                np.column_stack(
                    [features, -np.ones(len(features))]
                ),
                np.column_stack(
                    [-features, -np.ones(len(features))]
                ),
            ]
        )
        bounds = np.concatenate(
            [np.ones(len(features)), -np.ones(len(features))]
        )
        solution = linprog(
            np.concatenate(
                [np.zeros(coefficient_count), np.ones(1)]
            ),
            A_ub=inequalities,
            b_ub=bounds,
            bounds=[(None, None)] * coefficient_count + [(0.0, None)],
            method="highs",
        )
        if not solution.success:
            raise RuntimeError(
                f"short-side calibration failed for {class_id}: "
                f"{solution.message}"
            )
        short_coefficients[class_id] = solution.x[:-1].tolist()

    identity = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    zero_offsets = {class_id: [0.0, 0.0] for class_id in classes}
    identity_short = {
        class_id: [1.0, 0.0, 0.0, 0.0, 0.0]
        for class_id in classes
    }
    report = {
        "schema_version": 1,
        "stage": "AUTO-03",
        "train_worlds": sorted(train_worlds),
        "heldout_worlds": sorted(
            {row["world_id"] for row in heldout}
        ),
        "row_count": len(rows),
        "center_correction_from_current_prediction": correction.tolist(),
        "recommended_projection_center_affine": composed_affine.flatten().tolist(),
        "recommended_class_projection_calibration": {
            class_id: [
                *recommended_offsets[class_id],
                base_scales[class_id],
            ]
            for class_id in classes
        },
        "recommended_class_short_side_correction": short_coefficients,
        "short_side_feature_order": [
            "intercept",
            "(raw_center_x_px-320)/200",
            "(raw_center_y_px-240)/120",
            "base_short_side_px/60",
            "target_size_m/0.30",
        ],
        "before": {
            "train": percentile_metrics(
                train,
                correction=identity,
                center_offsets=zero_offsets,
                short_coefficients=identity_short,
            ),
            "heldout": percentile_metrics(
                heldout,
                correction=identity,
                center_offsets=zero_offsets,
                short_coefficients=identity_short,
            ),
            "all": percentile_metrics(
                rows,
                correction=identity,
                center_offsets=zero_offsets,
                short_coefficients=identity_short,
            ),
        },
        "after": {
            "train": percentile_metrics(
                train,
                correction=correction,
                center_offsets=center_offsets,
                short_coefficients=short_coefficients,
            ),
            "heldout": percentile_metrics(
                heldout,
                correction=correction,
                center_offsets=center_offsets,
                short_coefficients=short_coefficients,
            ),
            "all": percentile_metrics(
                rows,
                correction=correction,
                center_offsets=center_offsets,
                short_coefficients=short_coefficients,
            ),
        },
        "rows": rows,
    }
    Path(args.output).write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps({key: report[key] for key in report if key != "rows"}, indent=2))


if __name__ == "__main__":
    main()
