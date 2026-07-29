from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import linprog


ROOT = Path(__file__).resolve().parents[1]
SPOT = ROOT / "starter_ws" / "src" / "sanitation_spot_cleaning"
sys.path.insert(0, str(SPOT))

from sanitation_spot_cleaning.observation_pose_planner import (  # noqa: E402
    CandidateRegion,
    ObservationPosePlanner,
    Pose2D,
    VerificationCameraModel,
)


WORLD_IDS = (
    "world_a_asphalt_campus",
    "world_b_concrete_sidewalk",
    "world_c_wet_dark_ground",
    "world_d_mixed_curb_vegetation",
    "world_e_tiled_plaza",
    "world_f_service_road",
)

CLASS_PROJECTION_CALIBRATION = (
    ("plastic_bottle", 3.3742821866035477, -22.005209523783325, 1.297),
    ("metal_can", 0.6257472835344164, -16.946553085057214, 1.285),
    ("paper_litter", -4.061201371314579, -4.3156677050183445, 0.920),
    ("leaf_pile", 0.0, 0.0, 0.695),
    ("puddle", 1.4460236390133452, 4.448860887459424, 0.751),
)


def camera_without_short_correction() -> VerificationCameraModel:
    focal_px = 640.0 / (2.0 * math.tan(1.50098 / 2.0))
    return VerificationCameraModel(
        width_px=640,
        height_px=480,
        horizontal_fov_rad=1.50098,
        mount_xyz_m=(0.32, 0.28, 0.66),
        pitch_rad=math.radians(-35.0),
        predicted_self_pixel_fraction=0.0,
        predicted_target_self_overlap=0.0,
        mount_rpy_rad=(0.0, math.radians(35.0), math.radians(45.0)),
        fx_px=focal_px,
        fy_px=focal_px,
        cx_px=320.0,
        cy_px=240.0,
        projection_center_affine=(
            0.9200577497930318,
            0.19807390392387791,
            -6.743277535198672,
            -0.009249842484302759,
            1.2094590560977079,
            -1.5324689495967263,
        ),
        class_projection_calibration=CLASS_PROJECTION_CALIBRATION,
        projection_roi_margin_px=15.0,
    )


def percentile_metrics(rows: list[dict], coefficients: dict[str, np.ndarray]) -> dict:
    errors = [
        abs(
            row["base_short_side_px"]
            * float(row["features"] @ coefficients[row["class_id"]])
            - row["actual_short_side_px"]
        )
        / max(row["actual_short_side_px"], 1.0)
        for row in rows
    ]
    return {
        "sample_count": len(errors),
        "short_side_relative_error_p50": float(np.percentile(errors, 50)),
        "short_side_relative_error_p95": float(np.percentile(errors, 95)),
        "short_error_over_0_30_count": sum(value > 0.30 for value in errors),
        "short_side_relative_error_max": max(errors),
    }


def current_metrics(rows: list[dict]) -> dict:
    errors = [
        abs(row["current_predicted_short_side_px"] - row["actual_short_side_px"])
        / max(row["actual_short_side_px"], 1.0)
        for row in rows
    ]
    return {
        "sample_count": len(errors),
        "short_side_relative_error_p50": float(np.percentile(errors, 50)),
        "short_side_relative_error_p95": float(np.percentile(errors, 95)),
        "short_error_over_0_30_count": sum(value > 0.30 for value in errors),
        "short_side_relative_error_max": max(errors),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--raw-root", required=True)
    parser.add_argument("--matrix", required=True)
    parser.add_argument(
        "--train-worlds",
        default=",".join(WORLD_IDS[:4]),
    )
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    raw_root = Path(args.raw_root)
    matrix = json.loads(Path(args.matrix).read_text(encoding="utf-8"))
    truth = {str(item["candidate_id"]): item for item in matrix["trials"]}
    camera = camera_without_short_correction()
    affine = np.array(
        [
            camera.projection_center_affine[:3],
            camera.projection_center_affine[3:],
            (0.0, 0.0, 1.0),
        ]
    )
    inverse_affine = np.linalg.inv(affine)
    class_calibration = {
        class_id: (dx, dy, scale)
        for class_id, dx, dy, scale in CLASS_PROJECTION_CALIBRATION
    }
    rows = []
    for world_id in WORLD_IDS:
        runtime = json.loads(
            (raw_root / world_id / "runtime_trials.json").read_text(
                encoding="utf-8"
            )
        )
        for trial in runtime["trials"]:
            projection = trial.get("projection")
            if projection is None:
                continue
            candidate = truth[str(trial["candidate_id"])]
            oracle = candidate["oracle_candidate"]
            region = CandidateRegion(
                candidate_id=str(trial["candidate_id"]),
                center_xy_m=(float(oracle["x_m"]), float(oracle["y_m"])),
                target_size_m=float(oracle["target_size_m"]),
                class_id=str(candidate["class_id"]),
            )
            base_short, roi, _angle, _visible = (
                ObservationPosePlanner._camera_projection(
                    region,
                    Pose2D(*trial["after_approach_pose_map"]),
                    camera,
                )
            )
            center_x = (roi[0] + roi[2]) / 2.0
            center_y = (roi[1] + roi[3]) / 2.0
            dx, dy, _scale = class_calibration[region.class_id]
            raw_center = inverse_affine @ np.array(
                [center_x - dx, center_y - dy, 1.0]
            )
            rows.append(
                {
                    "world_id": world_id,
                    "candidate_id": region.candidate_id,
                    "class_id": region.class_id,
                    "features": np.array(
                        [
                            1.0,
                            (raw_center[0] - 320.0) / 200.0,
                            (raw_center[1] - 240.0) / 120.0,
                            base_short / 60.0,
                            region.target_size_m / 0.30,
                        ]
                    ),
                    "base_short_side_px": base_short,
                    "current_predicted_short_side_px": float(
                        projection["predicted_short_side_px"]
                    ),
                    "actual_short_side_px": float(
                        projection["actual_short_side_px"]
                    ),
                }
            )

    train_worlds = set(args.train_worlds.split(","))
    train = [row for row in rows if row["world_id"] in train_worlds]
    heldout = [row for row in rows if row["world_id"] not in train_worlds]
    if not train or not heldout:
        raise ValueError("calibration requires non-empty train and held-out rows")

    coefficients: dict[str, np.ndarray] = {}
    for class_id in sorted({row["class_id"] for row in rows}):
        class_rows = [row for row in train if row["class_id"] == class_id]
        features = np.array(
            [
                row["features"]
                * row["base_short_side_px"]
                / row["actual_short_side_px"]
                for row in class_rows
            ]
        )
        coefficient_count = features.shape[1]
        inequalities = np.vstack(
            [
                np.column_stack([features, -np.ones(len(features))]),
                np.column_stack([-features, -np.ones(len(features))]),
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
        coefficients[class_id] = solution.x[:-1]

    report = {
        "schema_version": 1,
        "stage": "AUTO-03",
        "fit_scope": "capture_projection_short_side_only",
        "train_worlds": sorted(train_worlds),
        "heldout_worlds": sorted(
            {row["world_id"] for row in heldout}
        ),
        "row_count": len(rows),
        "short_side_feature_order": [
            "intercept",
            "(raw_center_x_px-320)/200",
            "(raw_center_y_px-240)/120",
            "base_short_side_px/60",
            "target_size_m/0.30",
        ],
        "recommended_class_short_side_correction": {
            class_id: value.tolist()
            for class_id, value in coefficients.items()
        },
        "before": {
            "train": current_metrics(train),
            "heldout": current_metrics(heldout),
            "all": current_metrics(rows),
        },
        "after": {
            "train": percentile_metrics(train, coefficients),
            "heldout": percentile_metrics(heldout, coefficients),
            "all": percentile_metrics(rows, coefficients),
        },
    }
    Path(args.output).write_text(
        json.dumps(report, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
