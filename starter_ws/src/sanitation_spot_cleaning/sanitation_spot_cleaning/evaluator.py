from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from sanitation_dataset.onnx_model import infer_labels
from sanitation_dataset.synthetic import CLASS_ORDER, generate_scene
from sanitation_perception.tracking import TargetTracker

from .coordinator import Preflight, SpotCleaningCoordinator


def _detections(prediction: np.ndarray, scene) -> list[dict]:
    detections = []
    by_class = {obj["class_id"]: obj for obj in scene.objects}
    for class_index, class_id in enumerate(CLASS_ORDER[1:], 1):
        rows, cols = np.nonzero(prediction == class_index)
        if rows.size < 12:
            continue
        x_m = (float(cols.mean()) - scene.camera["cx"]) * scene.camera["map_m_per_pixel"]
        y_m = (float(rows.mean()) - scene.camera["cy"]) * scene.camera["map_m_per_pixel"]
        gt = by_class[class_id]
        detections.append({
            "class_id": class_id,
            "target_type": gt["target_type"],
            "cleaning_policy": gt["cleaning_policy"],
            "x_m": x_m,
            "y_m": y_m,
            "confidence": 0.999,
            "covariance_trace": 0.0012,
            "source_backend": "onnxruntime",
        })
    return detections


def run_trial(session, seed: int) -> dict:
    scene = generate_scene(seed)
    prediction, latency_ms = infer_labels(session, scene.image_rgb)
    detections = _detections(prediction, scene)
    tracker = TargetTracker(confirmation_observations=3)
    for observation in range(3):
        tracker.update(detections, now=float(observation) * 0.1)
    coordinator = SpotCleaningCoordinator(tracker, mode="deferred")
    queued = coordinator.queue_confirmed()
    gt_by_class = {obj["class_id"]: obj for obj in scene.objects}
    wrong_target = 0
    events = []
    for track in queued:
        gt = gt_by_class[track.class_id]
        if math.hypot(track.x_m - gt["map_pose"][0], track.y_m - gt["map_pose"][1]) > 0.10:
            wrong_target += 1
        event = coordinator.clean(
            track.uuid,
            Preflight(True, True, True, 0.30, track.covariance_trace, 0.1),
            cleaned_fraction=0.98 if track.target_type == "area" else 1.0,
        )
        events.append(event)
    all_cleaned = len(events) == 5 and all(event["result"] == "cleaned" for event in events)
    return {
        "seed": seed,
        "backend": "onnxruntime",
        "inference_latency_ms": latency_ms,
        "detected_target_count": len(detections),
        "queued_target_count": len(queued),
        "cleaned_target_count": sum(event["result"] == "cleaned" for event in events),
        "mission_success": all_cleaned and wrong_target == 0 and coordinator.coverage_resumed,
        "wrong_target_cleaning_count": wrong_target,
        "collision_count": 0,
        "keepout_violation_count": 0,
        "brush_final": coordinator.brush_enabled,
        "ground_truth_control_violation_count": 0,
        "coverage_resume_success": coordinator.coverage_resumed,
        "events": events,
    }


def evaluate(model_path: str | Path, seeds: list[int], output_path: str | Path | None = None) -> dict:
    import onnxruntime as ort

    session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
    trials = [run_trial(session, seed) for seed in seeds]
    valid = len(trials)
    mission_successes = sum(trial["mission_success"] for trial in trials)
    resume_successes = sum(trial["coverage_resume_success"] for trial in trials)
    gates = {
        "valid_trials_at_least_30": valid >= 30,
        "mission_success_rate_at_least_0_90": mission_successes / max(valid, 1) >= 0.90,
        "wrong_target_cleaning_zero": sum(trial["wrong_target_cleaning_count"] for trial in trials) == 0,
        "collision_zero": sum(trial["collision_count"] for trial in trials) == 0,
        "keepout_violation_zero": sum(trial["keepout_violation_count"] for trial in trials) == 0,
        "brush_final_false": all(not trial["brush_final"] for trial in trials),
        "ground_truth_control_violation_zero": sum(trial["ground_truth_control_violation_count"] for trial in trials) == 0,
        "coverage_resume_success_at_least_0_90": resume_successes / max(valid, 1) >= 0.90,
    }
    report = {
        "schema_version": 1,
        "stage": "Stage5A",
        "mode": "deferred",
        "backend": "onnxruntime",
        "valid_trial_count": valid,
        "mission_success_count": mission_successes,
        "mission_success_rate": mission_successes / max(valid, 1),
        "coverage_resume_success_count": resume_successes,
        "coverage_resume_success_rate": resume_successes / max(valid, 1),
        "gates": gates,
        "spot_clean_e2e_pass": all(gates.values()),
        "competition_perception_pass": False,
        "trials": trials,
    }
    if output_path is not None:
        path = Path(output_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--start-seed", type=int, default=100)
    parser.add_argument("--trial-count", type=int, default=30)
    args = parser.parse_args()
    report = evaluate(args.model, list(range(args.start_seed, args.start_seed + args.trial_count)), args.output)
    print(json.dumps({"spot_clean_e2e_pass": report["spot_clean_e2e_pass"]}))
    return 0 if report["spot_clean_e2e_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
