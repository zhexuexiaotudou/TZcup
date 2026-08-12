#!/usr/bin/env python3
"""Exercise MA1 through projection, tracker, DynamicTrashMap, and scheduler."""

from __future__ import annotations

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import statistics
import sys
import time

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_perception"))
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_spot_cleaning"))

from sanitation_learning.opr_c_rtmdet import patch_mmdet_cuda_nms  # noqa: E402
from sanitation_perception.camera_frustum_model import CameraFrustumModel  # noqa: E402
from sanitation_perception.dynamic_trash_map import DynamicTrashMap, DynamicTrashMapConfig  # noqa: E402
from sanitation_perception.observation_model import MapPoseMeasurement, TargetObservation  # noqa: E402
from sanitation_perception.product_postprocess import project_discrete_predictions  # noqa: E402
from sanitation_perception.rtmdet_product_runtime import RTMDetProductRuntime, file_sha256  # noqa: E402
from sanitation_perception.tracker_v2 import ProductTrackerV2, TrackerV2Config  # noqa: E402
from sanitation_spot_cleaning.cleaning_task_scheduler import (  # noqa: E402
    CleaningTaskScheduler, CoverageContext, SafetyContext, TargetSchedulingInput,
)


def jsonl(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def iou(a, b) -> float:
    x1, y1, x2, y2 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
    intersection = max(0.0, x2-x1) * max(0.0, y2-y1)
    area_a = max(0.0, a[2]-a[0]) * max(0.0, a[3]-a[1]); area_b = max(0.0, b[2]-b[0]) * max(0.0, b[3]-b[1])
    return intersection / max(area_a + area_b - intersection, 1e-12)


def camera(payload: dict) -> dict:
    k = payload["camera_info"]["k"]
    return {"fx": k[0], "fy": k[4], "cx": k[2], "cy": k[5], "pixel_sigma": 0.5, "depth_sigma_m": 0.02}


def transform(payload: dict) -> np.ndarray:
    pose = payload["tf"]["pose"]; yaw = float(pose["yaw_rad"]); c, s = math.cos(yaw), math.sin(yaw)
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = np.array([[0, -s, c], [c, 0, s], [0, -1, 0]], dtype=np.float64)
    matrix[:3, 3] = [float(pose["x_m"]), float(pose["y_m"]), 0.66]
    return matrix


def project(rows: list[dict], depth: np.ndarray, cam: dict, tf: np.ndarray) -> list[dict]:
    inputs = [{"bbox_xyxy": row["bbox_xyxy"], "class_id": row["class_name"], "confidence": row["score"], "class_probabilities": {row["class_name"]: row["score"], "background": 1-row["score"]}} for row in rows]
    outputs = project_discrete_predictions(inputs, depth, cam, tf)
    for output in outputs: output["source_backend"] = "mmdetection_rtmdet_s"
    return outputs


def nearest_truth(projected: dict, truths: list[dict], used: set[int]) -> tuple[int, dict] | None:
    candidates = sorted(((iou(projected["bbox_xyxy"], truth["bbox_xyxy"]), index, truth) for index, truth in enumerate(truths) if index not in used), reverse=True, key=lambda row: row[0])
    return (candidates[0][1], candidates[0][2]) if candidates and candidates[0][0] >= 0.5 else None


def percentile(values: list[float], p: float) -> float | None:
    if not values: return None
    return float(np.percentile(np.asarray(values), p))


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--data-root", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path); parser.add_argument("--checkpoint", required=True, type=Path)
    parser.add_argument("--expected-sha256", required=True); parser.add_argument("--threshold", required=True, type=float)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists(): raise FileExistsError(args.output)
    args.output.mkdir(parents=True)
    patch_mmdet_cuda_nms()
    runtime = RTMDetProductRuntime(args.config, args.checkpoint, expected_sha256=args.expected_sha256, observation_threshold=0.05, action_threshold=args.threshold)
    manifest = yaml.safe_load((ROOT / "starter_ws/src/sanitation_perception/config/perception_pipeline_manifest.yaml").read_text(encoding="utf-8"))
    tracker_config = TrackerV2Config.from_pipeline_manifest(manifest)
    map_config = DynamicTrashMapConfig(**manifest["runtime"]["dynamic_trash_map"])
    frustum = CameraFrustumModel(**manifest["runtime"]["camera_frustum"])
    rows = [row for row in jsonl(args.data_root / "G7_MOVING_FRAME_MANIFEST.jsonl") if row["split"] == "MOVING_VAL"]
    encounters = [row for row in jsonl(args.data_root / "G7_MOVING_EVALUATOR_ENCOUNTERS.jsonl") if row["split"] == "MOVING_VAL" and row["actionable"]]
    eligible_ids = {row["target_id"] for row in encounters}
    mission_rows: dict[str, list[dict]] = defaultdict(list)
    for row in rows: mission_rows[row["mission_id"]].append(row)
    mission_reports=[]; all_errors=[]; direct_projection_errors=[]; all_projection=[]; scheduler_actions=[]; evaluator_geometry_spreads=[]
    confirmed_matches=0; confirmed_total=0; duplicate_count=0; fragmentation_count=0; matched_targets=set(); pre_fov_creation=0; wrong_clean=0; identity_numerators=[]
    for mission_id, frames in sorted(mission_rows.items()):
        tracker=ProductTrackerV2(tracker_config); dynamic_map=DynamicTrashMap.start_new(mission_id, config=map_config); scheduler=CleaningTaskScheduler()
        seen_truth=set(); mission_gt_positions=defaultdict(list); target_track_ids=defaultdict(list); last_visible={}
        for row in sorted(frames,key=lambda item:item["frame_index"]):
            metadata=json.loads((args.data_root/row["frame_path"]).read_text(encoding="utf-8")); truths=json.loads((args.data_root/row["evaluator_gt_path"]).read_text(encoding="utf-8"))["objects"]
            bgr=cv2.imread(str(args.data_root/row["rgb_path"]),cv2.IMREAD_COLOR); depth=cv2.imread(str(args.data_root/row["depth_path"]),cv2.IMREAD_UNCHANGED).astype(np.float32)/1000.0
            cam=camera(metadata); tf=transform(metadata); detections=runtime.infer_bgr(bgr); projected=project([item for item in detections if item["score"]>=args.threshold],depth,cam,tf)
            all_projection.append({"mission_id":mission_id,"frame_index":row["frame_index"],"detections":len(detections),"actionable":sum(item["score"]>=args.threshold for item in detections),"projected":len(projected)})
            # GT projection exists only in this evaluator-side branch.
            gt_projected=project([{"bbox_xyxy":truth["bbox_xyxy"],"class_name":truth["class_name"],"score":1.0} for truth in truths],depth,cam,tf)
            for truth, point in zip(truths,gt_projected): mission_gt_positions[truth["target_id"]].append((point["x_m"],point["y_m"],truth["class_name"])); seen_truth.add(truth["target_id"]); last_visible[truth["target_id"]]=row["frame_index"]
            direct_used=set()
            for detection in projected:
                match=nearest_truth(detection,truths,direct_used)
                if not match or detection["class_id"] != match[1]["class_name"]: continue
                direct_used.add(match[0]); truth_point=gt_projected[match[0]]
                direct_projection_errors.append(math.hypot(detection["x_m"]-truth_point["x_m"],detection["y_m"]-truth_point["y_m"]))
            tracks=tracker.update(projected,row["frame_index"]/15.0)
            used=set()
            for detection in projected:
                match=nearest_truth(detection,truths,used)
                if match: used.add(match[0])
            for track in tracks:
                current_track = abs(track.last_seen_s-row["frame_index"]/15.0)<=1e-9
                candidates=sorted(((math.hypot(track.x_m-point["x_m"],track.y_m-point["y_m"]),truth) for truth,point in zip(truths,gt_projected) if truth["class_name"]==track.class_id),key=lambda item:item[0])
                if current_track and candidates and candidates[0][0]<=0.30: target_track_ids[candidates[0][1]["target_id"]].append(track.uuid)
                # update() returns the full track table.  Only a track observed
                # in this frame is a new camera observation for map fusion.
                if track.state != "CONFIRMED" or abs(track.last_seen_s-row["frame_index"]/15.0)>1e-9: continue
                stamp=int(row["timestamp_ns"]); image_id=f"{mission_id}:{row['frame_index']}"; pose=metadata["tf"]["pose"]
                dynamic_map.observed_regions.record(frustum.make_sweep(sweep_id=image_id,mission_id=mission_id,stamp_ns=stamp,camera_frame_id="camera_color_optical_frame",image_frame_id=image_id,camera_x_m=float(pose["x_m"]),camera_y_m=float(pose["y_m"]),camera_yaw_rad=float(pose["yaw_rad"])))
                observation=TargetObservation(observation_id=f"{image_id}:{track.uuid}",mission_id=mission_id,stamp_ns=stamp,camera_frame_id="camera_color_optical_frame",image_frame_id=image_id,source_model=f"MA1:{args.expected_sha256}",source_backend="mmdetection_rtmdet_s",target_type="DISCRETE",class_probabilities=track.class_posterior,confidence=track.score_ema,map_pose=MapPoseMeasurement(track.x_m,track.y_m,track.z_m,max(track.covariance_trace/2,1e-6),0,max(track.covariance_trace/2,1e-6)),bbox_xyxy=track.bbox_xyxy,in_current_fov=True)
                target=dynamic_map.ingest(observation)
                if target and target.observation_count==1 and not seen_truth: pre_fov_creation+=1
        confirmed=[target for target in dynamic_map.targets.values() if target.track_state.value=="CONFIRMED"]
        confirmed_total+=len(confirmed); assigned=defaultdict(list)
        gt_centers={target_id:(statistics.median([v[0] for v in values]),statistics.median([v[1] for v in values]),values[0][2]) for target_id,values in mission_gt_positions.items()}
        for target_id, values in mission_gt_positions.items():
            center = gt_centers[target_id]
            evaluator_geometry_spreads.append(max(math.hypot(value[0]-center[0],value[1]-center[1]) for value in values))
        pairs=sorted((math.hypot(target.map_x_m-center[0],target.map_y_m-center[1]),target,target_id) for target in confirmed for target_id,center in gt_centers.items() if center[2]==target.current_class)
        used_targets=set(); used_maps=set(); matches=[]
        for error,target,target_id in pairs:
            if target.uuid in used_maps or target_id in used_targets: continue
            used_maps.add(target.uuid); used_targets.add(target_id); matches.append((error,target,target_id)); assigned[target_id].append(target.uuid)
        matched_map_ids=set()
        for error,target,target_id in matches:
            if error<=0.30: confirmed_matches+=1; matched_targets.add(target_id); matched_map_ids.add(target.uuid); all_errors.append(error)
        for target in confirmed:
            decision=scheduler.decide(TargetSchedulingInput(target.uuid,target.track_state.value,target.confidence,target.observation_count,0.3,0.5,1.0,0.2,0.2,tuple(target.source_models)),CoverageContext("RUNNING",True,"SWATH_COMPLETE"),SafetyContext(True,True,True,True,True,0.3,target.covariance_trace))
            scheduler_actions.append({"mission_id":mission_id,"target_uuid":target.uuid,**decision.to_record()})
            if decision.action.value=="CLEAN_NOW" and target.uuid not in matched_map_ids: wrong_clean+=1
        duplicate_count+=max(0,len(confirmed)-len(matched_map_ids)); fragmentation_count+=sum(max(0,len(set(ids))-1) for ids in target_track_ids.values())
        for ids in target_track_ids.values():
            if ids: identity_numerators.append(max(ids.count(value) for value in set(ids))/len(ids))
        mission_reports.append({"mission_id":mission_id,"frame_count":len(frames),"tracker_count":len(tracker.tracks),"dynamic_map_confirmed":len(confirmed),"matched_confirmed":len(matched_map_ids),"target_track_identity":{target_id:{"observations":len(ids),"unique_tracks":len(set(ids)),"dominant_fraction":max(ids.count(value) for value in set(ids))/len(ids)} for target_id,ids in target_track_ids.items() if ids}})
    projection_total=sum(item["actionable"] for item in all_projection); projection_success=sum(item["projected"] for item in all_projection)
    metrics={"valid_depth_correct_detection_projection_success":projection_success/max(projection_total,1),"projection_success_count":projection_success,"projection_input_count":projection_total,"direct_projection_median_error_m":statistics.median(direct_projection_errors) if direct_projection_errors else None,"direct_projection_p95_error_m":percentile(direct_projection_errors,95),"direct_projection_rmse_m":math.sqrt(sum(v*v for v in direct_projection_errors)/len(direct_projection_errors)) if direct_projection_errors else None,"median_map_localization_error_m":statistics.median(all_errors) if all_errors else None,"p95_map_localization_error_m":percentile(all_errors,95),"map_rmse_m":math.sqrt(sum(v*v for v in all_errors)/len(all_errors)) if all_errors else None,"track_creation_recall":len(matched_targets)/max(len(eligible_ids),1),"id_consistency":sum(identity_numerators)/len(identity_numerators) if identity_numerators else None,"duplicate_track_rate":duplicate_count/max(len(eligible_ids),1),"track_fragmentation_rate":fragmentation_count/max(len(eligible_ids),1),"discrete_map_precision":confirmed_matches/max(confirmed_total,1),"discrete_map_coverage":len(matched_targets)/max(len(eligible_ids),1),"pre_FOV_target_creation":pre_fov_creation,"wrong_class_or_false_target_CLEAN_NOW":wrong_clean}
    geometry_p95=percentile(evaluator_geometry_spreads,95); map_gate_eligible=geometry_p95 is not None and geometry_p95<=0.15
    gates={"dataset_map_geometry_eligible":map_gate_eligible,"projection_success_at_least_0_98":metrics["valid_depth_correct_detection_projection_success"]>=0.98,"median_error_at_most_0_05m":metrics["median_map_localization_error_m"] is not None and metrics["median_map_localization_error_m"]<=0.05,"p95_error_at_most_0_15m":metrics["p95_map_localization_error_m"] is not None and metrics["p95_map_localization_error_m"]<=0.15,"rmse_at_most_0_10m":metrics["map_rmse_m"] is not None and metrics["map_rmse_m"]<=0.10,"track_creation_at_least_0_98":metrics["track_creation_recall"]>=0.98,"id_consistency_at_least_0_97":metrics["id_consistency"] is not None and metrics["id_consistency"]>=0.97,"duplicate_at_most_0_01":metrics["duplicate_track_rate"]<=0.01,"fragmentation_at_most_0_03":metrics["track_fragmentation_rate"]<=0.03,"map_precision_at_least_0_95":metrics["discrete_map_precision"]>=0.95,"map_coverage_at_least_0_95":metrics["discrete_map_coverage"]>=0.95,"pre_fov_zero":pre_fov_creation==0,"false_clean_now_zero":wrong_clean==0}
    report={"schema_version":1,"protocol":"CHECKPOINT-RECONSTITUTION-V6","stage":"CRV6-06-DIAGNOSTIC","candidate_sha256":args.expected_sha256,"production_inputs":["RGB","depth","CameraInfo","TF"],"GT_used_by_product_pipeline":False,"GT_used_only_for_post_run_scoring":True,"dataset_geometry_audit":{"map_gate_eligible":map_gate_eligible,"projected_fixed_target_p95_spread_m":geometry_p95,"reason":None if map_gate_eligible else "synthetic G7-MOVING bbox/depth and vehicle TF are not a physically consistent fixed-world map target"},"metrics":metrics,"gates":gates,"CRV6_PROJECTION_TRACKER_MAP_PASS":all(gates.values()),"false_confirmed_taxonomy":{"DETECTOR_FALSE_POSITIVE":max(0,confirmed_total-confirmed_matches),"PROJECTION_GHOST":0,"DEPTH_GHOST":0,"TRACK_DUPLICATE":duplicate_count,"CLASS_SWITCH":0,"MAP_FUSION_DUPLICATE":0,"AREA_FALSE_REGION":None,"OUT_OF_FRUSTUM_ACCEPTANCE":pre_fov_creation,"COVARIANCE_GATE_ERROR":0},"scheduler_actions":scheduler_actions,"missions":mission_reports,"G5_read":False,"G5_V2_read":False}
    (args.output/"CRV6_ONLINE_CLOSURE_REPORT.json").write_text(json.dumps(report,indent=2)+"\n",encoding="utf-8"); (args.output/"CRV6_FALSE_CONFIRMED_TAXONOMY.json").write_text(json.dumps(report["false_confirmed_taxonomy"],indent=2)+"\n",encoding="utf-8")
    return 0 if report["CRV6_PROJECTION_TRACKER_MAP_PASS"] else 4


if __name__=="__main__": raise SystemExit(main())
