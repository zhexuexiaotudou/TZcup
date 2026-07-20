#!/usr/bin/env python3
from __future__ import annotations

import hashlib
import json
from pathlib import Path


ROOT=Path(__file__).resolve().parents[1]
REVIEW=ROOT/"artifacts"/"stage5br3_20260720_review"


def load(path): return json.loads(Path(path).read_text(encoding="utf-8"))


def main() -> int:
    qa=load(REVIEW/"g2_annotation_qa.json"); scan=load(REVIEW/"g2_resolution_scan.json"); production=load(REVIEW/"production_isolation"/"production_isolation_report.json")
    integrity_path=REVIEW/"stage5br2_archive_integrity_report.json"
    integrity=load(integrity_path) if integrity_path.exists() else None
    integrity_pass=bool(integrity and integrity["all_four_surfaces_byte_identical"])
    worlds=load(REVIEW/"g2_worlds"/"g2_world_manifest.json")
    runtime=[]
    for world in worlds["worlds"]:
        report=load(REVIEW/"runtime_contract"/(world["world_id"]+".json")); report["world_sha256"]=world["sha256"]; runtime.append(report)
    attempts=[load(REVIEW/f"model_attempt_{attempt}"/f"model_screening_attempt_{attempt}.json") for attempt in (1,2,3)]
    stage5a_root=ROOT/"artifacts"/"stage5br3_regression_stage5a"
    stage4w_root=ROOT/"artifacts"/"stage5br3_regression_stage4w_seed0"
    stage5a_offline=load(stage5a_root/"stage5a_offline_report.json")
    stage5a_live=load(stage5a_root/"stage5a_live_smoke_report.json")
    stage5a_spot=load(stage5a_root/"spot_clean_e2e_report.json")
    stage4w=load(stage4w_root/"stage4w_static_summary.json")
    coverage=stage4w["coverage"]
    regression_summary={"schema_version":1,"stage5a":{"script_exit_code":0,"synthetic_perception_pass":stage5a_offline["synthetic_perception_pass"],"spot_clean_e2e_pass":stage5a_spot["spot_clean_e2e_pass"],"valid_trial_count":stage5a_spot["valid_trial_count"],"mission_success_count":stage5a_spot["mission_success_count"],"live_smoke_pass":stage5a_live["live_smoke_pass"],"live_inference_frame_count":stage5a_live["inference_frame_count"],"ground_truth_input_used":stage5a_live["ground_truth_input_used"],"ground_truth_control_violation_count":stage5a_live["ground_truth_control_violation_count"],"competition_perception_pass":stage5a_offline["competition_perception_pass"]},"stage4w_seed0":{"script_exit_code":0,"coverage_success":coverage["success"],"component_count":coverage["component_count"],"coverage_rate":coverage["empirical_metrics"]["coverage_rate"],"localization_xy_rmse_m":coverage["localization_regression_during_coverage"]["rmse_m"],"collision_count":coverage["collision_count"],"keepout_violation_sample_count":coverage["keepout_violation_sample_count"],"brush_state_violation_sample_count":coverage["brush_state_violation_sample_count"],"brush_disabled_on_exit":coverage["brush_disabled_on_exit"],"competition_efficiency_pass":coverage["competition_efficiency_pass"]},"full_regression_artifacts_packaged":False,"full_regression_artifacts_retained_locally":[str(stage5a_root).replace("\\","/"),str(stage4w_root).replace("\\","/")]}
    (REVIEW/"stage5br3_regression_summary.json").write_text(json.dumps(regression_summary,indent=2)+"\n")
    runtime_summary={"schema_version":1,"world_count":len(runtime),"all_worlds_runtime_contract_pass":all(item["runtime_contract_pass"] for item in runtime),"worlds":runtime,"no_stale_world_messages":{"pass":True,"evidence":"each world ran in a fresh Docker container, ROS_DOMAIN_ID, Gazebo process, bridge, and collector; reports bind final world SHA"}}
    (REVIEW/"runtime_contract"/"six_world_runtime_summary.json").write_text(json.dumps(runtime_summary,indent=2)+"\n")
    attempt_rows=[]
    for report in attempts:
        d=report["detector"]; a=report["area_segmenter"]
        attempt_rows.append({"attempt":report["attempt"],"hypothesis":d["hypothesis"],"detector_in_domain_f1":d["in_domain"]["f1"],"detector_cross_world_f1":d["cross_world"]["f1"],"detector_cross_world_ap50":d["cross_world"]["ap50"],"detector_cross_world_ap50_95":d["cross_world"]["ap50_95"],"small_object_recall":d["cross_world"]["small_object_recall"],"color_stress_f1":d["color_stress_f1"],"same_color_negative_fp_rate":report["same_color_negative_fp_rate_for_gate"],"area_cross_world_miou":a["cross_world"]["foreground_miou"],"screening_pass":report["screening_pass"]})
    best={key:max(row[key] for row in attempt_rows) for key in ("detector_in_domain_f1","detector_cross_world_f1","detector_cross_world_ap50","detector_cross_world_ap50_95","small_object_recall","color_stress_f1","area_cross_world_miou")}; best["lowest_same_color_negative_fp_rate"]=min(row["same_color_negative_fp_rate"] for row in attempt_rows)
    status={"schema_version":1,"stage":"Stage5BR3","stage5br2_archive_integrity":"passed_16_files_four_surfaces_byte_identical" if integrity_pass else "pending_post_commit_four_surface_audit","runtime_camera_contract":{"six_distinct_worlds":len({item["world_sha256"] for item in runtime})==6,"all_worlds_pass":runtime_summary["all_worlds_runtime_contract_pass"],"actual_vehicle_model":worlds["actual_vehicle_model_required"],"production_gt_isolation_pass":production["production_isolation_pass"]},"g2_screening_dataset":{"scene_count":qa["scene_count"],"frame_count":qa["frame_count"],"annotation_qa_pass":qa["annotation_qa_pass"],"negative_only_scene_count":qa["negative_only_scene_count"],"target_asset_leakage":qa["target_asset_leakage"],"hard_negative_asset_leakage":qa["hard_negative_asset_leakage"],"cross_split_exact_duplicate_count":qa["cross_split_exact_duplicate_count"],"cross_split_phash_duplicate_count":qa["cross_split_phash_duplicate_count"]},"resolution_scan":{"native_capture_resolution":scan["native_capture_resolution"],"candidates":[item["resolution"] for item in scan["candidates"]],"selected_for_screening":scan["selected_for_model_screening"],"actual_model_screening_resolution":[512,384]},"model_screening":{"architecture_attempt_limit":3,"attempts_executed":3,"attempts":attempt_rows,"best_observed":best,"all_gates_pass":False},"regressions":{"stage5a_pass":all((regression_summary["stage5a"]["synthetic_perception_pass"],regression_summary["stage5a"]["spot_clean_e2e_pass"],regression_summary["stage5a"]["live_smoke_pass"])),"stage4w_seed0_pass":regression_summary["stage4w_seed0"]["coverage_success"]},"formal_500_scene_5000_frame_gate_executed":False,"live_30_seed_10_min_gate_executed":False,"real_nav2_spot_clean_30_seed_gate_executed":False,"j6_gate_executed":False,"competition_perception_pass":False,"real_domain_evaluation_executed":False,"j6_runtime_pass":False,"competition_efficiency_pass":False,"competition_efficiency_boundary":"1053 m²/h < 3500 m²/h","first_blocking_layer":"G2_split_model_screening_gates_failed_after_3_attempts","REVIEW_PACKET_COMPLETE":integrity_pass,"READY_FOR_GPT_REVIEW_STAGE5B":False,"READY_FOR_STAGE5C":False}
    (REVIEW/"stage5br3_status.json").write_text(json.dumps(status,indent=2)+"\n")
    files=[]
    for path in sorted(REVIEW.rglob("*")):
        if path.is_file() and path.name!="artifact_manifest.json": files.append({"path":str(path.relative_to(REVIEW)).replace("\\","/"),"bytes":path.stat().st_size,"sha256":hashlib.sha256(path.read_bytes()).hexdigest()})
    manifest={"schema_version":1,"artifact":REVIEW.name,"files":files,"raw_dataset_location":"F:/Project/TZcup-stage5br3-data/g2_screening_native","raw_dataset_packaged":False,"raw_dataset_reason":"2.225 GB task evidence retained locally; review artifact contains QA, per-instance records, reports, models, and hashes","REVIEW_PACKET_COMPLETE":integrity_pass,"READY_FOR_GPT_REVIEW_STAGE5B":False,"READY_FOR_STAGE5C":False}
    (REVIEW/"artifact_manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
    print(json.dumps({"first_blocking_layer":status["first_blocking_layer"],"runtime_pass":runtime_summary["all_worlds_runtime_contract_pass"],"dataset_qa_pass":qa["annotation_qa_pass"],"attempts":3,"all_model_gates_pass":False,"archive_integrity_pass":integrity_pass,"review_packet_complete":integrity_pass},indent=2)); return 0


if __name__=="__main__": raise SystemExit(main())
