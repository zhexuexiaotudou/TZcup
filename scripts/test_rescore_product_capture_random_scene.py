import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_perception"))
from sanitation_perception.product_intermediate_capture import ProductIntermediateCapture

SPEC = importlib.util.spec_from_file_location("capture_rescore", ROOT / "scripts/rescore_product_capture_random_scene.py")
MODULE = importlib.util.module_from_spec(SPEC); SPEC.loader.exec_module(MODULE)


def _json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")


def _fixture(tmp_path):
    root = tmp_path / "capture"
    capture = ProductIntermediateCapture(root)
    capture.capture_frame(sensor="front", rgb_stamp_s=1., depth_stamp_s=1., rgb=np.zeros((8, 8, 3), dtype=np.uint8), depth=np.full((8, 8), 2., dtype=np.float32),
        camera_info={"frame_id":"front", "stamp_s":1., "width":8, "height":8, "k":[4.,0.,3.5,0.,4.,3.5,0.,0.,1.], "d":[]},
        map_from_camera=np.array([[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,-2.],[0.,0.,0.,1.]]), detections=[{"detection_index":0,"class_id":"litter_cube","confidence":.9,"xyxy":[3.,3.,4.,4.]}],
        prompt_decisions=[], prompt_detection_indices=np.array([], dtype=np.int64), prompt_masks=[], prompt_qualities=[],
        projection_diagnostics={"sample_stride":1,"ground_z_m":0.,"ground_tolerance_m":.1,"valid_depth_pixels_uv":np.array([[3,3]],dtype=np.int32),"valid_depth_m":np.array([2.],dtype=np.float32),"map_points_xyz":np.array([[0.,0.,0.]]),"ground_mask":np.array([True]),"in_grid_mask":np.array([True]),"public_free_mask":np.array([True]),"map_rows_cols":np.array([[2,2]],dtype=np.int32),"final_union_raster":np.zeros((4,4),dtype=np.uint8),"per_class_rasters":{"puddle":np.zeros((4,4),dtype=np.uint8),"fallen_leaves":np.zeros((4,4),dtype=np.uint8),"dust_or_soil":np.zeros((4,4),dtype=np.uint8)}},
        map_occupancy=np.zeros((4,4),dtype=np.int8), map_metadata={"frame_id":"map","stamp_s":1.,"width":4,"height":4,"resolution":1.,"origin_x":-2.,"origin_y":-2.})
    public = tmp_path / "public.json"; truth = tmp_path / "truth.json"; binding = tmp_path / "binding.json"
    _json(public, {"schema_version":1,"episode_id":"val-map-000-mission-000","map_id":"val-map-000"})
    _json(truth, {"schema_version":1,"namespace":"/evaluation/test","control_use_prohibited":True,"episode_id":"val-map-000-mission-000","map_id":"val-map-000","discrete_cubes":[{"object_id":"cube-0","edge_m":.03,"pose":{"x_m":0.,"y_m":0.,"z_m":.015}}],"dirt_patches":[]})
    _json(binding, {"schema_version":1,"episode_id":"val-map-000-mission-000","map_id":"val-map-000","source_commit":"a"*40,"capture_manifest_sha256":MODULE._inventory_digest(root),"public_manifest_sha256":hashlib.sha256(public.read_bytes()).hexdigest(),"evaluator_truth_sha256":hashlib.sha256(truth.read_bytes()).hexdigest(),"acceptance_session_binding":{"id":"s"},"runtime_closure_binding":{"id":"r"},"product_truth_input_used":False})
    return root, public, truth, binding


def _projection_evidence(root, binding, path):
    frame = root / "frames" / "frame-0000"
    meta, _ = MODULE._bundle(frame)
    replay = MODULE._replay_frame(root, frame, 0.5)
    target = replay["projected_targets"][0]
    row = json.loads(binding.read_text())
    _json(path, {
        "schema_version": 1,
        "namespace": "/evaluation/product_projection",
        "control_use_prohibited": True,
        "capture_manifest_sha256": row["capture_manifest_sha256"],
        "source_commit": row["source_commit"],
        "acceptance_session_binding": row["acceptance_session_binding"],
        "runtime_closure_binding": row["runtime_closure_binding"],
        "camera_frame_id": meta["camera_info"]["frame_id"],
        "map_frame_id": "map",
        "samples": [{
            "target_uuid": "11111111-1111-1111-1111-111111111111",
            "track_identity": "product-track-0",
            "frame": "frame-0000",
            "detection_index": target["detection_index"],
            "observation_stamp_s": meta["rgb_stamp_s"],
            "map_point_xyz": target["xyz"],
            "evaluator_object_id": "cube-0",
        }],
    })
    return path


def test_missing_capture_is_explicitly_blocked(tmp_path):
    root, public, truth, binding = _fixture(tmp_path)
    for child in (root / "frames").iterdir():
        child.rename(root / ("gone-" + child.name))
    report = MODULE.rescore(root, public, truth, binding)
    assert report["status"] == "BLOCKED"
    assert report["reason"] == "no_product_capture_frames"


def test_capture_truth_and_binding_are_required_and_offline_only(tmp_path):
    root, public, truth, binding = _fixture(tmp_path)
    report = MODULE.rescore(root, public, truth, binding)
    assert report["status"] == "RESCORED_OFFLINE"
    assert report["claim_boundary"]["eligible_as_formal_product_acceptance"] is False
    assert report["map_projection"]["rmse_m"] is None
    assert report["litter_cube"]["false_negative_count"] >= 0
    bad = json.loads(binding.read_text()); bad["product_truth_input_used"] = True; _json(binding, bad)
    assert MODULE.rescore(root, public, truth, binding)["status"] == "BLOCKED"


def test_capture_hash_and_episode_identity_drift_are_rejected(tmp_path):
    root, public, truth, binding = _fixture(tmp_path)
    bad = json.loads(binding.read_text()); bad["capture_manifest_sha256"] = "0" * 64; _json(binding, bad)
    assert "hash mismatch" in MODULE.rescore(root, public, truth, binding)["reason"]
    _fixture(tmp_path / "other")
    root, public, truth, binding = _fixture(tmp_path / "identity")
    row = json.loads(public.read_text()); row["map_id"] = "val-map-007"; _json(public, row)
    assert "identity mismatch" in MODULE.rescore(root, public, truth, binding)["reason"]


def test_valid_projection_evidence_computes_map_error(tmp_path):
    root, public, truth, binding = _fixture(tmp_path)
    evidence = _projection_evidence(root, binding, tmp_path / "projection.json")
    report = MODULE.rescore(root, public, truth, binding, evidence)
    assert report["status"] == "RESCORED_OFFLINE"
    assert report["map_projection"]["sample_count"] == 1
    sample = json.loads(evidence.read_text())["samples"][0]
    expected = float(np.linalg.norm(np.asarray(sample["map_point_xyz"]) - np.asarray([0., 0., .015])))
    assert abs(report["map_projection"]["rmse_m"] - expected) < 1e-9
    assert report["map_projection"]["source"] == "evaluator_only_projection_evidence"
    assert report["claim_boundary"]["truth_used_to_modify_product_output"] is False


def test_projection_evidence_rejects_cross_session_uuid_and_frame_drift(tmp_path):
    root, public, truth, binding = _fixture(tmp_path)
    evidence = _projection_evidence(root, binding, tmp_path / "projection.json")
    row = json.loads(evidence.read_text())
    row["acceptance_session_binding"] = {"id": "other"}; _json(evidence, row)
    assert "session or closure mismatch" in MODULE.rescore(root, public, truth, binding, evidence)["reason"]
    evidence = _projection_evidence(root, binding, evidence)
    row = json.loads(evidence.read_text()); row["samples"].append(dict(row["samples"][0])); _json(evidence, row)
    assert "duplicate target UUID" in MODULE.rescore(root, public, truth, binding, evidence)["reason"]
    evidence = _projection_evidence(root, binding, evidence)
    row = json.loads(evidence.read_text()); row["camera_frame_id"] = "wrong_camera"; _json(evidence, row)
    assert "camera frame mismatch" in MODULE.rescore(root, public, truth, binding, evidence)["reason"]


def test_projection_evidence_rejects_timestamp_and_hash_drift(tmp_path):
    root, public, truth, binding = _fixture(tmp_path)
    evidence = _projection_evidence(root, binding, tmp_path / "projection.json")
    row = json.loads(evidence.read_text())
    rollback = dict(row["samples"][0]); rollback["target_uuid"] = "22222222-2222-2222-2222-222222222222"; rollback["track_identity"] = "product-track-1"; rollback["observation_stamp_s"] = 0.
    row["samples"].append(rollback); _json(evidence, row)
    assert "timestamp" in MODULE.rescore(root, public, truth, binding, evidence)["reason"]
    evidence = _projection_evidence(root, binding, evidence)
    row = json.loads(evidence.read_text()); row["capture_manifest_sha256"] = "0" * 64; _json(evidence, row)
    assert "capture hash mismatch" in MODULE.rescore(root, public, truth, binding, evidence)["reason"]
