from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _load(name: str):
    path = ROOT / "scripts" / name
    spec = importlib.util.spec_from_file_location(path.stem, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _trace(pipeline: str, frame_ids: list[str]) -> dict:
    return {
        "pipeline": pipeline,
        "checkpoint_sha256": "481374d4839e72f05fff0d6d2f6135bc7d715d5c2faf84e75d7d97ca3fc6a361",
        "class_names": ["plastic_bottle", "metal_can", "paper_litter"],
        "input_color_order": "BGR",
        "resize": [640, 480],
        "keep_ratio": False,
        "pad": None,
        "mean": [103.53, 116.28, 123.675],
        "std": [57.375, 57.12, 58.395],
        "observation_threshold": 0.05,
        "action_threshold": 0.53,
        "nms": "mmdetection_config",
        "top_k": 100,
        "frames": [{
            "frame_id": frame_id,
            "detections": [{"class_name": "metal_can", "score": 0.9, "bbox_xyxy": [1, 2, 3, 4]}],
        } for frame_id in frame_ids],
        "stage_trace": [{"correct_class": True, "depth_valid": True, "projection_success": True}],
    }


def test_equal_traces_pass_all_numeric_contracts():
    module = _load("audit_odcv5_golden_parity.py")
    ids = [f"f-{index}" for index in range(150)]
    manifest = {
        "positive_frames": 100, "negative_frames": 50,
        "exact_rgb_duplicates": 0,
        "selection_independent_of_model_output": True,
        "required_coverage_complete": True,
        "G5_SEALED_FINAL_read": False, "G5_V2_read": False,
        "frames": [{"frame_id": item} for item in ids],
    }
    traces = {name: _trace(name, ids) for name in ("P0_NATIVE", "P1_ADAPTER", "P2_PRODUCT")}
    report = module.evaluate(manifest, traces, {"pass": True})
    assert report["ODCV5_01_PASS"] is True
    assert report["decoded_agreement"] == 1.0


def test_missing_checkpoint_and_traces_fail_closed():
    module = _load("audit_odcv5_golden_parity.py")
    report = module.evaluate(
        {"positive_frames": 100, "negative_frames": 50, "exact_rgb_duplicates": 0, "selection_independent_of_model_output": True, "required_coverage_complete": True, "G5_SEALED_FINAL_read": False, "G5_V2_read": False, "frames": []},
        {},
        module.checkpoint_preflight(None),
    )
    assert report["ODCV5_01_PASS"] is False
    assert report["RUNTIME_CONTRACT_BUG"] is None
    assert "D1_B_CHECKPOINT_MISSING" in report["blockers"]


def test_rgb_contract_or_bbox_drift_is_a_runtime_bug():
    module = _load("audit_odcv5_golden_parity.py")
    ids = [f"f-{index}" for index in range(150)]
    manifest = {"positive_frames": 100, "negative_frames": 50, "exact_rgb_duplicates": 0, "selection_independent_of_model_output": True, "required_coverage_complete": True, "G5_SEALED_FINAL_read": False, "G5_V2_read": False, "frames": [{"frame_id": item} for item in ids]}
    traces = {name: _trace(name, ids) for name in ("P0_NATIVE", "P1_ADAPTER", "P2_PRODUCT")}
    traces["P2_PRODUCT"]["input_color_order"] = "RGB"
    traces["P1_ADAPTER"]["frames"][0]["detections"][0]["bbox_xyxy"][0] = 4
    report = module.evaluate(manifest, traces, {"pass": True})
    assert report["ODCV5_01_PASS"] is False
    assert report["RUNTIME_CONTRACT_BUG"] is True


def test_manifest_selector_rejects_rgb_duplicates():
    module = _load("build_odcv5_golden_manifest.py")
    rows = []
    for index in range(150):
        rows.append({
            "frame_id": str(index), "positive": index < 100,
            "discrete_classes": ["metal_can"] if index < 100 else [],
            "domains": ["metal_can"] if index < 100 else ["negative_only"],
            "sha256": {"rgb": "same"},
        })
    try:
        module.select_manifest(rows, positive_count=100, negative_count=50)
    except ValueError as error:
        assert "duplicate" in str(error)
    else:
        raise AssertionError("duplicate RGB frames were accepted")


def test_empty_negative_frames_count_as_decoded_agreement():
    module = _load("audit_odcv5_golden_parity.py")
    ids = [f"f-{index}" for index in range(150)]
    manifest = {"positive_frames": 100, "negative_frames": 50, "exact_rgb_duplicates": 0, "selection_independent_of_model_output": True, "required_coverage_complete": True, "G5_SEALED_FINAL_read": False, "G5_V2_read": False, "frames": [{"frame_id": item} for item in ids]}
    traces = {name: _trace(name, ids) for name in ("P0_NATIVE", "P1_ADAPTER", "P2_PRODUCT")}
    for trace in traces.values():
        for frame in trace["frames"][-50:]:
            frame["detections"] = []
    report = module.evaluate(manifest, traces, {"pass": True})
    assert report["decoded_agreement"] == 1.0
    assert report["ODCV5_01_PASS"] is True
