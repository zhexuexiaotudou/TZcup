#!/usr/bin/env python3
"""Validate the immutable public W6 evidence contract before collection."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONTRACT = ROOT / "config" / "public_gazebo_dosod_evaluation_contract.json"
CLASSES = {"background": 0, "litter_cube": 1, "fallen_leaves": 2, "dust_or_soil": 3, "puddle": 4}


def load_contract(path: Path = DEFAULT_CONTRACT) -> dict[str, Any]:
    if path.is_symlink() or not path.is_file():
        raise ValueError("evaluation_contract_missing_or_linked")
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("evaluation_contract_not_object")
    ground_truth = value.get("ground_truth")
    postprocess = value.get("postprocess")
    tensor = value.get("tensor")
    parity = value.get("parity")
    deadlines = value.get("deadlines_sec")
    if (
        value.get("schema_version") != 1
        or value.get("contract_id") != "tzcup_public_gazebo_dosod_w6_evaluation_v1"
        or value.get("status") != "DRAFT_BLOCKED_UNTIL_PUBLIC_500_PLUS_100_EVIDENCE"
        or value.get("source_domain") != "public_gazebo_sensor"
        or value.get("claim_scope") != "NON_FORMAL_ENGINEERING_MODEL_GATE"
        or value.get("competition_acceptance_passed") is not False
        or value.get("product_acceptance_passed") is not False
        or value.get("classes") != CLASSES
        or not isinstance(ground_truth, dict)
        or ground_truth.get("evaluator_only") is not True
        or ground_truth.get("formal_w1_w5_enabled") is not False
        or ground_truth.get("mobile_non_formal_enabled") is not True
        or ground_truth.get("sensor_names") != ["g2_semantic_gt", "g2_instance_gt"]
        or ground_truth.get("topics") != ["/g2/semantic_gt/labels_map", "/g2/instance_gt/labels_map"]
        or not isinstance(postprocess, dict)
        or postprocess.get("official_hobot_prefilter_min_score") != 0.002
        or postprocess.get("official_hobot_nms_iou") != 0.65
        or postprocess.get("official_hobot_top_k") != 300
        or postprocess.get("official_anchor_class_selection") != "per_anchor_argmax"
        or postprocess.get("official_global_score_comparison") != "strictly_greater_than_prefilter"
        or postprocess.get("official_candidate_order") != "descending_score_then_original_anchor_index"
        or postprocess.get("official_candidate_cap") != 400
        or postprocess.get("official_nms_scope") != "class_agnostic"
        or postprocess.get("official_nms_equality") != "iou_equal_threshold_kept"
        or postprocess.get("project_adapter_class_thresholds") != {"comparison": "greater_or_equal", "values": {"litter_cube": 0.005, "fallen_leaves": 0.0025, "dust_or_soil": 0.002, "puddle": 0.003}}
        or not isinstance(tensor, dict)
        or tensor.get("layout") != "NCHW"
        or tensor.get("shape") != [1, 3, 640, 640]
        or tensor.get("dtype") != "float32"
        or tensor.get("value_range") != [0.0, 1.0]
        or tensor.get("source_color_order") != "RGB"
        or tensor.get("operation_order") != [
            "source_rgb_848x480_or_public_equivalent",
            "zero_pad_to_square",
            "opencv_inter_linear_resize_to_640",
            "float32_divide_255",
            "transpose_to_nchw",
        ]
        or tensor.get("square_padding")
        != {
            "fill": "black_zero",
            "source_extent": "max(source_width,source_height)",
            "left_top_rounding": "floor",
            "right_bottom_remainder": "source_extent-left_top-source_dimension",
        }
        or tensor.get("resize_scale") != "640/max(source_width,source_height)"
        or tensor.get("inverse_box_mapping")
        != "clip(model_coordinate/resize_scale-left_pad,source_width) and clip(model_coordinate/resize_scale-top_pad,source_height)"
        or tensor.get("inverse_roi")
        != {
            "model_box": "finite_ordered_half_open",
            "padding_only": "drop",
            "partial_overlap": "clip",
            "integer_rounding": "floor_left_top_ceil_right_bottom",
            "invalid_box": "drop_detection_not_frame",
        }
        or tensor.get("official_adapter_required") is not True
        or not isinstance(parity, dict)
        or parity.get("runner_identity_required") is not True
        or parity.get("evaluator_identity_required") is not True
        or parity.get("raw_output_math") != {"scores": {"cosine_min": 0.99, "normalized_rmse_max": 0.02}, "boxes": {"cosine_min": 0.99, "normalized_rmse_max": 0.02, "record_only": ["pixel_mae", "pixel_p95"]}, "formula": "nRMSE=RMSE(hbm-fp32)/max(RMS(fp32),1e-12); cosine=flattened_dot/(norms), both_zero=1, one_zero=0; nonfinite=BLOCKED"}
        or parity.get("behavior") != {"iou_match_min": 0.5, "class_aware": True, "prediction_order": "descending_score_then_anchor_index", "truth_tie_break": "lowest_truth_instance_or_index", "per_class_precision_recall_f1_min": 0.8, "macro_precision_recall_f1_min": 0.8, "fp_per_evaluated_frame_max": 0.2, "quantized_metric_drop_max": 0.02, "quantized_fp_per_frame_increase_max": 0.05, "require_each_class_gt": True}
        or not isinstance(deadlines, dict)
        or deadlines != {"compile": 3600, "parity": 1800, "evaluator": 600, "mobile_scene_generator": 60, "mobile_final_validation": 120, "mobile_readiness": 60, "fixture_compile": 900, "fixture_parity": 180, "fixture_evaluator": 120, "process_group_term_grace": 10}
    ):
        raise ValueError("evaluation_contract_invalid_or_not_admissible")
    return value


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    args = parser.parse_args()
    try:
        value = load_contract(args.contract)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print(f"BLOCKED:{type(exc).__name__}:{exc}")
        return 2
    print(json.dumps({"status": value["status"], "contract": str(args.contract.resolve()), "admissible": False}, sort_keys=True))
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
