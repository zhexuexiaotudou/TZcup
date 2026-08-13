#!/usr/bin/env python3
"""Freeze the one and only TGARV9 T3 official architecture decision."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--t2-report", type=Path, required=True)
    parser.add_argument("--t2-taxonomy", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    t2 = json.loads(args.t2_report.read_text())
    taxonomy = json.loads(args.t2_taxonomy.read_text())
    if t2.get("TGARV9_T2_HOLDOUT_PASS") is not False:
        raise RuntimeError("T3 decision is forbidden unless T2 HOLDOUT failed")
    if t2.get("VAL_NEW_read") or t2.get("G5_V2_read"):
        raise RuntimeError("sealed data boundary was violated before T3")
    candidates = [
        {
            "id": "co_detr",
            "official_upstream_availability": "paper/upstream project exists, but no directly installed v3.3.0 official config was found in the audited MMDetection wheel",
            "official_checkpoint_availability": "not selected; no audited installed checkpoint/config pair",
            "license": "not relied upon",
            "closed_set_fine_tuning": "possible upstream but not verified in the installed official runtime",
            "negative_frame_support": "unknown in the audited runtime",
            "small_object_capability": "multi-head collaborative training may help, but not proven against T2 taxonomy",
            "rtx4080_feasibility": "unknown/custom project risk",
            "j6_distillation_feasibility": "medium",
            "custom_ops_export_risk": "high/unknown",
            "decision": "REJECTED_UNVERIFIED_RUNTIME",
        },
        {
            "id": "dino_swin_l_5scale",
            "official_upstream_availability": "installed official MMDetection v3.3.0 config",
            "official_checkpoint_availability": "official model-zoo checkpoint",
            "license": "Apache-2.0",
            "closed_set_fine_tuning": "yes",
            "negative_frame_support": "yes after explicit loader configuration",
            "small_object_capability": "5-scale features and larger backbone",
            "rtx4080_feasibility": "poor relative to Swin-T; materially heavier than T2",
            "j6_distillation_feasibility": "low",
            "custom_ops_export_risk": "high",
            "decision": "REJECTED_CAPACITY_ONLY_BIAS_AND_DEPLOYABILITY",
        },
        {
            "id": "grounding_dino_swin_t_closed_set",
            "official_upstream_availability": "official MMDetection v3.3.0 config and documentation",
            "official_checkpoint_availability": "official converted OpenMMLab checkpoint",
            "license": "Apache-2.0",
            "closed_set_fine_tuning": "officially documented and configured",
            "negative_frame_support": "official cat config sets filter_empty_gt=false; retained explicitly",
            "small_object_capability": "multi-scale Swin-T plus grounded pretraining",
            "rtx4080_feasibility": "official docs report about 8.5 GB training memory; bounded batch=2 single RTX 4080 preflight required",
            "j6_distillation_feasibility": "medium; visual student can distill frozen vocabulary teacher",
            "custom_ops_export_risk": "high but bounded; deformable attention and BERT remain x86 research dependencies",
            "decision": "SELECTED",
        },
    ]
    report = {
        "schema_version": 1,
        "protocol": "TGARV9",
        "stage": "T3_ARCHITECTURE_DECISION",
        "triggering_t2_checkpoint": t2["selected_checkpoint"],
        "triggering_t2_metrics": t2["selected_metrics"],
        "triggering_t2_failure_taxonomy": taxonomy,
        "candidates": candidates,
        "selected_route": "grounding_dino_swin_t_closed_set",
        "selected_reason": "T2 reached perfect observation recall but failed class separation, small correct recall, actionable precision, and false/wrong CLEAN_NOW. Grounded large-scale pretraining plus language-conditioned classification directly tests domain appearance and class discrimination, while Swin-T remains feasible on RTX 4080 and is a materially different product dependency from RTMDet/DINO R50.",
        "selection_not_based_only_on_coco_map": True,
        "one_route_only": True,
        "official_sources": [
            "https://github.com/open-mmlab/mmdetection/blob/v3.3.0/configs/grounding_dino/README.md",
            "https://github.com/open-mmlab/mmdetection/blob/v3.3.0/configs/grounding_dino/grounding_dino_swin-t_finetune_8xb2_20e_cat.py",
            "https://github.com/open-mmlab/mmdetection/blob/v3.3.0/LICENSE",
        ],
        "VAL_NEW_read": False,
        "G5_V2_read": False,
        "T3_ARCHITECTURE_DECISION_FROZEN": True,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
