#!/usr/bin/env python3
"""Publish the mandatory fail-closed CRCRV11 final evidence set."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--source-commit", required=True)
    parser.add_argument("--pr", default="https://github.com/zhexuexiaotudou/TZcup/pull/91")
    args = parser.parse_args()
    root = args.root.resolve()
    output = root / "final"
    output.mkdir(parents=True, exist_ok=True)
    paths = {
        "baseline": root / "baseline" / "REPO_BASELINE.json",
        "boundary": root / "baseline" / "SEALED_BOUNDARY.json",
        "balance": root / "forensic" / "V11_CLASS_BALANCE_AUDIT.json",
        "background": root / "forensic" / "V11_BACKGROUND_UNIQUENESS_AUDIT.json",
        "views": root / "forensic" / "V11_VIEW_DISTRIBUTION_AUDIT.json",
        "labels": root / "forensic" / "V11_BACKGROUND_LABEL_AUDIT.json",
        "parity": root / "forensic" / "V11_CROP_PIXEL_PARITY.json",
        "root_cause": root / "forensic" / "V11_ROOT_CAUSE_DECISION.json",
        "five_view": root / "five_view" / "V11_FIVE_VIEW_MATRIX.json",
        "degradation": root / "five_view" / "V11_VIEW_DEGRADATION.json",
        "c11_qa": root / "c11_data" / "C11_DATA_QA.json",
        "c11_counts": root / "c11_data" / "C11_CLASS_COUNTS.json",
        "c11_background": root / "c11_data" / "C11_BACKGROUND_TAXONOMY.json",
        "c11_stats": root / "c11_data" / "C11_PAIR_STATS.json",
        "r1": root / "r1" / "CRCRV11_R1_REPORT.json",
        "r2": root / "r2" / "CRCRV11_R2_REPORT.json",
        "r3": root / "r3" / "CRCRV11_R3_REPORT.json",
    }
    for path in paths.values():
        if not path.is_file():
            raise FileNotFoundError(path)
    data = {name: load_json(path) for name, path in paths.items()}
    if data["c11_qa"].get("C11_DATA_PASS") is not True:
        raise RuntimeError("cannot finalize without C11_DATA_PASS=true")
    if any(data[name].get(f"CRCRV11_{name.upper()}_PASS") is not False for name in ("r1", "r2", "r3")):
        raise RuntimeError("stop condition B requires explicit R1/R2/R3 failures")
    sealed_fields = ("G10_DEV_VAL_SEALED_read", "VAL_NEW_read", "G5_V2_read")
    if any(data["boundary"].get(field) is not False for field in sealed_fields):
        raise RuntimeError("sealed boundary changed before stop condition B")

    r1, r2, r3 = data["r1"], data["r2"], data["r3"]
    downstream = {
        name: "dependency_blocked_not_executed" for name in (
            "classifier_holdout_product_gate", "action_verifier", "integrated_holdout",
            "dev_val", "val_new", "tracker_dynamic_trash_map", "full_gazebo_online",
            "performance", "x86_freeze", "g5_v2", "moving_30seed", "spot_clean_30seed",
            "post_clean", "soak_2h", "fault_matrix", "mcap_replay", "x86_release",
        )
    }
    status = {
        "schema_version": 1, "protocol": "CLOSE-RANGE-CLASSIFIER-CONTRACT-RECOVERY-V11",
        "stage": "CRCRV11-FINAL", "source_commit": args.source_commit,
        "stop_condition": "B_R1_R2_R3_ALL_FAILED", "CRCRV11_R1_PASS": False,
        "CRCRV11_R2_PASS": False, "CRCRV11_R3_PASS": False,
        "CLOSE_RANGE_CLASSIFIER_CONTRACT_BLOCKED": True, "MODEL_BLOCKED_INTERNAL": True,
        "SIMULATION_PRODUCT_COMPLETE": False, "PRODUCT_X86_PERCEPTION_READY": False,
        **{field: False for field in sealed_fields}, "downstream": downstream,
        "pr": {"number": 91, "url": args.pr, "expected_state": "open_draft"},
    }
    blockers = {
        "schema_version": 1, "protocol": "CRCRV11", "source_commit": args.source_commit,
        "primary_blockers": [
            {
                "id": "TARGET_CLASS_DOMAIN_GENERALIZATION",
                "severity": "critical",
                "evidence": {
                    "R1_holdout_target_macro_f1": r1["holdout"]["candidate_fused"]["target_macro_f1"],
                    "R1_metal_recall": r1["holdout"]["candidate_fused"]["per_class"]["metal_can"]["recall"],
                    "R2_stage2_macro_f1": r2["stage2"]["holdout"]["macro_f1"],
                    "R3_bottle_recall": r3["holdout"]["metrics"]["per_class"]["plastic_bottle"]["recall"],
                },
            },
            {
                "id": "BACKGROUND_CONTRACT_ROUTE_INSTABILITY",
                "severity": "high",
                "evidence": {
                    "V10_background_unique": data["background"]["train_unique_crops"],
                    "V10_sampler_repeat_factor": data["background"]["sampler_expected_repeat_factor"],
                    "R1_background_specificity": r1["holdout"]["candidate_fused"]["background_specificity"],
                    "R2_background_specificity": r2["binary"]["holdout"]["background_specificity"],
                    "R3_background_specificity": r3["holdout"]["metrics"]["background_specificity"],
                },
            },
        ],
        "forbidden_next_actions": ["classifier R4/R5", "new detector search", "sealed-data tuning", "lowering product gates"],
    }
    registry = {
        "schema_version": 1, "protocol": "CRCRV11", "source_commit": args.source_commit,
        "models": [
            {"route": "R1", "status": "failed", "checkpoint_sha256": r1["selected_checkpoint_sha256"],
             "candidate_macro_f1": r1["holdout"]["candidate_fused"]["macro_f1"]},
            {"route": "R2_BINARY", "status": "failed", "checkpoint_sha256": r2["binary"]["checkpoint_sha256"],
             "background_specificity": r2["binary"]["holdout"]["background_specificity"]},
            {"route": "R2_STAGE2", "status": "failed", "checkpoint_sha256": r2["stage2"]["checkpoint_sha256"],
             "macro_f1": r2["stage2"]["holdout"]["macro_f1"]},
            {"route": "R3", "status": "failed", "checkpoint_sha256": r3["checkpoint_sha256"],
             "candidate_macro_f1": r3["holdout"]["metrics"]["macro_f1"]},
        ],
        "selected_product_route": None, "frozen_product_model": None,
    }
    release = {
        "schema_version": 1, "protocol": "CRCRV11", "source_commit": args.source_commit,
        "release_bundle_created": False, "release_zip": None, "release_sha256": None,
        "reason": "R1/R2/R3 all failed; product and release gates remain locked",
        "MODEL_FREEZE_X86_created": False, "RELEASE_BUNDLE_PASS": False,
    }
    write_json(output / "PERCEPTION_CRCRV11_FINAL_STATUS.json", status)
    write_json(output / "PERCEPTION_CRCRV11_FINAL_BLOCKERS.json", blockers)
    write_json(output / "PERCEPTION_CRCRV11_MODEL_REGISTRY.json", registry)
    write_json(output / "PERCEPTION_CRCRV11_RELEASE_MANIFEST.json", release)
    evidence_lines = [
        "# CRCRV11 Evidence Index", "", f"Source commit: `{args.source_commit}`", "",
        "All large crops, checkpoints, and evaluated rows remain outside Git in the immutable task evidence root.", "",
        "| Evidence | SHA-256 |", "|---|---|",
    ]
    for name, path in paths.items():
        evidence_lines.append(f"| `{path.relative_to(root).as_posix()}` | `{sha256(path)}` |")
    evidence_lines.extend(["", "Sealed DEV_VAL, VAL_NEW, G5_V2, formal 30-seed, soak, replay, freeze, and release were not executed.", ""])
    (output / "PERCEPTION_CRCRV11_EVIDENCE_INDEX.md").write_text("\n".join(evidence_lines), encoding="utf-8")
    (output / "PERCEPTION_CRCRV11_THIRD_PARTY_NOTICES.md").write_text(
        "# CRCRV11 Third-Party Notices\n\n"
        "R1/R2/R3 use torchvision ConvNeXt-Tiny with official ImageNet weights. "
        "OpenCV is used for deterministic crop and pHash operations. Existing repository "
        "license and model notices remain authoritative. No release bundle was produced.\n",
        encoding="utf-8",
    )

    balance = data["balance"]
    five = data["five_view"]
    report = f"""# Close-Range Classifier Contract Recovery V11 Report

Stop condition: **B — R1/R2/R3 all failed**. `SIMULATION_PRODUCT_COMPLETE=false` and all downstream product gates remain unexecuted.

1. V10 TRAIN unique crops: `{json.dumps(balance['train']['unique_crops_by_class'])}`; HOLDOUT: `{json.dumps(balance['holdout']['unique_crops_by_class'])}`.
2. Background unique sources/crops: `{data['background']['train_unique_source_frames']}/{data['background']['train_unique_crops']}`; sampler repeat factor `{data['background']['sampler_expected_repeat_factor']:.4f}`.
3. Near-miss background label noise count: `{data['labels']['near_miss_020_050_count']}` at the frozen threshold; C11 ambiguous ignored TRAIN/HOLDOUT: `{data['c11_stats']['train_ambiguous_ignored']}/{data['c11_stats']['holdout_ambiguous_ignored']}`.
4. TRAIN runtime-faithful positive view fraction: `{data['views']['positive_runtime_faithful_fraction']:.4f}`.
5. A/B/C/D/E target macro-F1: `{', '.join(f'{view}={five["views"][view]["macro_f1"]:.4f}' for view in five['views'])}`.
6. F/G/H/I background specificity: `{', '.join(f'{view}={five["background_controls"][view]["background_specificity"]:.4f}' for view in five['background_controls'])}`.
7. Root causes supported: background scarcity, memorization, TRAIN/runtime view mismatch, over-strong V10 augmentation, context metadata mismatch, and target class confusion. Pixel/channel bug is unsupported by 100/100 parity; near-miss noise is unsupported in observed frozen proposals.
8. C11 background bank unique tight crops: `{data['c11_background']['unique_background_tight_crops']}`.
9. R1 candidate macro-F1/background specificity: `{r1['holdout']['candidate_fused']['macro_f1']:.4f}/{r1['holdout']['candidate_fused']['background_specificity']:.4f}`.
10. R2 binary specificity/litter recall: `{r2['binary']['holdout']['background_specificity']:.4f}/{r2['binary']['holdout']['litter_recall']:.4f}`.
11. R3 evidence: yes; tight/context complementary correctness rate `{data['degradation']['target_complementary_correctness_rate']:.4f}`.
12. Final classifier route: none; R1, R2, and R3 failed.
13. Best formal candidate-level classifier macro-F1: R1 `{r1['holdout']['candidate_fused']['macro_f1']:.4f}`, R2 `{r2['combined_holdout']['macro_f1']:.4f}`, R3 `{r3['holdout']['metrics']['macro_f1']:.4f}`; none passed.
14. ActionVerifier wrong actionable: NOT_EXECUTED dependency-blocked.
15. False/wrong CLEAN_NOW: NOT_EXECUTED dependency-blocked.
16. DEV_VAL one-shot: not accessed.
17. VAL_NEW used for training: no; not accessed.
18. Tracker far-to-close continuity: NOT_EXECUTED.
19. DynamicTrashMap precision/coverage/RMSE: NOT_EXECUTED.
20. Full Gazebo Online: NOT_EXECUTED.
21. x86 Hz/P95/drop: NOT_EXECUTED.
22. Freeze ID/hash: not created.
23. G5_V2 one-shot: not accessed.
24. 30-seed: NOT_EXECUTED.
25. Spot Cleaning zero wrong cleans: NOT_EXECUTED.
26. Camera-backed CLEANED: NOT_EXECUTED.
27. 2h soak: NOT_EXECUTED.
28. MCAP replay: NOT_EXECUTED.
29. Release ZIP/hash: not created.
30. `SIMULATION_PRODUCT_COMPLETE=false`.
31. PR #91: open Draft at finalization; merge is forbidden while the classifier gate is red.
"""
    (output / "CLOSE_RANGE_CLASSIFIER_CONTRACT_RECOVERY_V11_REPORT.md").write_text(report, encoding="utf-8")
    print(json.dumps(status, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
