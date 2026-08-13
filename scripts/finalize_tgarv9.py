#!/usr/bin/env python3
"""Publish the fail-closed TGARV9 result after bounded T1/T2/T3 fail."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess


DOWNSTREAM = (
    "val_new", "real_gazebo_observation", "tracker_map", "online_dev",
    "performance", "x86_freeze", "g5_v2", "dynamic_map_30seed",
    "spot_clean_30seed", "post_clean", "soak_2h", "fault_injection",
    "mcap_replay", "x86_release",
)


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def read_ci_log(path: Path) -> str:
    payload = path.read_bytes()
    if payload.startswith((b"\xff\xfe", b"\xfe\xff")):
        return payload.decode("utf-16")
    return payload.decode("utf-8", errors="replace")


def evidence_ref(path: Path) -> dict:
    return {
        "path": str(path.resolve()),
        "size_bytes": path.stat().st_size,
        "sha256": sha256(path),
    }


def require_unread(*records: dict) -> None:
    if any(record.get("VAL_NEW_read") is not False for record in records):
        raise RuntimeError("VAL_NEW boundary is not proven unread")
    if any(record.get("G5_V2_read") is not False for record in records):
        raise RuntimeError("G5_V2 boundary is not proven unread")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--artifact-root", type=Path, required=True)
    parser.add_argument("--v8-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ci-log", type=Path, required=True)
    parser.add_argument("--pr-url", required=True)
    parser.add_argument("--pr-state", choices=("DRAFT_OPEN",), required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    root = args.artifact_root
    paths = {
        "baseline": root / "baseline/HISTORICAL_RESULTS.json",
        "boundary": root / "baseline/UNREAD_DATA_BOUNDARY.json",
        "gate_provenance": root / "baseline/GATE_PROVENANCE_V9.json",
        "g9_qa": root / "01_g9/prepared_final_v2/G9_QA.json",
        "g9_manifest": root / "01_g9/prepared_final_v2/G9_HOLDOUT_MANIFEST.json",
        "g9_tubes": root / "01_g9/prepared_final_v2/G9_TARGET_TUBES.json",
        "t1": root / "02_t1/T1_G9_HOLDOUT_REPORT.json",
        "t1_taxonomy": root / "02_t1/T1_FAILURE_TAXONOMY.json",
        "t2_train": root / "03_t2/training/run/T2_TRAIN_REPORT.json",
        "t2": root / "03_t2/selection/T2_G9_HOLDOUT_REPORT.json",
        "t2_taxonomy": root / "03_t2/selection/T2_FAILURE_TAXONOMY.json",
        "t3_decision": root / "04_t3/decision/T3_ARCHITECTURE_DECISION.json",
        "t3_train": root / "04_t3/training/run/T3_TRAIN_REPORT.json",
        "t3": root / "04_t3/selection/T3_G9_HOLDOUT_REPORT.json",
        "t3_taxonomy": root / "04_t3/selection/T3_FAILURE_TAXONOMY.json",
        "ci": args.ci_log,
    }
    missing = [name for name, path in paths.items() if not path.is_file()]
    if missing:
        raise FileNotFoundError(f"required evidence missing: {missing}")
    records = {name: read(path) for name, path in paths.items() if name != "ci"}
    if not records["g9_qa"]["G9_PASS"]:
        raise RuntimeError("G9 did not pass")
    if records["t1"]["TGARV9_T1_HOLDOUT_PASS"]:
        raise RuntimeError("T1 did not fail")
    if records["t2"]["TGARV9_T2_HOLDOUT_PASS"]:
        raise RuntimeError("T2 did not fail")
    if records["t3"]["TGARV9_T3_HOLDOUT_PASS"]:
        raise RuntimeError("T3 did not fail")
    require_unread(records["boundary"], records["g9_manifest"], records["t1"], records["t2"], records["t3"])
    if "development workflow fast validation passed" not in read_ci_log(args.ci_log):
        raise RuntimeError("full local CI pass evidence missing")

    commit = subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip()
    t1 = records["t1"]["selected_metrics"]
    t2 = records["t2"]["selected_metrics"]
    t3 = records["t3"]["selected_metrics"]
    t3_detector = next(
        row["detector_diagnostics"] for row in records["t3"]["candidates"]
        if row["checkpoint"] == records["t3"]["selected_checkpoint"]
    )
    qa = records["g9_qa"]
    tube_count = len(records["g9_tubes"]["tubes"])
    small_count = sum(qa["small_first_visible_counts_by_class"].values())
    selected_t3_path = root / "04_t3/training/run" / records["t3"]["selected_checkpoint"]
    if sha256(selected_t3_path) != records["t3"]["selected_checkpoint_sha256"]:
        raise RuntimeError("selected T3 checkpoint hash mismatch")
    args.output.mkdir(parents=True)

    downstream = {name: "NOT_RUN_DEPENDENCY_BLOCKED" for name in DOWNSTREAM}
    status = {
        "schema_version": 1,
        "protocol": "TEMPORAL-GEOMETRY-ARCHITECTURE-RECOVERY-V9",
        "source_commit": commit,
        "routes": {"T1": "FAILED", "T2": "FAILED", "T3": "FAILED"},
        "selected_route": None,
        "best_failed_candidate": {
            "route": "T3",
            "architecture": "official_mmdetection_grounding_dino_swin_t_closed_set",
            "checkpoint": records["t3"]["selected_checkpoint"],
            "checkpoint_sha256": records["t3"]["selected_checkpoint_sha256"],
            "metrics": t3,
        },
        "TGARV9_ALL_ROUTES_EXHAUSTED": True,
        "MODEL_BLOCKED_INTERNAL": True,
        "TGARV9_VAL_NEW_PASS": False,
        "PRODUCT_X86_PERCEPTION_READY": False,
        "SIMULATION_PRODUCT_COMPLETE": False,
        "VAL_NEW_read": False,
        "G5_V2_read": False,
        "downstream_gates": downstream,
        "stopping_condition": "B_T1_T2_T3_ALL_FAILED",
        "pr": {"url": args.pr_url, "state": args.pr_state},
    }
    write_json(args.output / "PERCEPTION_TGARV9_FINAL_STATUS.json", status)

    blocker = {
        "schema_version": 1,
        "blockers": [{
            "id": "TGARV9-B01",
            "gate": "G9_HOLDOUT_PRODUCT_CONFIRMATION",
            "classification": "MODEL_BLOCKED_INTERNAL",
            "best_failed_route": "T3",
            "observed": {
                "eventual_correct_class_recall": t3["eventual_correct_class_recall"],
                "small_eventual_correct_class_recall": t3["small_eventual_correct_class_recall"],
                "confirmed_actionable_precision": t3["confirmed_actionable_precision"],
                "wrong_confirmed_actionable_rate": t3["wrong_confirmed_actionable_rate"],
                "clean_opportunity_miss": t3["clean_opportunity_miss"],
                "false_CLEAN_NOW": t3["false_CLEAN_NOW"],
                "wrong_class_CLEAN_NOW": t3["wrong_class_CLEAN_NOW"],
            },
            "required": {
                "eventual_correct_class_recall_min": 0.95,
                "small_eventual_correct_class_recall_min": 0.90,
                "confirmed_actionable_precision_min": 0.95,
                "wrong_confirmed_actionable_rate_max": 0.01,
                "clean_opportunity_miss_max": 0.02,
                "false_CLEAN_NOW": 0,
                "wrong_class_CLEAN_NOW": 0,
            },
            "blocks": list(DOWNSTREAM),
            "next_authorization_required": "A new protocol with new TRAIN development data and a materially new class-separation/track-association hypothesis; current VAL_NEW must remain sealed or be replaced before reuse.",
        }],
    }
    write_json(args.output / "PERCEPTION_TGARV9_FINAL_BLOCKERS.json", blocker)

    registry = {
        "schema_version": 1,
        "selected_product_model": None,
        "all_candidates_status": "REJECTED_NOT_PRODUCT_FROZEN",
        "best_failed_candidate": evidence_ref(selected_t3_path),
        "candidate_groups": {
            "T1": {"model": "RGDRV8 Route A frozen input", "status": "REJECTED_G9"},
            "T2": {"model": "official MMDetection DINO R50 4-scale improved", "status": "REJECTED_G9", "selected_checkpoint": records["t2"]["selected_checkpoint"], "selected_checkpoint_sha256": records["t2"]["selected_checkpoint_sha256"]},
            "T3": {"model": "official MMDetection Grounding-DINO Swin-T closed-set", "status": "REJECTED_G9", "selected_checkpoint": records["t3"]["selected_checkpoint"], "selected_checkpoint_sha256": records["t3"]["selected_checkpoint_sha256"]},
        },
    }
    write_json(args.output / "PERCEPTION_TGARV9_MODEL_REGISTRY.json", registry)
    write_json(args.output / "PERCEPTION_TGARV9_RELEASE_MANIFEST.json", {
        "schema_version": 1,
        "release_created": False,
        "reason": "ALL_BOUNDED_ROUTES_FAILED_G9_HOLDOUT",
        "x86_zip": None,
        "freeze_manifest": None,
        "rollback_point": commit,
        "evidence_retained_at": str(root.resolve()),
    })
    (args.output / "PERCEPTION_TGARV9_THIRD_PARTY_NOTICES.md").write_text(
        "# PERCEPTION TGARV9 Third-Party Notices\n\n"
        "No x86 product release was created. Development candidates used OpenMMLab MMDetection/MMCV/MMEngine, PyTorch, Transformers, and the BERT base uncased assets. The selected T3 research route and audited official checkpoint/config are Apache-2.0. Preserve the upstream license and model notices before any future redistribution.\n",
        encoding="utf-8",
    )

    refs = {name: evidence_ref(path) for name, path in paths.items()}
    lines = [
        "# PERCEPTION TGARV9 Evidence Index", "", f"Source commit: `{commit}`", "",
        "Stopping condition B is proven: T1, T2 and T3 all failed G9 HOLDOUT. VAL_NEW and G5_V2 were not read.", "",
    ]
    for name, reference in refs.items():
        lines.append(f"- `{name}`: `{reference['path']}`; {reference['size_bytes']} bytes; SHA256 `{reference['sha256']}`")
    (args.output / "PERCEPTION_TGARV9_EVIDENCE_INDEX.md").write_text("\n".join(lines) + "\n", encoding="utf-8")

    report = f"""# TEMPORAL GEOMETRY ARCHITECTURE RECOVERY V9 REPORT

## Outcome

`TGARV9_ALL_ROUTES_EXHAUSTED=true`, `MODEL_BLOCKED_INTERNAL=true`, and `SIMULATION_PRODUCT_COMPLETE=false`. T1, official DINO-family T2, and the one authorized Grounding-DINO T3 all failed the strict G9 HOLDOUT product gate. `VAL_NEW_read=false` and `G5_V2_read=false`; downstream product gates were therefore not run. No x86 model was frozen or released.

## Required answers

1. The historical RGDRV8 Route A result contains 40 wrong actionable frame predictions among 1,750 (2.2857%). A matched per-prediction counterfactual through T1 was not available, so an exact "how many filtered" claim would be invalid. On independent G9, T1 reduced wrong-class confirmed actions to 0, but still produced 70 false `CLEAN_NOW`; this is not a pass and is not presented as the same population.
2. For the best failed T3 candidate at the frozen 0.05 raw diagnostic threshold, raw detector wrong/unmatched fraction was `{1.0 - t3_detector['precision']:.6f}` (precision `{t3_detector['precision']:.6f}`); confirmed wrong-class actionable rate was `{t3['wrong_confirmed_actionable_rate']:.6f}`. These are different evaluation units.
3. G9 has {qa['mission_count']} missions, {tube_count} target tubes, {small_count} small-first-visible targets, and {qa['negative_only_missions']} negative-only missions ({qa['frame_count']} frames). It passed every mandatory QA gate; 23 is the required minimum while 24 was only recommended.
4. T1 did not pass. Temporal/geometry retained observation recall `{t1['eventual_observation_recall']:.6f}` but correct-class recall `{t1['eventual_correct_class_recall']:.6f}`, small recall `{t1['small_eventual_correct_class_recall']:.6f}`, precision `{t1['confirmed_actionable_precision']:.6f}`, miss `{t1['clean_opportunity_miss']:.6f}`, and 70 false clean actions failed.
5. Yes. T2 used the official MMDetection v3.3.0 DINO R50 4-scale improved config/checkpoint, official checkpoint SHA256 `6f47a9136abcba7b6293a9f1bf4870e4c604276d0d11c4c84ceabfda9ea14245`, under Apache-2.0; six checkpoints and 400 negative frames were retained.
6. Yes. The sole T3 route was official MMDetection Grounding-DINO Swin-T closed-set fine-tuning, selected because grounded pretraining and language-conditioned classification directly tested T2's class-separation/domain failure while remaining feasible on the RTX 4080. Official checkpoint SHA256 was `822d7e9db9ce6ff2119b72dc6e78606a1b0a2c307234798adf0cab50f1b424e3`.
7. No product route was selected. T3 epoch 6 is only the best failed candidate; it is explicitly rejected and not frozen.
8. Yes. VAL_NEW remained unread through selection and remains unread at stopping condition B.
9. VAL_NEW raw detector P/R/F1/AP: not available because VAL_NEW was not read.
10. VAL_NEW confirmed precision/recall: not available because VAL_NEW was not read.
11. No. Best failed T3 wrong confirmed actionable was `{t3['wrong_confirmed_actionable_rate']:.6f}`, above 0.01.
12. No. Best failed T3 small eventual correct recall was `{t3['small_eventual_correct_class_recall']:.6f}`, below 0.90.
13. Best failed T3 clean opportunity miss was `{t3['clean_opportunity_miss']:.6f}`, above 0.02.
14. Best failed T3 `OBSERVE_AGAIN` cost was `{t3['OBSERVE_AGAIN_per_target']:.6f}` per target, maximum 2 (a P95 was not separately emitted), {t3['extra_travel_distance_m']:.2f} m and {t3['extra_time_s']:.1f} s.
15. Tracker correct-class confirmed recall: not run, dependency-blocked by G9 HOLDOUT.
16. DynamicTrashMap precision/coverage/RMSE: not run, dependency-blocked.
17. Full Gazebo online 24/30 mission: not run, dependency-blocked.
18. x86 sustainable Hz/P95/drop: not run; deployability pre-screen is forbidden before a HOLDOUT pass.
19. Freeze ID/hash: none.
20. G5_V2 remained unread and was not run.
21. Formal 30-seed acceptance: not run, dependency-blocked.
22. Spot Cleaning zero wrong-target/false-candidate: not run, dependency-blocked.
23. Camera-backed `CLEANED`: not run, dependency-blocked.
24. Two-hour soak: not run, dependency-blocked.
25. MCAP replay: not run, dependency-blocked.
26. x86 release ZIP path/hash: none.
27. `SIMULATION_PRODUCT_COMPLETE=false`.
28. PR #90 is Draft/Open at `{args.pr_url}`; it must not be marked Ready or merged while the product gate is failed.
29. No. J6/field are not the only remaining blockers; an internal real-Gazebo class-separation and safe-confirmation blocker remains.

## Best failed candidate

T3 epoch 6 (`{records['t3']['selected_checkpoint_sha256']}`) reached raw AP50:95 `{t3_detector['AP50_95']:.6f}`, observation recall `{t3['eventual_observation_recall']:.6f}`, correct-class recall `{t3['eventual_correct_class_recall']:.6f}`, small recall `{t3['small_eventual_correct_class_recall']:.6f}`, confirmed precision `{t3['confirmed_actionable_precision']:.6f}`, wrong confirmed rate `{t3['wrong_confirmed_actionable_rate']:.6f}`, five false clean actions and five wrong-class clean actions. This demonstrates strong localization without product-safe target confirmation.
"""
    (args.output / "TEMPORAL_GEOMETRY_ARCHITECTURE_RECOVERY_V9_REPORT.md").write_text(report, encoding="utf-8")
    print(json.dumps(status, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
