#!/usr/bin/env python3
"""Publish the fail-closed RGDRV8 result after bounded Routes A/B/C fail."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path: Path) -> dict:
    return json.loads(path.read_text())


def ref(path: Path) -> dict:
    return {"logical_path": path.as_posix(), "size_bytes": path.stat().st_size, "sha256": sha256(path)}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--artifact-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists(): raise FileExistsError(args.output)
    root = args.artifact_root
    paths = {
        "ga1": root / "ga1_forensics/GA1_FAILURE_FORENSICS_REPORT.json",
        "g8": root / "g8_data/prepared_final_v2/G8_DATASET_QA.json",
        "route_a": root / "route_a/selection/ROUTE_A_HOLDOUT_SELECTION.json",
        "route_b": root / "route_b/verifier/run/ROUTE_B_VERIFIER_REPORT.json",
        "route_c_crops": root / "route_c/crops/run/ROUTE_C_CONTEXT_CROP_REPORT.json",
        "route_c_verifier": root / "route_c/verifier/run/ROUTE_C_VERIFIER_REPORT.json",
        "route_c_specialist": root / "route_c/specialist/run/ROUTE_C_SPECIALIST_REPORT.json",
    }
    records = {name: read(path) for name, path in paths.items()}
    if records["route_a"]["RGDRV8_ROUTE_A_HOLDOUT_PASS"]: raise RuntimeError("Route A did not fail")
    if records["route_b"]["ROUTE_B_VERIFIER_HOLDOUT_PASS"]: raise RuntimeError("Route B did not fail")
    if records["route_c_verifier"]["ROUTE_C_VERIFIER_HOLDOUT_PASS"]: raise RuntimeError("Route C verifier did not fail")
    if records["route_c_specialist"]["ROUTE_C_SPECIALIST_HOLDOUT_PASS"]: raise RuntimeError("Route C specialist did not fail")
    if any(records[name].get("VAL_NEW_read") for name in ("route_a", "route_b", "route_c_verifier")): raise RuntimeError("VAL boundary violated")
    args.output.mkdir(parents=True)
    commit = subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip()
    route_a = records["route_a"]["selected_metrics"]
    route_b = records["route_b"]["holdout_metrics"]
    route_c = records["route_c_verifier"]["holdout_metrics"]
    specialist = records["route_c_specialist"]["selected_holdout"]
    next_research = {
        "schema_version": 1,
        "stage": "RGDRV8-STOPPING-CONDITION-B",
        "remaining_failure_type": ["CLOSED_SET_CLASS_SEPARATION", "HARD_NEGATIVE_REJECTION", "SMALL_SPECIALIST_PROPOSAL_RECALL"],
        "per_class_failure": {
            "Route_B": {name: {key: value for key, value in row.items() if key in ("recall", "precision")} for name, row in route_b["per_class"].items()},
            "Route_C_verifier": {name: {key: value for key, value in row.items() if key in ("recall", "precision")} for name, row in route_c["per_class"].items()},
            "Route_C_specialist_recall": specialist["per_class_recall"],
        },
        "small_failure": {"Route_C_specialist_recall": specialist["proposal_recall"], "required": 0.97},
        "hard_negative_failure": {"Route_B_background_specificity": route_b["background_specificity"], "Route_C_background_specificity": route_c["background_specificity"], "required": 0.98},
        "why_routes_failed": {
            "A": {"failed_gate": "wrong_actionable_rate", "observed": route_a["wrong_actionable_rate"], "required_max": 0.01},
            "B": {"failed_gates": ["macro_f1", "background_specificity", "paper_precision"], "observed": {"macro_f1": route_b["macro_f1"], "background_specificity": route_b["background_specificity"], "paper_precision": route_b["paper_precision"]}},
            "C": {"failed_gates": ["verifier_macro_f1", "background_specificity", "small_specialist_proposal_recall"], "observed": {"verifier_macro_f1": route_c["macro_f1"], "background_specificity": route_c["background_specificity"], "small_specialist_proposal_recall": specialist["proposal_recall"]}},
        },
        "minimum_next_architecture_hypothesis": "A deployable joint proposal-and-verification model with multi-scale ROIAlign features and explicit metric-learning/background-rejection loss; research authorization required because bounded A/B/C are exhausted.",
        "minimum_new_data_requirement": "A new world/seed/asset-isolated TRAIN development extension focused on bottle-vs-metal-vs-paper confusers and taxonomy-balanced negative-only sequences; keep current VAL_NEW sealed for the next frozen candidate or replace it before any reuse.",
        "VAL_NEW_read": False,
        "G5_V2_read": False,
    }
    (args.output / "NEXT_ARCHITECTURE_RESEARCH_REQUIRED.json").write_text(json.dumps(next_research, indent=2) + "\n")
    status = {
        "schema_version": 1,
        "protocol": "REAL-GAZEBO-DETECTOR-RECOVERY-V8",
        "source_commit": commit,
        "selected_route": None,
        "routes": {"A": "FAILED", "B": "FAILED", "C": "FAILED"},
        "MODEL_BLOCKED_INTERNAL_REAL_GAZEBO_DETECTOR": True,
        "RGDRV8_REAL_GAZEBO_DETECTOR_PASS": False,
        "SIMULATION_PRODUCT_COMPLETE": False,
        "PRODUCT_X86_PERCEPTION_READY": False,
        "VAL_NEW_read": False,
        "G5_V2_read": False,
        "downstream_gates": {name: "NOT_RUN_DEPENDENCY_BLOCKED" for name in ("tracker_map", "online_dev", "performance", "freeze", "g5v2", "moving_30seed", "spot_clean_30seed", "post_clean", "soak", "faults", "replay", "x86_release")},
        "stopping_condition": "B_ALL_BOUNDED_MODEL_ROUTES_FAILED",
    }
    (args.output / "PERCEPTION_RGDRV8_FINAL_STATUS.json").write_text(json.dumps(status, indent=2) + "\n")
    blockers = {"schema_version": 1, "blockers": [{"id": "RGDRV8-B01", "gate": "REAL_GAZEBO_DETECTOR", "classification": "MODEL_BLOCKED_INTERNAL_REAL_GAZEBO_DETECTOR", "evidence": next_research, "blocks": list(status["downstream_gates"])}]}
    (args.output / "PERCEPTION_RGDRV8_FINAL_BLOCKERS.json").write_text(json.dumps(blockers, indent=2) + "\n")
    registry = {"schema_version": 1, "selected_model": None, "candidates": {"route_a": ref(root / "route_a/training/run/epoch_4.pth"), "route_b_verifier": ref(root / "route_b/verifier/run/verifier.pt"), "route_c_verifier": ref(root / "route_c/verifier/run/verifier.pt"), "route_c_specialist": ref(root / "route_c/specialist/run/specialist.pt")}, "all_candidates_status": "REJECTED_NOT_PRODUCT_FROZEN"}
    (args.output / "PERCEPTION_RGDRV8_MODEL_REGISTRY.json").write_text(json.dumps(registry, indent=2) + "\n")
    release = {"schema_version": 1, "release_created": False, "reason": "REAL_GAZEBO_DETECTOR_GATE_FAILED", "x86_zip": None, "freeze_manifest": None, "rollback_point": commit}
    (args.output / "PERCEPTION_RGDRV8_RELEASE_MANIFEST.json").write_text(json.dumps(release, indent=2) + "\n")
    notices = "# PERCEPTION RGDRV8 Third-Party Notices\n\nNo RGDRV8 x86 product release was created. Development candidates used MMDetection/RTMDet, PyTorch, Torchvision/MobileNetV3 and OpenCV under their upstream licenses; consult repository-level third-party notices before any future release.\n"
    (args.output / "PERCEPTION_RGDRV8_THIRD_PARTY_NOTICES.md").write_text(notices)
    evidence = ["# PERCEPTION RGDRV8 Evidence Index", "", f"Source commit: `{commit}`", "", "VAL_NEW and G5_V2 were not read.", ""]
    for name, path in paths.items(): evidence.append(f"- `{name}`: `{path.as_posix()}`; SHA256 `{sha256(path)}`")
    (args.output / "PERCEPTION_RGDRV8_EVIDENCE_INDEX.md").write_text("\n".join(evidence) + "\n")
    report = f"""# REAL GAZEBO DETECTOR RECOVERY V8 REPORT

## Outcome

`MODEL_BLOCKED_INTERNAL_REAL_GAZEBO_DETECTOR=true` and `SIMULATION_PRODUCT_COMPLETE=false`. All three authorized routes were executed and failed on HOLDOUT_NEW. VAL_NEW and G5_V2 remain unread. No detector freeze, tracker/map acceptance, formal mission, soak, replay, or x86 release was attempted.

## Required answers

1. GA1 small recall 0.75 was primarily low-score correct-class detection on real Gazebo small targets, not a missing product runtime path.
2. GA1 wrong actionable 0.1875 was dominated by unknown hard negatives (5/6), with one shadow-like confuser; metal-can predictions contributed 4/6 wrong actions.
3. G8 added 121 missions and 2420 frames: TRAIN/HOLDOUT/VAL = 63/30/28 missions and 1260/600/560 frames; encounters/class = 153/150/153, 78/75/78, 70/68/70.
4. Yes. G8 QA proves zero world, seed, asset, exact-RGB and pHash overlap across splits.
5. Route A failed only the wrong-actionable gate: {route_a['wrong_actionable_rate']:.6f} > 0.01; recall and precision were {route_a['eventual_correct_class_recall']:.6f} and {route_a['actionable_precision']:.6f}.
6. Route B was enabled; background specificity was {route_b['background_specificity']:.6f}, below 0.98.
7. Route C was enabled; its specialist small proposal recall was {specialist['proposal_recall']:.6f}, below 0.97 and below the general Route B proposal small recall 0.989474.
8. No route was selected; A/B/C all failed.
9. No final independent detector metrics exist because no candidate passed HOLDOUT and VAL_NEW stayed sealed.
10. No final actionable precision pass exists.
11. No final wrong-actionable pass exists; Route A observed {route_a['wrong_actionable_rate']:.6f}.
12. No final independent small eventual recall exists.
13. Tracker correct-class confirmed recall: not run, dependency-blocked.
14. DynamicTrashMap precision/coverage: not run, dependency-blocked.
15. Full Gazebo online 24/30 mission: not run, dependency-blocked.
16. x86 sustainable Hz/P95/drop: not run, dependency-blocked.
17. Freeze ID/hash: none.
18. G5_V2: unread and not run.
19. 30-seed DynamicTrashMap: not run.
20. Spot Cleaning: not run.
21. Camera-backed CLEANED: not run.
22. 2h soak: not run.
23. MCAP replay: not run.
24. x86 release ZIP: none.
25. `SIMULATION_PRODUCT_COMPLETE=false`.
26. PR #90 must remain Draft/Open because the product gate failed.
27. J6/field are not the only remaining blockers; the internal real-Gazebo detector remains blocked.
"""
    (args.output / "REAL_GAZEBO_DETECTOR_RECOVERY_V8_REPORT.md").write_text(report)
    print(json.dumps(status, indent=2))
    return 0


if __name__ == "__main__": raise SystemExit(main())
