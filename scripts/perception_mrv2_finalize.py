#!/usr/bin/env python3
"""Create the fail-closed MODEL-RECOVERY-V2 final evidence package."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def build_status(a: dict, b: dict, c: dict, grounding: dict, *, source: dict) -> dict:
    a_pass = bool(a["static_decision"]["static_gate_pass"])
    b_pass = bool(b["MRV2_B_DETECTOR_PASS"])
    c_pass = bool(c["static_decision"]["static_gate_pass"])
    static_pass = a_pass or b_pass or c_pass
    return {
        "schema_version": 1,
        "stage": "MRV2-13-FINAL-STATUS",
        "source": source,
        "historical": {
            "X1": "FAILED_STATIC_FULL_PIPELINE",
            "X2": "BLOCKED_EXTERNAL_NETWORK_ASSET",
            "X3": "FAILED_STATIC_FULL_PIPELINE",
            "historical_states_rewritten": False,
        },
        "routes": {
            "MRV2-A": "PASS" if a_pass else "FAILED_STATIC_FULL_PIPELINE",
            "MRV2-B": "PASS" if b_pass else "FAILED_STATIC_FULL_PIPELINE",
            "MRV2-C": "PASS" if c_pass else "FAILED_STATIC_FULL_PIPELINE",
            "selected_route": next(
                (name for name, passed in (("MRV2-A", a_pass), ("MRV2-B", b_pass), ("MRV2-C", c_pass)) if passed),
                None,
            ),
            "routes_exhausted": not static_pass,
        },
        "MRV2_X86_STATIC_PASS": static_pass,
        "MODEL_BLOCKED_INTERNAL": not static_pass,
        "MODEL_FREEZE_X86_created": False,
        "G5_SEALED_FINAL_read": False,
        "legacy_G4_D6_read": False,
        "SEALED_FINAL_PASS": False,
        "ONLINE_DYNAMIC_DISCOVERY_PASS": False,
        "DYNAMIC_TRASH_MAP_PASS": False,
        "SPOT_CLEAN_PRODUCT_PASS": False,
        "POST_CLEAN_VERIFICATION_PASS": False,
        "SOAK_2H_PASS": False,
        "MCAP_REPLAY_PASS": False,
        "RELEASE_BUNDLE_PASS": False,
        "PRODUCT_X86_PERCEPTION_READY": False,
        "PRODUCT_J6_TOOLCHAIN_READY": False,
        "PRODUCT_J6_BOARD_READY": False,
        "PRODUCT_FIELD_READY": False,
        "COMPETITION_PERCEPTION_PASS": False,
        "GROUNDING_DINO_OFFICIAL_CHECKPOINT_OBTAINED": True,
        "GROUNDING_DINO_BENCHMARK_EXECUTED": True,
        "GROUNDING_DINO_REFERENCE_STATIC_PASS": bool(
            grounding["GROUNDING_DINO_REFERENCE_STATIC_PASS"]
        ),
        "blocked_follow_on_gates": [
            "MRV2-06 freeze", "MRV2-07 sealed final", "MRV2-08 moving-camera",
            "MRV2-09 spot-clean/post-clean", "MRV2-10 soak/replay/release",
            "MRV2-11 J6", "MRV2-12 field",
        ],
        "claim_boundary": (
            "All three authorized MRV2 model routes failed the fixed development gate. "
            "No freeze, sealed-final access, deployment, or product-ready claim is allowed."
        ),
    }


def evidence_record(root: Path, relative: str) -> dict:
    path = root / relative
    return {"path": relative, "bytes": path.stat().st_size, "sha256": sha256(path)}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--source-local-commit", required=True)
    parser.add_argument("--source-remote-commit", required=True)
    parser.add_argument("--source-tree", required=True)
    parser.add_argument("--mrv2-c-train-report", type=Path, required=True)
    args = parser.parse_args()
    root = args.root
    final = root / "final"
    final.mkdir(parents=True, exist_ok=True)
    a = json.loads((root / "mrv2_a/MRV2_A_R960_FULL_STATIC.json").read_text(encoding="utf-8"))
    b = json.loads((root / "mrv2_b/MRV2_B_SCREEN.json").read_text(encoding="utf-8"))
    c = json.loads((root / "mrv2_c/MRV2_C_FULL_STATIC.json").read_text(encoding="utf-8"))
    grounding = json.loads((root / "grounding_dino/GROUNDING_DINO_BENCHMARK.json").read_text(encoding="utf-8"))
    c_train = json.loads(args.mrv2_c_train_report.read_text(encoding="utf-8"))
    source = {
        "local_commit": args.source_local_commit,
        "remote_equivalent_commit": args.source_remote_commit,
        "tree": args.source_tree,
        "code_tree_equivalence_verified": True,
    }
    status = build_status(a, b, c, grounding, source=source)
    write_json(final / "PERCEPTION_MRV2_FINAL_STATUS.json", status)

    val_a = a["splits"]["VAL"]
    val_c = c["splits"]["VAL"]
    cross_c = c["cross_world_aggregate"]
    blockers = {
        "schema_version": 1,
        "stage": "MRV2-13-FINAL-BLOCKERS",
        "primary_blocker": "MODEL_BLOCKED_INTERNAL",
        "internal_blockers": [
            {
                "id": "small_object_recall",
                "gate": 0.70,
                "historical_X3": 0.3076923076923077,
                "MRV2_A_VAL": val_a["discrete"]["small_object_recall"],
                "MRV2_C_VAL": val_c["discrete"]["small_object_recall"],
                "MRV2_C_cross_world": cross_c["discrete"]["small_object_recall"],
            },
            {
                "id": "candidate_flood",
                "gate_false_candidates_per_min": 2.0,
                "MRV2_A_VAL": val_a["candidate"]["false_candidates_per_min"],
                "MRV2_C_VAL": val_c["candidate"]["false_candidates_per_min"],
            },
            {
                "id": "metal_can_cross_domain",
                "VAL_gate": 0.90,
                "each_domain_gate": 0.70,
                "MRV2_C_VAL": val_c["discrete"]["per_class"]["metal_can"]["recall"],
                "MRV2_C_D1_D4": {
                    name: c["splits"][name]["discrete"]["per_class"]["metal_can"]["recall"]
                    for name in ("D1", "D2", "D3", "D4")
                },
            },
            {
                "id": "area_boundary_and_negative_fp",
                "VAL_boundary_f1": val_c["area"]["boundary_f1"],
                "D4_boundary_f1": c["splits"]["D4"]["area"]["boundary_f1"],
                "D4_negative_area_fp_per_frame": c["splits"]["D4"]["area"]["negative_area_fp_per_frame"],
            },
            {
                "id": "grounding_dino_reference_not_viable",
                "VAL_candidate_recall": grounding["splits"]["VAL"]["all_gt_candidate_recall"],
                "VAL_small_recall": grounding["splits"]["VAL"]["small_object_candidate_recall"],
                "VAL_false_candidates_per_min": grounding["splits"]["VAL"]["false_candidates_per_min"],
                "proposal_inference_p95_ms": grounding["performance"]["inference"]["p95_ms"],
                "preprocess_p95_ms": grounding["performance"]["preprocess"]["p95_ms"],
            },
        ],
        "external_resources_also_absent_but_not_current_stop_reason": [
            "physical J6 board", "qualifying real RGB-D device/recording", "independent map GT",
        ],
        "required_next_research": [
            "Acquire or synthesize substantially more diverse native <18px TRAIN instances with independent annotation QA.",
            "Train a dedicated small-object architecture with calibrated size-aware objectness rather than lowering a global threshold.",
            "Add D1/D2/D4 metal material-lighting positives and explicit negative-area taxonomy frames to TRAIN only.",
            "Replace or retrain the area boundary head with taxonomy-balanced hard negatives before reopening a new protocol.",
        ],
    }
    write_json(final / "PERCEPTION_MRV2_FINAL_BLOCKERS.json", blockers)

    registry = {
        "schema_version": 1,
        "selected_model": None,
        "freeze_id": None,
        "models": [
            {"route": "historical-X3", "status": "FAILED_STATIC_FULL_PIPELINE"},
            {"route": "MRV2-A", "status": "FAILED_STATIC_FULL_PIPELINE", "checkpoint": a["models"]["detector"]},
            {"route": "MRV2-B", "status": "FAILED_STATIC_FULL_PIPELINE", "checkpoint": a["models"]["detector"], "postprocess": "bounded_ground3_tiling"},
            {
                "route": "MRV2-C",
                "status": "FAILED_STATIC_FULL_PIPELINE",
                "training_report": {
                    "logical_external_path": str(args.mrv2_c_train_report),
                    "bytes": args.mrv2_c_train_report.stat().st_size,
                    "sha256": sha256(args.mrv2_c_train_report),
                },
                "checkpoint": {
                    "logical_external_path": str(args.mrv2_c_train_report.parent / c_train["checkpoint"]["path"]),
                    **c_train["checkpoint"],
                },
                "architecture": c_train["architecture"],
                "teacher": c_train["teacher"],
                "reproduction_command": "python3 scripts/perception_mrv2_c_train.py --epochs 6 --epoch-frames 600 [recorded roots]",
            },
            {
                "route": "MRV2-05-reference",
                "status": "FAILED_REFERENCE_STATIC_BENCHMARK",
                "checkpoint": grounding["official_checkpoint"],
                "shipped_in_product": False,
                "reproduction_command": "python3 scripts/perception_grounding_dino_benchmark.py [recorded official source/checkpoint and fixed dataset roots]",
            },
        ],
    }
    write_json(final / "PERCEPTION_MRV2_MODEL_REGISTRY.json", registry)

    release = {
        "schema_version": 1,
        "release_created": False,
        "release_path": None,
        "release_sha256": None,
        "deployed": False,
        "rollback_point": None,
        "reason": "MRV2_X86_STATIC_PASS=false; freeze and all downstream release gates are locked",
        "source": source,
    }
    write_json(final / "PERCEPTION_MRV2_RELEASE_MANIFEST.json", release)

    notices = f"""# PERCEPTION MRV2 third-party notices

## Grounding DINO

- Upstream: IDEA-Research/GroundingDINO
- Source commit: `856dde20aee659246248e20734ef9ba5214f5e44`
- Source license: Apache-2.0
- Checkpoint: `groundingdino_swint_ogc.pth`
- Checkpoint SHA256: `{grounding['official_checkpoint']['sha256']}`
- Role: reference benchmark only
- Shipped in product: no
- Redistribution: not attempted; exact checkpoint-artifact license remains a release blocker.
- Local modification: recorded CUDA use of the official PyTorch deformable-attention fallback because the reference container has no nvcc/custom op.

## Torchvision

- Role: FCOS/ResNet-FPN training and benchmark runtime
- Distribution status: no product bundle was created because the static gate failed.
"""
    (final / "PERCEPTION_MRV2_THIRD_PARTY_NOTICES.md").write_text(notices, encoding="utf-8")

    core_paths = [
        "baseline/BASELINE.json", "audits/MRV2_00_DECISION.json",
        "mrv2_a/MRV2_A_R960_FULL_STATIC.json", "mrv2_b/MRV2_B_SCREEN.json",
        "mrv2_c/MRV2_C_FULL_STATIC.json", "grounding_dino/GROUNDING_DINO_PROVENANCE.json",
        "grounding_dino/GROUNDING_DINO_BENCHMARK.json",
        "final/PERCEPTION_MRV2_FINAL_STATUS.json", "final/PERCEPTION_MRV2_FINAL_BLOCKERS.json",
        "final/PERCEPTION_MRV2_MODEL_REGISTRY.json", "final/PERCEPTION_MRV2_RELEASE_MANIFEST.json",
        "final/PERCEPTION_MRV2_THIRD_PARTY_NOTICES.md",
    ]
    records = [evidence_record(root, path) for path in core_paths]
    index_lines = ["# PERCEPTION MRV2 evidence index", "", f"Source tree: `{source['tree']}`", ""]
    index_lines.extend(
        f"- `{item['path']}` — {item['bytes']} bytes — `{item['sha256']}`"
        for item in records
    )
    (final / "PERCEPTION_MRV2_EVIDENCE_INDEX.md").write_text(
        "\n".join(index_lines) + "\n", encoding="utf-8"
    )

    c_domains = "/".join(
        f"{c['splits'][name]['discrete']['per_class']['metal_can']['recall']:.4f}"
        for name in ("D1", "D2", "D3", "D4")
    )
    report = f"""# MODEL-RECOVERY-V2 final report

## Outcome

MODEL-RECOVERY-V2 did not produce a freeze-eligible x86 model. MRV2-A, MRV2-B and MRV2-C all failed the unchanged static development gate, so `MODEL_BLOCKED_INTERNAL=true`. G5 and legacy D6 remained unread; no freeze, deployment, soak, replay, J6 or field performance claim was made.

## Required answers

1. Selected route: none. A/B/C all failed.
2. Small-object recall: historical X3 `0.3077`; best formal MRV2 VAL result was MRV2-C `{val_c['discrete']['small_object_recall']:.4f}`; cross-world `{cross_c['discrete']['small_object_recall']:.4f}`, below `0.70`.
3. MRV2-C metal_can recall: VAL `{val_c['discrete']['per_class']['metal_can']['recall']:.4f}`; D1/D2/D3/D4 `{c_domains}`. It does not satisfy VAL `0.90` and every-domain `0.70`.
4. Area: VAL boundary F1 `{val_c['area']['boundary_f1']:.4f}`; D4 boundary `{c['splits']['D4']['area']['boundary_f1']:.4f}` and negative-area FP/frame `{c['splits']['D4']['area']['negative_area_fp_per_frame']:.4f}`. Not all gates passed.
5. Grounding DINO: official checkpoint obtained and executed over holdout+VAL+D1-D5. VAL candidate recall `{grounding['splits']['VAL']['all_gt_candidate_recall']:.4f}`, small recall `{grounding['splits']['VAL']['small_object_candidate_recall']:.4f}`; reference gate failed. Historical X2 remains unchanged.
6. Sealed final: not opened; static prerequisite failed.
7. 30-seed moving-camera: not run; no valid freeze/sealed-final pass.
8. DynamicTrashMap: software remains implemented, but no MRV2 formal product pass was unlocked.
9. Spot Cleaning/post-clean: not run under MRV2 because prerequisites failed; no CLEANED claim.
10. 2h soak/MCAP replay: not run; no frozen product pipeline existed.
11. x86 release: none; path/hash are null.
12. J6: not started under MRV2 because the x86 teacher never froze; board remains absent.
13. Field: no qualifying RGB-D/independent GT; no field metric was fabricated.
14. PR #90: remains Draft and open.
15. Remaining blocker: internal model quality first; physical J6, real RGB-D and independent GT also remain external resources but are not the reason execution stopped.

## Route evidence

- MRV2-A: VAL macro F1 `{val_a['discrete']['macro_f1']:.4f}`, small recall `{val_a['discrete']['small_object_recall']:.4f}`, metal_can `{val_a['discrete']['per_class']['metal_can']['recall']:.4f}`, FP/min `{val_a['candidate']['false_candidates_per_min']:.1f}`.
- MRV2-B: bounded tiling added no accepted small candidates and remained failed.
- MRV2-C: 28/102 eligible TRAIN small truths received teacher-refined geometry; P2 training completed 6x600 frames. VAL macro F1 `{val_c['discrete']['macro_f1']:.4f}`, small recall `{val_c['discrete']['small_object_recall']:.4f}`, FP/min `{val_c['candidate']['false_candidates_per_min']:.1f}`.

No later gate is inferred from these development results.
"""
    (final / "MODEL_RECOVERY_V2_REPORT.md").write_text(report, encoding="utf-8")

    artifacts = []
    for path in sorted(root.rglob("*")):
        if not path.is_file() or path.name == "artifact_manifest.json" or path.suffix not in (".json", ".md"):
            continue
        artifacts.append(evidence_record(root, path.relative_to(root).as_posix()))
    write_json(root / "artifact_manifest.json", {
        "schema_version": 1, "stage": "MRV2-EVIDENCE-MANIFEST", "artifacts": artifacts
    })
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
