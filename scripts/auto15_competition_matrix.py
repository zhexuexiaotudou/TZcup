#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import platform

from validate_product_acceptance_contract import (
    DEFAULT_CONTRACT,
    ProductAcceptanceContractError,
    load_contract,
    validate_auto15_execution_evidence,
    validate_static_contract,
)


ROOT = Path(__file__).resolve().parents[1]

SCENARIOS = (
    ("mapping", "建图与 20,000 m² 地图加载", ("AUTO-11",)),
    ("full_coverage", "全覆盖清扫", ("AUTO-02", "AUTO-12")),
    ("timed_trajectory", "定时轨迹", ("AUTO-11",)),
    (
        "discrete_pick",
        "离散垃圾识别与抓取",
        ("AUTO-08", "AUTO-09"),
    ),
    ("leaf_pile", "落叶堆识别与清扫", ("AUTO-08",)),
    ("puddle", "积水识别与清扫", ("AUTO-08",)),
    ("spot_cleaning", "学习感知定点清扫", ("AUTO-08",)),
    ("dynamic_avoidance", "动态避障", ("AUTO-02",)),
    ("narrow_corridor", "窄通道", ("AUTO-02",)),
    ("boundary_protection", "边界保护", ("AUTO-02",)),
    ("emergency_stop", "急停", ("AUTO-02", "AUTO-10")),
    ("app", "APP", ("AUTO-10",)),
    ("speech", "语音", ("AUTO-10",)),
    ("llm_dsl", "LLM 任务分解", ("AUTO-10",)),
    ("bin_full", "满箱与拒绝入箱", ("AUTO-09",)),
    ("recovery_replay", "恢复与回放", ("AUTO-02", "AUTO-11")),
    ("efficiency", "3500 m²/h 效率", ("AUTO-12",)),
    ("j6_runtime", "J6 runtime", ("AUTO-14",)),
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path: Path, payload: object) -> None:
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def build_matrix(
    state: dict,
    execution_evidence: dict | None = None,
    evidence_root: Path = ROOT,
    authoritative_source: Path | None = None,
) -> dict:
    product_contract = load_contract(DEFAULT_CONTRACT)
    contract = validate_static_contract(product_contract, authoritative_source)
    runtime_evidence = None
    if execution_evidence is not None:
        runtime_evidence = validate_auto15_execution_evidence(
            product_contract, execution_evidence, evidence_root
        )
    execution_ids = iter(contract["required_execution_ids"])
    rows = []
    blocking_dependencies = []
    for scenario_id, title, dependencies in SCENARIOS:
        scenario_evidence = (
            runtime_evidence["scenario_evidence_counts"].get(scenario_id, {})
            if runtime_evidence
            else {}
        )
        scenario_groups = {
            group
            for execution, group in (
                runtime_evidence["execution_to_mission_group"].items()
                if runtime_evidence
                else []
            )
            if execution.startswith(f"{scenario_id}:seed-")
        }
        dep_states = {
            stage: state["stages"][stage]["status"] for stage in dependencies
        }
        failed = [
            stage
            for stage, status in dep_states.items()
            if status != "PASS"
        ]
        for stage in failed:
            if stage not in blocking_dependencies:
                blocking_dependencies.append(stage)
        rows.append(
            {
                "scenario_id": scenario_id,
                "title": title,
                "required_stages": list(dependencies),
                "dependency_status": dep_states,
                "component_evidence_status": (
                    "AVAILABLE" if not failed else "BLOCKED"
                ),
                "integrated_execution_status": (
                    "EVIDENCE_RETAINED_NOT_PRODUCT_ACCEPTED"
                    if runtime_evidence
                    else "NOT_EXECUTED"
                ),
                "scenario_seed_count": scenario_evidence.get("executions", 0),
                "required_execution_ids": [next(execution_ids) for _ in range(contract["seed_count_per_scenario"])],
                "formal_mission_count": len(scenario_groups),
                "video_count": scenario_evidence.get("videos", 0),
                "mcap_count": scenario_evidence.get("mcaps", 0),
                "first_blocking_dependency": failed[0] if failed else None,
                "claim_boundary": (
                    "Existing stage evidence is indexed only; no AUTO-15 "
                    "integrated mission is inferred from component results."
                ),
            }
        )
    return {
        "schema_version": 2,
        "stage": "AUTO-15",
        "status": "BLOCKED",
        "simulation_competition_matrix_pass": False,
        "static_contract_pass": contract["static_contract_pass"],
        "runtime_execution_evidence_pass": bool(
            runtime_evidence and runtime_evidence["execution_evidence_pass"]
        ),
        "product_runtime_states": contract["product_runtime_states"],
        "first_blocking_layer": "dependency_AUTO-08_learned_spot_cleaning_blocked",
        "scenario_count": len(rows),
        "required_scenario_count": contract["scenario_count"],
        "required_seeds_per_scenario": contract["seed_count_per_scenario"],
        "required_unique_execution_count": contract["required_execution_count"],
        "required_integrated_missions": contract["minimum_mission_group_count"],
        "executed_integrated_missions": runtime_evidence["mission_group_count"] if runtime_evidence else 0,
        "formal_video_count": runtime_evidence["retained_execution_count"] if runtime_evidence else 0,
        "formal_mcap_count": runtime_evidence["retained_execution_count"] if runtime_evidence else 0,
        "blocking_dependencies": blocking_dependencies,
        "scenarios": rows,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--state",
        default=str(ROOT / "config" / "autonomy" / "AUTONOMOUS_STATE.json"),
    )
    parser.add_argument("--output", required=True)
    parser.add_argument("--implementation-commit", required=True)
    parser.add_argument("--execution-evidence", type=Path)
    parser.add_argument("--evidence-root", type=Path, default=ROOT)
    parser.add_argument("--authoritative-source", type=Path)
    args = parser.parse_args()

    state_path = Path(args.state).resolve()
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    state = json.loads(state_path.read_text(encoding="utf-8"))
    try:
        execution_evidence = (
            json.loads(args.execution_evidence.read_text(encoding="utf-8"))
            if args.execution_evidence
            else None
        )
        matrix = build_matrix(
            state, execution_evidence, args.evidence_root, args.authoritative_source
        )
    except (OSError, json.JSONDecodeError, ProductAcceptanceContractError) as exc:
        parser.error(f"AUTO-15 execution evidence failed closed: {exc}")
    write_json(output / "competition_matrix.json", matrix)

    metrics = {
        "required_scenario_count": matrix["required_scenario_count"],
        "indexed_scenario_count": matrix["scenario_count"],
        "component_evidence_available_count": sum(
            row["component_evidence_status"] == "AVAILABLE"
            for row in matrix["scenarios"]
        ),
        "component_evidence_blocked_count": sum(
            row["component_evidence_status"] == "BLOCKED"
            for row in matrix["scenarios"]
        ),
        "required_seeds_per_scenario": matrix["required_seeds_per_scenario"],
        "required_unique_execution_count": matrix["required_unique_execution_count"],
        "retained_unique_execution_count": matrix["formal_video_count"],
        "executed_seeds_per_scenario": min(
            (row["scenario_seed_count"] for row in matrix["scenarios"]), default=0
        ),
        "required_integrated_missions": 30,
        "executed_integrated_missions": matrix["executed_integrated_missions"],
        "formal_video_count": matrix["formal_video_count"],
        "formal_mcap_count": matrix["formal_mcap_count"],
        "static_contract_pass": matrix["static_contract_pass"],
        "runtime_execution_evidence_pass": matrix["runtime_execution_evidence_pass"],
        "product_runtime_states": matrix["product_runtime_states"],
        "simulation_competition_matrix_pass": False,
    }
    write_json(output / "metrics_summary.json", metrics)
    blockers = [
        {
            "blocker_id": "AUTO15-B1",
            "type": "dependency",
            "stage": "AUTO-08",
            "status": state["stages"]["AUTO-08"]["status"],
            "cause": state["stages"]["AUTO-08"]["first_blocking_layer"],
            "impact": (
                "learned discrete/area perception and spot-cleaning "
                "scenarios cannot enter integrated formal missions"
            ),
        },
        {
            "blocker_id": "AUTO15-B2",
            "type": "dependency",
            "stage": "AUTO-14",
            "status": state["stages"]["AUTO-14"]["status"],
            "cause": state["stages"]["AUTO-14"]["first_blocking_layer"],
            "impact": "J6 runtime scenario cannot execute",
        },
    ]
    write_json(output / "blocker_register.json", blockers)
    unexecuted = (
        [
            "180 unique scenario/seed execution receipts (18 scenarios x 10 seeds)",
            "30 distinct integrated formal mission groups",
            "retained hash-verified video for every constituent execution",
            "retained hash-verified MCAP for every constituent execution",
            "aggregate competition metrics",
        ]
        if not matrix["runtime_execution_evidence_pass"]
        else ["product-gate metrics and product runtime acceptance"]
    )
    write_json(
        output / "stage_status.json",
        {
            "schema_version": 1,
            "program": "TZcup autonomous final",
            "stage_id": "AUTO-15",
            "implementation_commit": args.implementation_commit,
            "status": "BLOCKED",
            "first_blocking_layer": matrix["first_blocking_layer"],
            "attempt_count": 1,
            "machine_gate_pass": False,
            "human_review_required": False,
            "human_approval_required": False,
            "competition_evidence": False,
            "static_contract_pass": matrix["static_contract_pass"],
            "runtime_execution_evidence_pass": matrix["runtime_execution_evidence_pass"],
            "product_runtime_states": matrix["product_runtime_states"],
            "dependencies": {
                stage: state["stages"][stage]["status"]
                for stage in state["stages"]["AUTO-15"]["dependencies"]
            },
            "metrics": metrics,
            "unexecuted_items": unexecuted,
            "claim_boundary": (
                "Runtime receipts are evidence accounting only and do not "
                "promote product runtime states."
                if matrix["runtime_execution_evidence_pass"]
                else "Static contract/cardinality validation passed, but no AUTO-15 runtime execution receipts, videos, or MCAPs were supplied; product runtime states remain false."
            ),
        },
    )
    write_json(
        output / "attempt_ledger.json",
        {
            "schema_version": 1,
            "stage": "AUTO-15",
            "attempts": [
                {
                    "attempt_id": "AUTO-15-DEPENDENCY-PREFLIGHT-V1",
                    "hypothesis": (
                        "all mandatory dependencies may be ready for the "
                        "integrated formal competition matrix"
                    ),
                    "input_commit": args.implementation_commit,
                    "result": "BLOCKED",
                    "first_failure": matrix["first_blocking_layer"],
                    "decision": (
                        "do_not_launch_integrated_missions; preserve the "
                        "complete requirement and blocker matrix"
                    ),
                }
            ],
        },
    )
    write_json(
        output / "environment.json",
        {
            "platform": platform.platform(),
            "python": platform.python_version(),
            "state_file": state_path.name,
        },
    )
    (output / "commands.txt").write_text(
        "py -3 scripts/auto15_competition_matrix.py "
        "--output artifacts/autonomous_auto15_20260730_evidence "
        "--implementation-commit <sha>\n"
        "py -3 scripts/ci_fast.py\n",
        encoding="utf-8",
    )
    (output / "README.md").write_text(
        "# AUTO-15 evidence\n\n"
        "Complete 18-scenario requirement/dependency matrix. AUTO-15 formal "
        "integrated missions were not executed and are not claimed.\n",
        encoding="utf-8",
    )

    state["stages"]["AUTO-15"].update(
        {
            "status": "BLOCKED",
            "machine_gate_pass": False,
            "blocked": True,
            "blocked_external": False,
            "first_blocking_layer": matrix["first_blocking_layer"],
            "attempt_count": 1,
            "selected_attempt": "AUTO-15-DEPENDENCY-PREFLIGHT-V1",
            "implementation_commit": args.implementation_commit,
            "evidence_dir": output.relative_to(ROOT).as_posix(),
            "metrics": metrics,
            "unexecuted_items": unexecuted,
        }
    )
    state["final_states"]["SIMULATION_COMPETITION_MATRIX_PASS"] = False
    state["final_states"]["FINAL_COMPETITION_EVIDENCE_COMPLETE"] = False
    state["run"]["branch"] = "agent/autonomous-auto15"
    state["run"]["current_stage"] = "AUTO-16"
    state["run"]["last_commit"] = args.implementation_commit
    state_path.write_text(json.dumps(state, indent=2) + "\n", encoding="utf-8")

    files = []
    for path in sorted(output.rglob("*")):
        if path.is_file() and path.name != "artifact_manifest.json":
            files.append(
                {
                    "path": path.relative_to(output).as_posix(),
                    "bytes": path.stat().st_size,
                    "sha256": sha256(path),
                }
            )
    write_json(
        output / "artifact_manifest.json",
        {
            "schema_version": 1,
            "stage": "AUTO-15",
            "implementation_commit": args.implementation_commit,
            "file_count": len(files),
            "files": files,
        },
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
