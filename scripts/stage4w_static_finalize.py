#!/usr/bin/env python3
"""Finalize one Stage4W/AUTO-02 static Coverage trial."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def build_summary(root: Path, seed: int, coverage_code: int) -> dict:
    coverage = json.loads(
        (root / "coverage_report.json").read_text(encoding="utf-8")
    )
    replay = json.loads(
        (root / "auto02_replay_audit.json").read_text(encoding="utf-8")
    )
    return {
        "schema_version": 1,
        "stage": "Stage4W",
        "seed": seed,
        "coverage_exit_code": coverage_code,
        "coverage": coverage,
        "rosbag_replay": replay["replay_gate_pass"],
        "static_gate_pass": bool(
            coverage_code == 0
            and coverage.get("success")
            and coverage.get("full_execution_success")
            and coverage.get("empirical_metrics", {}).get(
                "coverage_rate", 0
            )
            >= 0.90
            and coverage.get("collision_count") == 0
            and coverage.get("keepout_violation_sample_count") == 0
            and coverage.get("brush_state_violation_sample_count") == 0
            and coverage.get("brush_disabled_on_exit")
            and coverage.get("swath_exclusion_intersection_count") == 0
            and coverage.get(
                "localization_regression_during_coverage", {}
            ).get("pass_rmse_at_most_0_05m")
            and replay["replay_gate_pass"]
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--coverage-code", type=int, required=True)
    args = parser.parse_args()
    summary = build_summary(args.root, args.seed, args.coverage_code)
    (args.root / "stage4w_static_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return 0 if summary["static_gate_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
