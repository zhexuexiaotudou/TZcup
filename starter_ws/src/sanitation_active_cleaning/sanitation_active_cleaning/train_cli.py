"""Train and freeze the belief-only tabular Q-learning planner."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Sequence

from .models import TaskConfig
from .rl import train_q_policy


def _seeds(value: str) -> list[int]:
    if ":" in value:
        start, stop = (int(item) for item in value.split(":", 1))
        return list(range(start, stop))
    return [int(item.strip()) for item in value.split(",") if item.strip()]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--train-seeds", default="100:140")
    parser.add_argument("--validation-seeds", default="200:210")
    parser.add_argument("--test-seeds", default="300:310")
    parser.add_argument("--policy-seed", type=int, default=7)
    parser.add_argument("--checkpoint", required=True, type=Path)
    parser.add_argument("--report", required=True, type=Path)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    policy, report = train_q_policy(
        TaskConfig.from_json(args.config),
        train_seeds=_seeds(args.train_seeds),
        validation_seeds=_seeds(args.validation_seeds),
        test_seeds=_seeds(args.test_seeds),
        policy_seed=args.policy_seed,
    )
    policy.save(args.checkpoint)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
