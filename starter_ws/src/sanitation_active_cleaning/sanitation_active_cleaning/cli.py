"""Command-line paired evaluation for the URDF-independent environment."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Sequence

from .evaluation import evaluate_paired
from .models import TaskConfig


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--seeds", default="101,102,103")
    parser.add_argument("--output", type=Path)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    seeds = [int(item.strip()) for item in args.seeds.split(",") if item.strip()]
    report = evaluate_paired(TaskConfig.from_json(args.config), seeds=seeds)
    encoded = json.dumps(report, indent=2, sort_keys=True, ensure_ascii=False)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded + "\n", encoding="utf-8")
    else:
        print(encoded)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
