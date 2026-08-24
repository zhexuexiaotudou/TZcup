"""One-command scenario generation and URDF-independent research closure."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

from sanitation_campus_scenario.generator import GenerationError, generate_episode, load_config
from sanitation_campus_scenario.io import write_episode

from .harness import run_bundle


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--profile", choices=("research", "formal"), default="research")
    parser.add_argument("--split", choices=("train", "val", "hidden"), default="train")
    parser.add_argument("--map-index", type=int, default=0)
    parser.add_argument("--mission-index", type=int, default=0)
    parser.add_argument("--output", required=True, type=Path)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    output = args.output.resolve()
    if output.exists():
        print(f"output already exists: {output}", file=sys.stderr)
        return 2
    try:
        scenario_dir = output / "scenario"
        files = generate_episode(
            load_config(args.config),
            args.profile,
            args.split,
            args.map_index,
            args.mission_index,
            include_proxy=True,
        )
        write_episode(scenario_dir, files)
        report = run_bundle(scenario_dir)
        output.mkdir(parents=True, exist_ok=True)
        (output / "report.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    except (GenerationError, OSError, ValueError, RuntimeError) as exc:
        print(f"demo failed: {exc}", file=sys.stderr)
        return 2
    print(output / "report.json")
    return 0 if report["metrics"]["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
