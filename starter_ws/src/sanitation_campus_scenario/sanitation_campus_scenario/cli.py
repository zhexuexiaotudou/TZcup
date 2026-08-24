"""Command-line interface for deterministic scenario generation."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

from .generator import GenerationError, generate_episode, load_config, split_index
from .io import write_episode, write_json_file


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="sanitation-campus-scenario")
    subparsers = parser.add_subparsers(dest="command", required=True)
    generate = subparsers.add_parser("generate", help="generate one deterministic episode")
    generate.add_argument("--config", required=True, type=Path)
    generate.add_argument("--profile", required=True, choices=("research", "formal"))
    generate.add_argument("--split", required=True, choices=("train", "val", "hidden"))
    generate.add_argument("--map-index", required=True, type=int)
    generate.add_argument("--mission-index", required=True, type=int)
    generate.add_argument("--output", required=True, type=Path)
    generate.add_argument("--include-proxy", action="store_true", help="include a static 0.60 x 0.40 m planning proxy explicitly marked as not URDF")
    index = subparsers.add_parser(
        "split-index", help="write the frozen map/mission split index"
    )
    index.add_argument("--config", required=True, type=Path)
    index.add_argument("--output", required=True, type=Path)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        config = load_config(args.config)
        if args.command == "generate":
            files = generate_episode(
                config,
                args.profile,
                args.split,
                args.map_index,
                args.mission_index,
                include_proxy=args.include_proxy,
            )
            output = write_episode(args.output, files)
        else:
            output = write_json_file(args.output, split_index(config))
        print(output)
        return 0
    except GenerationError as exc:
        print(f"generation failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
