#!/usr/bin/env python3
"""Build the formal OPRV3 G6 development-only corpus."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))

from sanitation_learning.g6_dataset import build_g6_dataset  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    qa = build_g6_dataset(args.output)
    print(args.output / "reports/G6_DATASET_QA.json")
    return 0 if qa["G6_DATASET_PASS"] else 4


if __name__ == "__main__":
    raise SystemExit(main())
