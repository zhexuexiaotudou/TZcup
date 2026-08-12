#!/usr/bin/env python3
"""Build the ODCV5-02 G7 moving-camera development pack."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))

from sanitation_learning.g7_moving_dataset import build_g7_moving_dataset  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    result = build_g7_moving_dataset(args.output)
    print(args.output / "reports/G7_MOVING_QA.json")
    return 0 if result["G7_MOVING_PASS"] else 4


if __name__ == "__main__":
    raise SystemExit(main())
