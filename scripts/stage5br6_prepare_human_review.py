from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LEARNING = ROOT / "starter_ws" / "src" / "sanitation_learning"
sys.path.insert(0, str(LEARNING))

from sanitation_learning.human_review_handoff import prepare_handoff  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description="Build isolated Stage5BR6-A reviewer handoff packages.")
    parser.add_argument("--positive-root", required=True)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--g2-dataset-root")
    source.add_argument("--prepared-negative-root")
    parser.add_argument("--output-root", required=True)
    parser.add_argument("--seed", type=int)
    args = parser.parse_args()
    result = prepare_handoff(
        args.positive_root,
        args.g2_dataset_root,
        args.output_root,
        args.seed,
        prepared_negative_root=args.prepared_negative_root,
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
