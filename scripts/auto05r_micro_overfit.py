#!/usr/bin/env python3
"""AUTO-05R-2/3 micro-overfit CLI (scaffold; no real training).

Usage::

    py -3 scripts/auto05r_micro_overfit.py \
        --model-type discovery|classifier|leaf|puddle \
        --data-root <g4_dataset_root> --output-dir <out> [--config <yaml>]

The CLI validates the frozen training protocol, builds the requested model
card, and writes a report that always states ``micro_overfit_pass=false`` and
``executed=false`` until a real training + gate run has happened.  It
deliberately does not run G4 training or export a formal ONNX model.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws" / "src" / "sanitation_learning"))

from sanitation_learning.g4_training import load_training_protocol  # noqa: E402


DEFAULT_CONFIG = (
    ROOT
    / "starter_ws"
    / "src"
    / "sanitation_learning"
    / "config"
    / "auto05r_training_protocol.yaml"
)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="AUTO-05R-2/3 micro-overfit scaffold CLI"
    )
    parser.add_argument(
        "--model-type",
        required=True,
        choices=("discovery", "classifier", "leaf", "puddle"),
    )
    parser.add_argument("--data-root", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--config", default=DEFAULT_CONFIG, type=Path)
    args = parser.parse_args()

    try:
        protocol = load_training_protocol(args.config)
    except ValueError as exc:
        print(f"error: invalid training protocol: {exc}", file=sys.stderr)
        return 2
    if not args.data_root.is_dir():
        print(
            f"error: --data-root must be an existing directory: {args.data_root}",
            file=sys.stderr,
        )
        return 2
    args.output_dir.mkdir(parents=True, exist_ok=True)

    torch_available = True
    try:
        from sanitation_learning.g4_models import build_g4_models, model_summary

        models = build_g4_models()
        model_card = model_summary(models)[args.model_type]
        reason = "scaffold_only_no_training_executed"
    except RuntimeError as exc:
        model_card = {
            "model_id": None,
            "state": "not_trained",
            "error": str(exc),
        }
        torch_available = False
        reason = "torch_unavailable"

    selection = protocol["model_selection"]
    report = {
        "schema_version": 1,
        "stage": "AUTO-05R",
        "task": "AUTO-05R-2-3",
        "model_type": args.model_type,
        "executed": False,
        "micro_overfit_pass": False,
        "status": "not_trained",
        "reason": reason,
        "torch_available": torch_available,
        "model_card": model_card,
        "data_root": str(args.data_root),
        "output_dir": str(args.output_dir),
        "protocol": {
            "model_seed": protocol["models"][args.model_type]["seed"],
            "sample_counts": protocol["micro_overfit"]["sample_counts"],
            "gates": protocol["micro_overfit"]["gates"],
            "batch_proportions": protocol["batch_proportions"],
            "optimizer": protocol["optimizer"],
            "scheduler": protocol["scheduler"],
            "ema_decay": protocol["ema_decay"],
            "early_stopping_patience": protocol["early_stopping_patience"],
            "test_split_readable_during_training": selection[
                "test_split_readable_during_training"
            ],
            "hard_negative_mining_from_test": selection[
                "hard_negative_mining_from_test"
            ],
        },
    }
    report_path = args.output_dir / "micro_overfit_report.json"
    report_path.write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    print(
        json.dumps(
            {
                "status": report["status"],
                "micro_overfit_pass": False,
                "executed": False,
                "model_type": args.model_type,
                "report": str(report_path),
            },
            indent=2,
        )
    )
    return 0 if torch_available else 2


if __name__ == "__main__":
    raise SystemExit(main())
