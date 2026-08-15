#!/usr/bin/env python3
"""AUTO-05R-1 G4 dataset finalization CLI.

Wraps `sanitation_learning.g4_qa.finalize_g4_dataset`.  Non-strict mode fails
on data-quality violations but reports incomplete formal scale as
expected/actual; strict mode requires every frozen gate including the full
12 world / 300 scene / 3000 frame scale.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws" / "src" / "sanitation_learning"))

from sanitation_learning.g4_qa import finalize_g4_dataset  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument(
        "--contract",
        default=str(
            ROOT
            / "starter_ws"
            / "src"
            / "sanitation_learning"
            / "config"
            / "auto05r_g4_contract.yaml"
        ),
    )
    parser.add_argument("--strict", action="store_true")
    args = parser.parse_args()
    report = finalize_g4_dataset(
        args.data_root,
        args.output_dir,
        contract_path=args.contract,
        strict=args.strict,
    )
    print(
        json.dumps(
            {
                "G4_dataset_gate_pass": report["G4_dataset_gate_pass"],
                "quality_gates_pass": report["quality_gates_pass"],
                "formal_scale": report["formal_scale"],
                "scene_count": report["scene_count"],
                "frame_count": report["frame_count"],
                "failed_gates": [
                    name for name, passed in report["gates"].items() if not passed
                ],
                "errors": report["errors"][:10],
            },
            indent=2,
        )
    )
    if args.strict:
        return 0 if report["G4_dataset_gate_pass"] else 2
    return 0 if report["quality_gates_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
