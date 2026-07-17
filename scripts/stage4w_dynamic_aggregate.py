#!/usr/bin/env python3
"""Assemble the Stage4W dynamic-obstacle, filter, safety, and replay gate."""

import argparse
import json
from pathlib import Path

from sanitation_tasks.stage4w_gate import assemble


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--coverage-code", type=int, default=0)
    parser.add_argument("--dynamic-code", type=int, default=0)
    parser.add_argument("--filter-code", type=int, default=0)
    parser.add_argument("--safety-code", type=int, default=0)
    args = parser.parse_args()
    exit_codes = {
        "coverage": args.coverage_code,
        "dynamic": args.dynamic_code,
        "filters": args.filter_code,
        "safety": args.safety_code,
    }
    report = assemble(args.output_dir, exit_codes)
    (args.output_dir / "stage4w_dynamic_report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    raise SystemExit(0 if report["success"] else 2)


if __name__ == "__main__":
    main()
