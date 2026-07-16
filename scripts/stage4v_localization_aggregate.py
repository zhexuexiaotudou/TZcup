#!/usr/bin/env python3
import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'starter_ws' / 'src' / 'sanitation_tasks'))

from sanitation_tasks.stage4v_aggregate import aggregate  # noqa: E402


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('root', type=Path)
    parser.add_argument('--lane', required=True)
    parser.add_argument('--required-seeds', type=int, default=10)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    report = aggregate(args.root, args.lane, args.required_seeds)
    output = args.output or args.root / 'stage4v_localization_report.json'
    output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8'
    )
    print(json.dumps(report, ensure_ascii=False))
    return 0 if report['formal_gate_pass'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
