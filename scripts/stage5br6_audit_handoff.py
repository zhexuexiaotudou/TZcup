from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LEARNING = ROOT / "starter_ws" / "src" / "sanitation_learning"
sys.path.insert(0, str(LEARNING))

from sanitation_learning.human_review_handoff import audit_handoff  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description="Audit Stage5BR6 reviewer ZIP isolation and integrity.")
    parser.add_argument("--handoff-root", required=True)
    parser.add_argument("--output")
    args = parser.parse_args()
    report = audit_handoff(args.handoff_root)
    rendered = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        Path(args.output).write_text(rendered, encoding="utf-8")
    print(rendered, end="")


if __name__ == "__main__":
    main()
