from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import yaml


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "starter_ws" / "src" / "sanitation_learning"
sys.path.insert(0, str(PACKAGE))
from sanitation_learning.camera_mechanics import evaluate_all  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=str(PACKAGE / "config" / "stage5br5_active_observation.yaml"))
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    document = yaml.safe_load(Path(args.config).read_text(encoding="utf-8"))
    report = evaluate_all(document)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, ensure_ascii=False))
    raise SystemExit(0 if report["mechanical_grid_has_viable_candidate"] else 2)


if __name__ == "__main__":
    main()
