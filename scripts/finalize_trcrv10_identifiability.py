#!/usr/bin/env python3
"""Finalize TRCRV10 identifiability curves and the conservative size gate."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from train_trcrv10_identifiability import CLASSES, metrics


BUCKETS = ("lt12", "12_18", "18_32", "32_48", "48_64", "64_96", "ge96")
LOWER_BOUNDS = {"lt12": 0, "12_18": 12, "18_32": 18, "32_48": 32,
                "48_64": 48, "64_96": 64, "ge96": 96}


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def passes(result: dict, macro: float, per_class: float) -> bool:
    return (
        result["macro_f1"] >= macro
        and all(row["precision"] >= per_class and row["recall"] >= per_class
                for row in result["per_class"].values())
    )


def combine_confusions(rows: list[dict]) -> dict:
    confusion = np.sum([np.asarray(row["confusion"], dtype=np.int64) for row in rows], axis=0)
    truth, predicted = [], []
    for expected in range(len(CLASSES)):
        for actual in range(len(CLASSES)):
            count = int(confusion[expected, actual])
            truth.extend([expected] * count); predicted.extend([actual] * count)
    return metrics(truth, predicted)


def reliable_bucket(results: list[dict], view: str) -> str | None:
    runs = [row for row in results if row["view"] == view]
    if len(runs) != 2:
        raise ValueError(f"view {view} does not have exactly two model runs")
    for index, candidate in enumerate(BUCKETS):
        larger = BUCKETS[index:]
        observed = [bucket for bucket in larger if any(bucket in run["by_size"] for run in runs)]
        if candidate not in observed:
            continue
        if all(all(bucket in run["by_size"] and passes(run["by_size"][bucket], .97, .95)
                   for bucket in observed) for run in runs):
            return candidate
    return None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--raw", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    raw = read(args.raw)
    results = raw["results"]
    if set(raw["models"]) != {"convnext_tiny", "resnet18"}:
        raise ValueError("protocol requires exactly ConvNeXt-Tiny and ResNet18")

    curves = []
    confusions = []
    domains = []
    for run in results:
        curves.append({"model": run["model"], "view": run["view"], "by_size": run["by_size"]})
        confusions.append({"model": run["model"], "view": run["view"],
                           "by_size": {key: value["confusion"] for key, value in run["by_size"].items()}})
        domains.append({"model": run["model"], "view": run["view"], "by_domain": run["by_domain"]})

    thresholds = {view: reliable_bucket(results, view) for view in ("tight", "context")}
    eligible = [(LOWER_BOUNDS[bucket], view, bucket) for view, bucket in thresholds.items() if bucket]
    if eligible:
        selected_px, selected_view, selected_bucket = min(eligible)
    else:
        selected_px = selected_view = selected_bucket = None

    large_gates = {}
    for threshold, buckets in ((64, ("64_96", "ge96")), (96, ("ge96",))):
        per_run = []
        for run in results:
            available = [run["by_size"][bucket] for bucket in buckets if bucket in run["by_size"]]
            combined = combine_confusions(available) if available else None
            per_run.append({"model": run["model"], "view": run["view"], "metrics": combined,
                            "pass": combined is not None and passes(combined, .95, .90)})
        by_view = {
            view: all(row["pass"] for row in per_run if row["view"] == view)
            for view in ("tight", "context")
        }
        large_gates[str(threshold)] = {"runs": per_run, "by_view_both_models_pass": by_view,
                                       "any_view_both_models_pass": any(by_view.values())}

    asset_fail = not (large_gates["64"]["any_view_both_models_pass"] or large_gates["96"]["any_view_both_models_pass"])
    summary = {
        "schema_version": 1,
        "protocol": "TRCRV10-01",
        "model_search_count": 2,
        "model_search_exhausted": True,
        "reliable_bucket_by_view_conservative_across_models": thresholds,
        "selected_view": selected_view,
        "selected_bucket": selected_bucket,
        "MIN_RELIABLE_CLASSIFICATION_SHORT_SIDE_PX": selected_px,
        "large_target_asset_rule": large_gates,
        "VISUAL_IDENTIFIABILITY_FAIL": asset_fail,
        "VISUAL_IDENTIFIABILITY_PASS": not asset_fail and selected_px is not None,
        "production_runtime_uses_GT_crop": False,
    }
    write(args.output / "IDENTIFIABILITY_BY_SIZE.json", {"schema_version": 1, "curves": curves})
    write(args.output / "IDENTIFIABILITY_CONFUSION_BY_SIZE.json", {"schema_version": 1, "runs": confusions})
    write(args.output / "IDENTIFIABILITY_BY_DOMAIN.json", {"schema_version": 1, "runs": domains})
    write(args.output / "IDENTIFIABILITY_SUMMARY.json", summary)
    print(json.dumps(summary, indent=2))
    return 0 if summary["VISUAL_IDENTIFIABILITY_PASS"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
