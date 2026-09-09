#!/usr/bin/env python3
"""Compare ONNX/quantized/HBM output tensors without semantic substitution."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def compare_outputs(
    reference: dict[str, np.ndarray],
    candidate: dict[str, np.ndarray],
    *,
    minimum_cosine: float = 0.99,
) -> dict:
    if set(reference) != set(candidate):
        raise ValueError("parity output node names differ")
    nodes = {}
    for name in sorted(reference):
        first = np.asarray(reference[name], dtype=np.float64)
        second = np.asarray(candidate[name], dtype=np.float64)
        if first.shape != second.shape:
            raise ValueError(f"parity output shape differs for {name}")
        flat_first, flat_second = first.ravel(), second.ravel()
        denominator = float(np.linalg.norm(flat_first) * np.linalg.norm(flat_second))
        if denominator == 0.0:
            cosine = 1.0 if np.array_equal(flat_first, flat_second) else 0.0
        else:
            cosine = float(np.dot(flat_first, flat_second) / denominator)
        absolute = np.abs(first - second)
        nodes[name] = {
            "shape": list(first.shape),
            "cosine_similarity": cosine,
            "max_abs_error": float(absolute.max(initial=0.0)),
            "mean_abs_error": float(absolute.mean()) if absolute.size else 0.0,
            "pass": cosine >= minimum_cosine,
        }
    return {
        "schema_version": 1,
        "minimum_cosine": minimum_cosine,
        "nodes": nodes,
        "all_nodes_pass": bool(nodes) and all(item["pass"] for item in nodes.values()),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--reference-runtime", required=True)
    parser.add_argument("--candidate-runtime", required=True)
    parser.add_argument("--minimum-cosine", type=float, default=0.99)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    with np.load(args.reference, allow_pickle=False) as archive:
        reference = {name: archive[name] for name in archive.files}
    with np.load(args.candidate, allow_pickle=False) as archive:
        candidate = {name: archive[name] for name in archive.files}
    report = compare_outputs(reference, candidate, minimum_cosine=args.minimum_cosine)
    report["reference_runtime"] = args.reference_runtime
    report["candidate_runtime"] = args.candidate_runtime
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["all_nodes_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
