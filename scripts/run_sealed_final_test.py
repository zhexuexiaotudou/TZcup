#!/usr/bin/env python3
"""One-shot G5 sealed-final test CLI (fail-closed scaffolding).

The sealed final set can only be opened when a validated ``MODEL_FREEZE.json``
exists and the G5 manifest satisfies the frozen contract (>= 4 unseen worlds,
>= 100 scenes, >= 1000 frames, unseen target/hard-negative assets).  The first
access and evaluation are recorded atomically; reruns and partial probing are
refused.

This task intentionally does NOT create or expose a real G5 dataset.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws" / "src" / "sanitation_learning"))

from sanitation_learning.g4_gates import (  # noqa: E402
    evaluate_policy,
    load_policy,
)
from sanitation_learning.g4_sealed_final import (  # noqa: E402
    SealedFinalGate,
    SealedFinalReuseError,
)


P5_POLICY = (
    ROOT
    / "starter_ws"
    / "src"
    / "sanitation_learning"
    / "config"
    / "perception_p5_final_policy.yaml"
)


def _load_json(path: Path) -> dict:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return payload


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--freeze", required=True, type=Path)
    parser.add_argument("--sealed-manifest", required=True, type=Path)
    parser.add_argument("--development-registry", required=True, type=Path)
    parser.add_argument("--evidence-dir", required=True, type=Path)
    parser.add_argument("--dataset-root", required=True, type=Path)
    parser.add_argument("--evaluator", required=True, type=Path)
    args = parser.parse_args()

    from sanitation_learning.g4_manifest import load_freeze

    try:
        freeze = load_freeze(args.freeze)
        sealed_manifest = _load_json(args.sealed_manifest)
        development_registry = _load_json(args.development_registry)
        if not args.dataset_root.is_dir():
            raise FileNotFoundError(
                f"sealed dataset root not found: {args.dataset_root}"
            )
        if not args.evaluator.is_file():
            raise FileNotFoundError(
                f"sealed evaluator not found: {args.evaluator}"
            )
        from sanitation_learning.g4_manifest import file_sha256

        evaluator_sha256 = file_sha256(args.evaluator)
        if evaluator_sha256 != freeze["final_evaluator_sha256"]:
            raise ValueError(
                "sealed evaluator SHA-256 does not match MODEL_FREEZE.json"
            )
        gate = SealedFinalGate(args.evidence_dir)
        access = gate.open_once(
            freeze_path=args.freeze,
            sealed_manifest=sealed_manifest,
            development_world_ids=development_registry.get(
                "world_ids", []
            ),
            development_target_assets=development_registry.get(
                "target_assets", []
            ),
            development_hard_negative_assets=development_registry.get(
                "hard_negative_assets", []
            ),
        )
        print(json.dumps({"event": "sealed_final_first_access", **access}, indent=2))
        # Only after the atomic access record exists may evaluator code open
        # the sealed dataset. There is no partial/open-only mode and no input
        # path for precomputed or user-supplied metrics.
        spec = importlib.util.spec_from_file_location(
            "tzcup_frozen_g5_evaluator", args.evaluator
        )
        if spec is None or spec.loader is None:
            raise RuntimeError("unable to load frozen G5 evaluator")
        evaluator_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(evaluator_module)
        evaluate_fn = getattr(
            evaluator_module, "evaluate_sealed_final", None
        )
        if not callable(evaluate_fn):
            raise RuntimeError(
                "frozen evaluator must define evaluate_sealed_final"
            )
        metrics = evaluate_fn(
            dataset_root=args.dataset_root,
            freeze=freeze,
            sealed_manifest=sealed_manifest,
        )
        if not isinstance(metrics, dict):
            raise ValueError("sealed evaluator must return a metrics mapping")
        policy = load_policy(P5_POLICY)
        policy_result = evaluate_policy(policy, metrics)
        result = gate.evaluate_once(
            metrics=metrics,
            freeze_id=freeze["freeze_id"],
        )
        summary = {
            "event": "sealed_final_evaluation",
            "P5_FINAL_PASS": policy_result["pass"],
            "not_evaluated_gates": policy_result["not_evaluated"],
            "gates": policy_result["gates"],
            "result_record": str(gate.result_path),
        }
        print(json.dumps(summary, indent=2))
        return 0 if policy_result["pass"] else 2
    except (
        SealedFinalReuseError,
        ValueError,
        FileNotFoundError,
        RuntimeError,
    ) as exc:
        print(
            json.dumps(
                {
                    "sealed_final_blocked": True,
                    "reason": str(exc),
                    "P5_FINAL_PASS": False,
                    "access_may_be_consumed": (
                        args.evidence_dir / "sealed_final_access.json"
                    ).is_file(),
                },
                indent=2,
            ),
            file=sys.stderr,
        )
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
