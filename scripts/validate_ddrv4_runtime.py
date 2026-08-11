#!/usr/bin/env python3
"""Fail-closed GPU runtime preflight for the combined DDRV4 x86 stack."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import platform


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--area-onnx", required=True, type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    import mmdet
    import onnxruntime as ort
    import torch

    if not torch.cuda.is_available():
        raise RuntimeError("DDRV4 runtime preflight requires CUDA PyTorch")
    available = ort.get_available_providers()
    if "CUDAExecutionProvider" not in available:
        raise RuntimeError(f"CUDAExecutionProvider is unavailable: {available}")
    session = ort.InferenceSession(
        args.area_onnx.as_posix(), providers=["CUDAExecutionProvider"]
    )
    active = session.get_providers()
    if not active or active[0] != "CUDAExecutionProvider":
        raise RuntimeError(f"Area ONNX silently fell back from CUDA: {active}")
    report = {
        "schema_version": 1,
        "stage": "DDRV4-06-RUNTIME-PREFLIGHT",
        "platform": platform.platform(),
        "gpu": torch.cuda.get_device_name(0),
        "torch": torch.__version__,
        "mmdetection": mmdet.__version__,
        "onnxruntime": ort.__version__,
        "available_providers": available,
        "active_area_providers": active,
        "area_onnx": args.area_onnx.as_posix(),
        "RUNTIME_PREFLIGHT_PASS": True,
    }
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
