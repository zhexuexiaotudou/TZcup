#!/usr/bin/env python3
"""Export the frozen-size MRV2-A detector to a product ONNX artifact."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import torch


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws" / "src" / "sanitation_learning"))
sys.path.insert(0, str(ROOT / "scripts"))

from perception_oprv3_moving_benchmark import load_detector  # noqa: E402


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class ProductFCOSExport(torch.nn.Module):
    def __init__(self, detector: torch.nn.Module):
        super().__init__()
        self.detector = detector

    def forward(self, images: torch.Tensor):
        result = self.detector([images[0]])[0]
        return result["boxes"], result["scores"], result["labels"]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    if device.type != "cuda":
        raise RuntimeError("formal MRV2-A ONNX export requires CUDA")
    detector, metadata = load_detector(args.checkpoint, device)
    width, height = (int(value) for value in metadata["input_size"])
    wrapper = ProductFCOSExport(detector).eval()
    dummy = torch.zeros((1, 3, height, width), dtype=torch.float32, device=device)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with torch.no_grad():
        torch.onnx.export(
            wrapper,
            dummy,
            args.output,
            input_names=["images"],
            output_names=["boxes", "scores", "labels"],
            opset_version=17,
            do_constant_folding=True,
            dynamic_axes={
                "boxes": {0: "detections"},
                "scores": {0: "detections"},
                "labels": {0: "detections"},
            },
        )
    import onnx

    model = onnx.load(args.output.as_posix())
    onnx.checker.check_model(model)
    custom_domains = sorted(
        {node.domain for node in model.graph.node if node.domain}
    )
    report = {
        "schema_version": 1,
        "protocol": "OPRV3-MRV2-A-ONNX-EXPORT",
        "checkpoint": {
            "path": args.checkpoint.as_posix(),
            "sha256": sha256(args.checkpoint),
        },
        "onnx": {
            "path": args.output.as_posix(),
            "sha256": sha256(args.output),
            "opset": 17,
            "input_shape": [1, 3, height, width],
            "custom_domains": custom_domains,
            "checker_pass": True,
        },
        "metadata": metadata,
        "G5_SEALED_FINAL_read": False,
        "legacy_G4_D6_read": False,
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
