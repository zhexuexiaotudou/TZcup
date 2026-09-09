#!/usr/bin/env python3
"""Run native ADE20K SegFormer on the fixed G2 negative-only frames.

The worker preserves all 150 ADE20K classes.  It intentionally defines no
mapping from generic water/vegetation labels to road puddle or leaf pile.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import tempfile
import time
from collections import Counter
from pathlib import Path

import numpy as np

import c1_holdout_native_worker as isolation_helpers
import screen_emf_ewasr_negative as g2_manifest


MODEL_ID = "area_nvidia_segformer_b0_ade20k"
RUNTIME_IMAGE_DIGEST = (
    "sha256:bf61e2b6bca3b1fc6a66100986b59b2eec8aef91a24079b2560bd365622ecf86"
)
ARTIFACTS = {
    "model.safetensors": "6ae39addd01de6b1b8bde2cf677d43a5cd733424b8d186de3f95d1c51fee23f9",
    "config.json": "209caa9091e4632f7c8883c11170cd08ad29af68b23c09590aa4a5befb1a2a7f",
    "preprocessor_config.json": (
        "8039d1d210abaa7117ad78e58cdfd6141a2ec72c03dae891b3cd76737e422c6c"
    ),
}
RUNTIME_VERSIONS = {
    "torch": "2.5.1+cu124",
    "transformers": "4.44.2",
    "safetensors": "0.8.0",
    "numpy": "1.26.4",
}
RELEVANT_NATIVE_CLASSES = {
    "tree": 4,
    "grass": 9,
    "plant": 17,
    "water": 21,
    "sea": 26,
    "field": 29,
    "river": 60,
    "flower": 66,
    "palm": 72,
    "lake": 128,
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def verify_artifacts(model_dir: Path) -> dict[str, str]:
    resolved = model_dir.resolve(strict=True)
    if not resolved.is_dir():
        raise ValueError("SegFormer model directory is missing")
    observed = {}
    for name, expected in ARTIFACTS.items():
        path = resolved / name
        if not path.is_file() or path.is_symlink():
            raise ValueError(f"fixed SegFormer artifact is missing: {name}")
        observed[name] = sha256(path)
        if observed[name] != expected:
            raise ValueError(f"fixed SegFormer artifact SHA-256 mismatch: {name}")
    return observed


def summarize_prediction(native_classes: np.ndarray, id2label: dict[int, str]) -> dict:
    values = np.asarray(native_classes)
    if values.ndim != 2 or not np.issubdtype(values.dtype, np.integer):
        raise ValueError("SegFormer native prediction must be a 2-D integer array")
    if values.size == 0 or int(values.min()) < 0 or int(values.max()) >= 150:
        raise ValueError("SegFormer native prediction contains an invalid class id")
    counts = Counter(int(value) for value in values.ravel().tolist())
    total = int(values.size)
    top = sorted(counts.items(), key=lambda item: (-item[1], item[0]))[:10]
    return {
        "grid_hw": [int(values.shape[0]), int(values.shape[1])],
        "top_native_classes": [
            {
                "class_id": class_id,
                "class_name": id2label[class_id],
                "pixel_count": count,
                "pixel_fraction": count / total,
            }
            for class_id, count in top
        ],
        "relevant_native_classes": {
            name: {
                "class_id": class_id,
                "pixel_count": counts[class_id],
                "pixel_fraction": counts[class_id] / total,
            }
            for name, class_id in RELEVANT_NATIVE_CLASSES.items()
        },
    }


def write_json_atomic(path: Path, payload: dict) -> None:
    if path.exists() or not path.parent.is_dir():
        raise ValueError("output must be a new file in an existing directory")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        if path.exists():
            raise ValueError("output appeared during atomic write")
        os.replace(temporary, path)
    except Exception:
        temporary.unlink(missing_ok=True)
        raise


def evaluate(args: argparse.Namespace) -> dict:
    if args.runtime_image_digest != RUNTIME_IMAGE_DIGEST:
        raise ValueError("runtime image digest is outside the fixed allowlist")
    model_dir = args.model_dir.resolve(strict=True)
    manifest = args.manifest.resolve(strict=True)
    source_root = args.source_root_path.resolve(strict=True)
    output = args.output.resolve()
    artifact_hashes = verify_artifacts(model_dir)
    isolation = isolation_helpers.validate_runtime_isolation(
        [Path(__file__).resolve(), model_dir, manifest, source_root], output.parent
    )
    payload, records = g2_manifest.load_negative_manifest(
        manifest,
        source_root_id=args.source_root_id,
        source_root_path=source_root,
    )

    import cv2
    import safetensors
    import torch
    import transformers
    from PIL import Image
    from transformers import SegformerForSemanticSegmentation, SegformerImageProcessor

    versions = {
        "torch": torch.__version__,
        "transformers": transformers.__version__,
        "safetensors": safetensors.__version__,
        "numpy": np.__version__,
    }
    if versions != RUNTIME_VERSIONS:
        raise ValueError(f"fixed SegFormer runtime versions changed: {versions}")
    processor = SegformerImageProcessor.from_pretrained(
        str(model_dir), local_files_only=True
    )
    model = SegformerForSemanticSegmentation.from_pretrained(
        str(model_dir), local_files_only=True, use_safetensors=True
    )
    if tuple(model.decode_head.classifier.weight.shape) != (150, 256, 1, 1):
        raise ValueError("SegFormer classifier shape changed")
    id2label = {int(key): value for key, value in model.config.id2label.items()}
    if len(id2label) != 150 or set(id2label) != set(range(150)):
        raise ValueError("SegFormer native class order changed")
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    model.to(device).eval()

    rows = []
    started = time.perf_counter()
    with torch.inference_mode():
        for record in records:
            rgb = cv2.cvtColor(record["rgb_bgr"], cv2.COLOR_BGR2RGB)
            batch = processor(images=Image.fromarray(rgb), return_tensors="pt")
            pixels = batch["pixel_values"].to(device)
            logits = model(pixel_values=pixels).logits
            native = logits.argmax(dim=1)[0].to("cpu").numpy().astype(np.int16)
            rows.append(
                {
                    "record_id": record["record_id"],
                    "rgb_sha256": record["rgb_sha256"],
                    "semantic_sha256": record["semantic_sha256"],
                    **summarize_prediction(native, id2label),
                }
            )
    elapsed = time.perf_counter() - started
    relevant_pixels = Counter()
    total_pixels = 0
    for row in rows:
        total_pixels += row["grid_hw"][0] * row["grid_hw"][1]
        for name, values in row["relevant_native_classes"].items():
            relevant_pixels[name] += values["pixel_count"]

    return {
        "schema_version": 1,
        "protocol_id": "EMFJ6V3",
        "stage": "AREA_SEGFORMER_G2_NATIVE_NEGATIVE_DIAGNOSTIC",
        "development_only": True,
        "model_id": MODEL_ID,
        "artifacts": artifact_hashes,
        "source_area_manifest_sha256": sha256(manifest),
        "source_area_manifest_canonical_sha256": payload["manifest_sha256"],
        "source_root_id": args.source_root_id,
        "frame_count": len(rows),
        "runtime": {
            **versions,
            "device": str(device),
            "elapsed_seconds": elapsed,
            "runtime_image_digest": RUNTIME_IMAGE_DIGEST,
            "isolation": isolation,
        },
        "native_class_order_count": 150,
        "native_relevant_activation": {
            name: {
                "pixel_count": relevant_pixels[name],
                "pixel_fraction": relevant_pixels[name] / total_pixels,
            }
            for name in RELEVANT_NATIVE_CLASSES
        },
        "target_class_mapping": None,
        "target_area_metrics_computed": False,
        "a4_area_pass_computed": False,
        "license_release_allowed": False,
        "selected": False,
        "frozen": False,
        "training_performed": False,
        "truth_boundary": (
            "This diagnostic preserves the 150 native ADE20K classes. Generic water, "
            "vegetation and scene labels are not mapped to road puddle or leaf pile; "
            "therefore target Area metrics and A4 pass remain not applicable."
        ),
        "rows": rows,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model-dir", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--source-root-id", required=True)
    parser.add_argument("--source-root-path", type=Path, required=True)
    parser.add_argument("--runtime-image-digest", required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report = evaluate(args)
    write_json_atomic(args.output.resolve(), report)
    print(
        json.dumps(
            {
                "output": str(args.output),
                "records": report["frame_count"],
                "target_class_mapping": None,
                "a4_area_pass_computed": False,
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
