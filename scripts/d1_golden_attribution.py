#!/usr/bin/env python3
"""Build a 10-frame D1 evaluator golden unit without reading sealed data."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

from PIL import Image, ImageDraw


CATEGORY_NAMES = {1: "plastic_bottle", 2: "metal_can", 3: "paper_litter"}
SOURCE_TO_TARGET = {1: 1, 2: 2, 7: 3}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def iou(box: list[float], other: list[float]) -> float:
    x1, y1 = max(box[0], other[0]), max(box[1], other[1])
    x2, y2 = min(box[2], other[2]), min(box[3], other[3])
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    first_area = max(0.0, box[2] - box[0]) * max(0.0, box[3] - box[1])
    second_area = max(0.0, other[2] - other[0]) * max(0.0, other[3] - other[1])
    union = first_area + second_area - intersection
    return intersection / union if union else 0.0


def letterbox_roundtrip(box: list[float], width: int, height: int) -> tuple[list[float], float]:
    ratio = min(640.0 / width, 640.0 / height)
    resized_width = round(width * ratio)
    resized_height = round(height * ratio)
    pad_x = (640.0 - resized_width) / 2.0
    pad_y = (640.0 - resized_height) / 2.0
    encoded = [
        box[0] * ratio + pad_x,
        box[1] * ratio + pad_y,
        box[2] * ratio + pad_x,
        box[3] * ratio + pad_y,
    ]
    decoded = [
        (encoded[0] - pad_x) / ratio,
        (encoded[1] - pad_y) / ratio,
        (encoded[2] - pad_x) / ratio,
        (encoded[3] - pad_y) / ratio,
    ]
    return decoded, max(abs(left - right) for left, right in zip(box, decoded))


def select_frames(images: list[dict[str, Any]], count: int = 10) -> list[dict[str, Any]]:
    annotated = [item for item in images if item.get("annotations")]
    ranked = sorted(
        annotated,
        key=lambda item: max(
            float(annotation["bbox_short_side_px"])
            for annotation in item["annotations"]
        ),
        reverse=True,
    )
    selected: list[dict[str, Any]] = []
    for category_id in CATEGORY_NAMES:
        selected.extend(
            item
            for item in ranked
            if any(int(annotation["category_id"]) == category_id for annotation in item["annotations"])
            and item not in selected
        )
        selected = selected[: min(len(selected), 3 * category_id)]
    for item in ranked:
        if item not in selected:
            selected.append(item)
        if len(selected) == count:
            break
    if len(selected) != count:
        raise RuntimeError(f"golden unit requires {count} unique annotated frames")
    return selected


def render_frame(
    source: Path,
    destination: Path,
    annotations: list[dict[str, Any]],
    predictions: list[dict[str, Any]],
) -> None:
    image = Image.open(source).convert("RGB")
    draw = ImageDraw.Draw(image)
    for annotation in annotations:
        box = [float(value) for value in annotation["bbox_xyxy"]]
        name = CATEGORY_NAMES[int(annotation["category_id"])]
        draw.rectangle(box, outline=(0, 255, 0), width=3)
        draw.text((box[0] + 2, box[1] + 2), f"GT {name}", fill=(0, 255, 0))
    for prediction in predictions:
        if float(prediction["confidence"]) < 0.5:
            continue
        box = [float(value) for value in prediction["bbox_xyxy"]]
        target_id = prediction.get("target_category_id")
        name = CATEGORY_NAMES.get(target_id, f"source_{prediction['source_class_index']}")
        draw.rectangle(box, outline=(255, 0, 0), width=2)
        draw.text(
            (box[0] + 2, max(0.0, box[1] - 12)),
            f"PT {name} {float(prediction['confidence']):.3f}",
            fill=(255, 0, 0),
        )
    destination.parent.mkdir(parents=True, exist_ok=True)
    image.save(destination)


def build_contact_sheet(rendered: list[Path], output: Path) -> None:
    tiles = []
    for path in rendered:
        image = Image.open(path).convert("RGB")
        image.thumbnail((640, 360))
        tile = Image.new("RGB", (640, 380), "white")
        tile.paste(image, ((640 - image.width) // 2, 20))
        ImageDraw.Draw(tile).text((5, 3), path.stem, fill="black")
        tiles.append(tile)
    sheet = Image.new("RGB", (1280, 380 * 5), "white")
    for index, tile in enumerate(tiles):
        sheet.paste(tile, ((index % 2) * 640, (index // 2) * 380))
    output.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(output)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selection", type=Path, required=True)
    parser.add_argument("--inference", type=Path, required=True)
    parser.add_argument("--development-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--render-dir", type=Path, required=True)
    arguments = parser.parse_args()
    selection = json.loads(arguments.selection.read_text(encoding="utf-8"))
    inference = json.loads(arguments.inference.read_text(encoding="utf-8"))
    if selection.get("source_split") not in {None, "train"}:
        raise RuntimeError("golden selection must be TRAIN-only")
    if any(value is not False for value in selection["forbidden_read_flags"].values()):
        raise RuntimeError("golden selection records forbidden data access")
    selected = select_frames(selection["images"])
    inference_by_id = {int(item["image_id"]): item for item in inference["images"]}
    records = []
    rendered = []
    for index, item in enumerate(selected, start=1):
        source = (arguments.development_root / item["relative_path"]).resolve()
        if not source.is_relative_to(arguments.development_root.resolve()):
            raise RuntimeError("golden image escapes development root")
        predictions = inference_by_id[int(item["image_id"])]["predictions"]
        checks = []
        for annotation in item["annotations"]:
            box = [float(value) for value in annotation["bbox_xyxy"]]
            width, height = int(item["width"]), int(item["height"])
            decoded, error = letterbox_roundtrip(box, width, height)
            checks.append(
                {
                    "annotation_id": int(annotation["annotation_id"]),
                    "category_id": int(annotation["category_id"]),
                    "category_name": CATEGORY_NAMES[int(annotation["category_id"])],
                    "bbox_within_image": (
                        0 <= box[0] < box[2] <= width and 0 <= box[1] < box[3] <= height
                    ),
                    "bbox_self_iou": iou(box, box),
                    "letterbox_inverse_max_error_px": error,
                    "letterbox_decoded_box": decoded,
                }
            )
        rendered_path = arguments.render_dir / f"{index:02d}_image_{item['image_id']}.png"
        render_frame(source, rendered_path, item["annotations"], predictions)
        rendered.append(rendered_path)
        records.append(
            {
                "image_id": int(item["image_id"]),
                "relative_path": item["relative_path"],
                "sha256": sha256(source),
                "largest_gt_short_side_px": max(
                    float(value["bbox_short_side_px"]) for value in item["annotations"]
                ),
                "checks": checks,
                "native_prediction_count_at_0_5": sum(
                    float(value["confidence"]) >= 0.5 for value in predictions
                ),
                "render": str(rendered_path),
            }
        )
    contact_sheet = arguments.render_dir / "D1_GOLDEN_CONTACT_SHEET.png"
    build_contact_sheet(rendered, contact_sheet)
    all_checks = [check for record in records for check in record["checks"]]
    report = {
        "schema_version": 1,
        "stage": "EMFJ6V3_A0_3_EVALUATOR_GOLDEN_UNIT",
        "development_only": True,
        "sealed_access_allowed": False,
        "selection_sha256": sha256(arguments.selection),
        "native_inference_sha256": sha256(arguments.inference),
        "source_to_target_class_index": SOURCE_TO_TARGET,
        "frame_count": len(records),
        "category_coverage": sorted(
            {check["category_name"] for check in all_checks}
        ),
        "bbox_mapping_pass": all(check["bbox_within_image"] for check in all_checks),
        "iou_self_check_pass": all(check["bbox_self_iou"] == 1.0 for check in all_checks),
        "letterbox_inverse_pass": all(
            check["letterbox_inverse_max_error_px"] <= 1e-9 for check in all_checks
        ),
        "class_index_check_pass": SOURCE_TO_TARGET == {1: 1, 2: 2, 7: 3},
        "contact_sheet": str(contact_sheet),
        "visual_review": "not_run",
        "records": records,
    }
    report["golden_unit_machine_pass"] = all(
        report[key]
        for key in (
            "bbox_mapping_pass",
            "iou_self_check_pass",
            "letterbox_inverse_pass",
            "class_index_check_pass",
        )
    ) and report["frame_count"] >= 10 and len(report["category_coverage"]) == 3
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps({key: report[key] for key in (
        "frame_count", "category_coverage", "golden_unit_machine_pass", "contact_sheet"
    )}, indent=2))
    return 0 if report["golden_unit_machine_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
