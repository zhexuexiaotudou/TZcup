"""Official Torchvision FCOS teacher for the P2 data-learnability gate."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np

from .g4_data import (
    DISCOVERY_MODEL_SIZE,
    discrete_boxes_for_frame,
    read_rgb,
)
from .g4_evaluation import discovery_metrics
from .g4_pretrained import provenance_record, torchvision_cache_path


TEACHER_ARCHITECTURE = "torchvision_fcos_resnet50_fpn"
TEACHER_WEIGHT_SPEC = "fcos_resnet50_fpn_coco"
TEACHER_INPUT_SIZE = DISCOVERY_MODEL_SIZE
TEACHER_GATE_RECALL = 0.85
TEACHER_GATE_FALSE_CANDIDATES_PER_MIN = 10.0
TEACHER_THRESHOLDS = tuple(
    round(value, 2) for value in np.arange(0.05, 0.96, 0.05)
)
TEACHER_REQUIRED_DATA_GATES = (
    "scene_pose_reset_contract_100_percent",
    "manifest_pixel_target_consistency_100_percent",
)


def _torch():
    try:
        import torch
    except ImportError as exc:
        raise RuntimeError("PyTorch is required for the FCOS teacher") from exc
    return torch


def require_teacher_dataset_gate(evidence_dir: str | Path) -> dict:
    """Fail closed unless the full, current G4 dataset QA is green."""
    qa_path = Path(evidence_dir) / "g4_dataset_qa.json"
    if not qa_path.is_file():
        raise RuntimeError(f"P2 teacher requires dataset QA: {qa_path}")
    payload = json.loads(qa_path.read_text(encoding="utf-8"))
    gates = payload.get("gates", {})
    missing = [name for name in TEACHER_REQUIRED_DATA_GATES if name not in gates]
    failed = [
        name for name in TEACHER_REQUIRED_DATA_GATES if gates.get(name) is not True
    ]
    if (
        payload.get("G4_dataset_gate_pass") is not True
        or payload.get("quality_gates_pass") is not True
        or payload.get("full_capture_executed") is not True
        or missing
        or failed
    ):
        raise RuntimeError(
            "P2 teacher dataset gate is not green: "
            f"G4={payload.get('G4_dataset_gate_pass')} "
            f"quality={payload.get('quality_gates_pass')} "
            f"full_capture={payload.get('full_capture_executed')} "
            f"missing={missing} failed={failed}"
        )
    return {
        "path": qa_path.name,
        "sha256": hashlib.sha256(qa_path.read_bytes()).hexdigest(),
        "G4_dataset_gate_pass": True,
        "quality_gates_pass": True,
        "full_capture_executed": True,
        "required_gates": {name: True for name in TEACHER_REQUIRED_DATA_GATES},
    }


def build_fcos_teacher():
    """Build a one-class FCOS teacher from exact official COCO weights."""
    torch = _torch()
    try:
        import torchvision
    except ImportError as exc:
        raise RuntimeError("torchvision is required for the FCOS teacher") from exc

    weights = torchvision.models.detection.FCOS_ResNet50_FPN_Weights.COCO_V1
    try:
        model = torchvision.models.detection.fcos_resnet50_fpn(
            weights=weights,
            min_size=TEACHER_INPUT_SIZE[1],
            max_size=TEACHER_INPUT_SIZE[0],
            box_score_thresh=0.01,
            box_detections_per_img=100,
            topk_candidates=1000,
        )
    except Exception as exc:
        raise RuntimeError(
            "official FCOS ResNet50-FPN COCO weights are required"
        ) from exc
    classification = model.head.classification_head
    replacement = torch.nn.Conv2d(256, 1, kernel_size=3, stride=1, padding=1)
    torch.nn.init.normal_(replacement.weight, std=0.01)
    torch.nn.init.constant_(
        replacement.bias, -math.log((1.0 - 0.01) / 0.01)
    )
    classification.cls_logits = replacement
    classification.num_classes = 1
    model.provenance = provenance_record(
        TEACHER_WEIGHT_SPEC,
        cache_path=torchvision_cache_path(TEACHER_WEIGHT_SPEC),
        torchvision_version=torchvision.__version__,
    )
    model.architecture_role = "reference_teacher_not_default_deployable"
    model.model_id = "g4_fcos_resnet50_fpn_teacher_v1"
    return model


class FCOSDiscoveryDataset:
    """RGB-only fixed-size frames and one-class FCOS targets."""

    def __init__(
        self,
        rows: list[dict],
        instances_by_key: dict[tuple[int, int], list[dict]],
    ):
        self.rows = list(rows)
        self.instances_by_key = instances_by_key

    def __len__(self) -> int:
        return len(self.rows)

    def __getitem__(self, index: int):
        torch = _torch()
        row = self.rows[index]
        rgb = read_rgb(row)
        native_size = (int(rgb.shape[1]), int(rgb.shape[0]))
        truth = discrete_boxes_for_frame(
            row,
            self.instances_by_key,
            native_size=native_size,
            model_size=TEACHER_INPUT_SIZE,
        )
        resized = cv2.resize(
            rgb, TEACHER_INPUT_SIZE, interpolation=cv2.INTER_AREA
        )
        image = torch.from_numpy(
            np.ascontiguousarray(
                resized.transpose(2, 0, 1), dtype=np.float32
            )
            / 255.0
        )
        boxes = torch.as_tensor(
            [item["bbox_xyxy"] for item in truth], dtype=torch.float32
        ).reshape(-1, 4)
        labels = torch.zeros((len(truth),), dtype=torch.int64)
        return image, {"boxes": boxes, "labels": labels}, row


def fcos_collate(batch):
    images, targets, rows = zip(*batch)
    return list(images), list(targets), list(rows)


def teacher_predictions(
    model,
    rows: list[dict],
    instances_by_key: dict[tuple[int, int], list[dict]],
    *,
    device,
    score_threshold: float = 0.01,
    batch_size: int = 4,
) -> list[dict]:
    """Run batched teacher inference and return discovery-metric frames."""
    torch = _torch()
    from torch.utils.data import DataLoader

    dataset = FCOSDiscoveryDataset(rows, instances_by_key)
    loader = DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=False,
        num_workers=0,
        collate_fn=fcos_collate,
    )
    model.eval()
    frames: list[dict] = []
    with torch.no_grad():
        for images, targets, batch_rows in loader:
            outputs = model([image.to(device) for image in images])
            for output, target, row in zip(outputs, targets, batch_rows):
                scores = output["scores"].detach().cpu()
                keep = scores >= score_threshold
                boxes = output["boxes"].detach().cpu()[keep]
                selected_scores = scores[keep]
                truth = [
                    {
                        "class_index": 0,
                        "semantic_class": "litter_candidate",
                        "bbox_xyxy": box.tolist(),
                    }
                    for box in target["boxes"]
                ]
                frames.append(
                    {
                        "row": row,
                        "scene_seed": int(row["scene_seed"]),
                        "frame_index": int(row["frame_index"]),
                        "split": row["split"],
                        "world_id": row["world_id"],
                        "negative_only": bool(row.get("negative_only", False)),
                        "detections": [
                            {
                                "class_index": 0,
                                "score": float(score),
                                "bbox_xyxy": box.tolist(),
                            }
                            for box, score in zip(boxes, selected_scores)
                        ],
                        "truth": truth,
                    }
                )
    return frames


def filter_prediction_frames(frames: Iterable[dict], threshold: float) -> list[dict]:
    return [
        {
            **frame,
            "detections": [
                item
                for item in frame["detections"]
                if float(item["score"]) >= threshold
            ],
        }
        for frame in frames
    ]


def teacher_gate(metrics: dict) -> dict:
    gates = {
        "candidate_recall_at_least_0_85": (
            metrics["all_gt_candidate_recall"] >= TEACHER_GATE_RECALL
        ),
        "false_candidates_per_min_at_most_10": (
            metrics["false_candidates_per_min"]
            <= TEACHER_GATE_FALSE_CANDIDATES_PER_MIN
        ),
    }
    return {"gates": gates, "all_pass": all(gates.values())}


def teacher_constraint_distance(metrics: dict) -> float:
    return (
        max(0.0, TEACHER_GATE_RECALL - metrics["all_gt_candidate_recall"])
        / TEACHER_GATE_RECALL
        + max(
            0.0,
            metrics["false_candidates_per_min"]
            - TEACHER_GATE_FALSE_CANDIDATES_PER_MIN,
        )
        / TEACHER_GATE_FALSE_CANDIDATES_PER_MIN
    )


def select_teacher_threshold(frames: list[dict]) -> tuple[float, dict, list[dict]]:
    sweep = []
    for threshold in TEACHER_THRESHOLDS:
        metrics = discovery_metrics(filter_prediction_frames(frames, threshold))
        verdict = teacher_gate(metrics)
        sweep.append(
            {
                "threshold": threshold,
                "metrics": metrics,
                **verdict,
                "constraint_distance": teacher_constraint_distance(metrics),
            }
        )
    selected = min(
        sweep,
        key=lambda item: (
            not item["all_pass"],
            item["constraint_distance"],
            -item["metrics"]["all_gt_candidate_recall"],
            item["metrics"]["false_candidates_per_min"],
            -item["threshold"],
        ),
    )
    return float(selected["threshold"]), selected, sweep


__all__ = [
    "FCOSDiscoveryDataset",
    "TEACHER_ARCHITECTURE",
    "TEACHER_GATE_FALSE_CANDIDATES_PER_MIN",
    "TEACHER_GATE_RECALL",
    "TEACHER_INPUT_SIZE",
    "TEACHER_THRESHOLDS",
    "TEACHER_WEIGHT_SPEC",
    "build_fcos_teacher",
    "fcos_collate",
    "filter_prediction_frames",
    "teacher_gate",
    "teacher_constraint_distance",
    "teacher_predictions",
    "require_teacher_dataset_gate",
    "select_teacher_threshold",
]
