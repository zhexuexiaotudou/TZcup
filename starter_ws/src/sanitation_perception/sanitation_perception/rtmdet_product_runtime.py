"""Hash-bound MMDetection RTMDet runtime for the CRV6 x86 product path."""

from __future__ import annotations

import hashlib
from pathlib import Path


CLASS_NAMES = ("plastic_bottle", "metal_can", "paper_litter")


def file_sha256(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def decode_rtmdet_result(result, *, observation_threshold: float, action_threshold: float, top_k: int = 100) -> list[dict]:
    """Convert native MMDet coordinates/scores without an extra NMS or resize."""
    instances = result.pred_instances.to("cpu")
    rows = [
        {
            "class_name": CLASS_NAMES[int(label)],
            "class_id": CLASS_NAMES[int(label)],
            "label": int(label),
            "score": float(score),
            "confidence": float(score),
            "bbox_xyxy": [float(value) for value in bbox],
            "actionable": float(score) >= float(action_threshold),
            "source_backend": "mmdetection_rtmdet_s",
        }
        for bbox, score, label in zip(
            instances.bboxes.tolist(), instances.scores.tolist(), instances.labels.tolist()
        )
        if float(score) >= float(observation_threshold)
    ]
    return sorted(rows, key=lambda item: (-item["score"], item["class_name"], item["bbox_xyxy"]))[:top_k]


class RTMDetProductRuntime:
    """The production-callable PyTorch x86 detector boundary used by CRV6."""

    def __init__(
        self, config: str | Path, checkpoint: str | Path, *, expected_sha256: str,
        observation_threshold: float, action_threshold: float, device: str = "cuda:0", top_k: int = 100,
    ):
        actual = file_sha256(checkpoint)
        if actual != expected_sha256:
            raise RuntimeError(f"RTMDet checkpoint SHA-256 mismatch: {actual} != {expected_sha256}")
        if not 0.0 <= observation_threshold <= action_threshold <= 1.0:
            raise ValueError("RTMDet observation/action thresholds are invalid")
        from mmdet.apis import init_detector
        self.model = init_detector(str(config), str(checkpoint), device=device)
        self.checkpoint_sha256 = actual
        self.observation_threshold = float(observation_threshold)
        self.action_threshold = float(action_threshold)
        self.top_k = int(top_k)

    def infer_bgr(self, bgr):
        """Infer one native 640x480 BGR frame through the product entry point."""
        from mmdet.apis import inference_detector
        result = inference_detector(self.model, bgr)
        return decode_rtmdet_result(
            result, observation_threshold=self.observation_threshold,
            action_threshold=self.action_threshold, top_k=self.top_k,
        )
