"""EdgeSAM ONNX box-prompt adapter used after DOSOD discovery."""

from __future__ import annotations

from pathlib import Path

import numpy as np


class EdgeSamOnnxSegmenter:
    """Run the locked EdgeSAM-3x encoder and decoder using box prompts."""

    image_size = 1024
    pixel_mean = np.asarray([123.675, 116.28, 103.53], dtype=np.float32)
    pixel_std = np.asarray([58.395, 57.12, 57.375], dtype=np.float32)

    def __init__(
        self,
        encoder_path: str | Path | None = None,
        decoder_path: str | Path | None = None,
        *,
        encoder_session=None,
        decoder_session=None,
    ) -> None:
        if encoder_session is None or decoder_session is None:
            if not encoder_path or not Path(encoder_path).is_file():
                raise FileNotFoundError(f"EdgeSAM encoder missing: {encoder_path}")
            if not decoder_path or not Path(decoder_path).is_file():
                raise FileNotFoundError(f"EdgeSAM decoder missing: {decoder_path}")
            import onnxruntime as ort

            encoder_session = ort.InferenceSession(
                str(encoder_path), providers=["CPUExecutionProvider"]
            )
            decoder_session = ort.InferenceSession(
                str(decoder_path), providers=["CPUExecutionProvider"]
            )
        self.encoder = encoder_session
        self.decoder = decoder_session
        if {item.name for item in self.encoder.get_inputs()} != {"image"}:
            raise ValueError("EdgeSAM encoder must expose the image input")
        if {item.name for item in self.decoder.get_inputs()} != {
            "image_embeddings",
            "point_coords",
            "point_labels",
        }:
            raise ValueError("EdgeSAM decoder input contract mismatch")

    def _encode(self, rgb_image: np.ndarray) -> tuple[np.ndarray, float, tuple[int, int]]:
        if rgb_image.ndim != 3 or rgb_image.shape[2] != 3 or rgb_image.size == 0:
            raise ValueError("EdgeSAM input must be a non-empty HxWx3 RGB image")
        import cv2

        height, width = rgb_image.shape[:2]
        scale = self.image_size / float(max(height, width))
        resized_height = int(height * scale + 0.5)
        resized_width = int(width * scale + 0.5)
        resized = cv2.resize(
            rgb_image, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR
        ).astype(np.float32)
        tensor = np.zeros((self.image_size, self.image_size, 3), dtype=np.float32)
        tensor[:resized_height, :resized_width] = resized
        tensor = tensor.transpose(2, 0, 1)[None]
        tensor = (tensor - self.pixel_mean[None, :, None, None]) / self.pixel_std[
            None, :, None, None
        ]
        features = self.encoder.run(None, {"image": tensor})[0]
        features = np.asarray(features, dtype=np.float32)
        if features.shape != (1, 256, 64, 64) or not np.isfinite(features).all():
            raise RuntimeError("EdgeSAM encoder returned invalid features")
        return features, scale, (resized_height, resized_width)

    def segment_boxes(self, rgb_image: np.ndarray, boxes_xyxy: np.ndarray) -> tuple[list[np.ndarray], list[float]]:
        boxes = np.asarray(boxes_xyxy, dtype=np.float32).reshape(-1, 4)
        if not np.isfinite(boxes).all():
            raise ValueError("EdgeSAM box prompts must be finite")
        if not len(boxes):
            return [], []
        import cv2

        features, scale, (resized_height, resized_width) = self._encode(rgb_image)
        masks: list[np.ndarray] = []
        qualities: list[float] = []
        for box in boxes:
            coords = (box.reshape(1, 2, 2) * scale).astype(np.float32)
            labels = np.asarray([[2.0, 3.0]], dtype=np.float32)
            scores, low_res_masks = self.decoder.run(
                None,
                {
                    "image_embeddings": features,
                    "point_coords": coords,
                    "point_labels": labels,
                },
            )
            scores = np.asarray(scores, dtype=np.float32).reshape(-1)
            low_res_masks = np.asarray(low_res_masks, dtype=np.float32)[0]
            if not (np.isfinite(scores).all() and np.isfinite(low_res_masks).all()):
                raise RuntimeError("EdgeSAM decoder returned non-finite outputs")
            best = int(np.argmax(scores))
            mask = cv2.resize(
                low_res_masks[best],
                (self.image_size, self.image_size),
                interpolation=cv2.INTER_LINEAR,
            )[:resized_height, :resized_width]
            height, width = rgb_image.shape[:2]
            mask = cv2.resize(mask, (width, height), interpolation=cv2.INTER_LINEAR) > 0.0
            masks.append(mask)
            qualities.append(float(scores[best]))
        return masks, qualities

