"""G4 model zoo and ONNX contract for AUTO-05R-2/3.

Four model families are defined here:

- ``DiscoveryDetector``: class-agnostic ``litter_candidate`` detector that
  emits an objectness heatmap, centre offset and bbox regression from
  stride-4/8 FPN-style features (fixed input ``[1, 3, 512, 384]``).
- ``CandidateCropClassifier``: four-class crop classifier
  ``background / plastic_bottle / metal_can / paper_litter``
  (input ``[1, 3, 192, 192]``).
- ``LeafSegmenter`` / ``PuddleSegmenter``: independent binary area
  segmenters with a shared-encoder option; every segmenter owns an
  independent decoder and boundary head (input ``[1, 4, 384, 512]``).
- ``build_g4_models`` / ``model_summary``: model cards with parameter
  counts, input/output names/shapes/dtypes and ``state: not_trained``.

PyTorch is an optional runtime dependency.  Every torch import is deferred to
call time (functions or module ``__getattr__``), so importing this module never
fails on hosts without torch; the torch paths are exercised by tests that
``pytest.importorskip("torch")`` first.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from .auto04_contract import box_iou, decode_centernet_outputs


DISCOVERY_INPUT_SHAPE = (1, 3, 512, 384)
CLASSIFIER_INPUT_SHAPE = (1, 3, 192, 192)
SEGMENTER_INPUT_SHAPE = (1, 4, 384, 512)
CLASSIFIER_CLASSES = ("background", "plastic_bottle", "metal_can", "paper_litter")
DISCOVERY_STRIDES = (4, 8)
SEGMENTER_TASKS = ("leaf", "puddle")
DEFAULT_ONNX_OPSET = 17

_MODEL_CLASSES: dict[str, type] = {}
_LAZY_MODEL_TYPES = frozenset(
    {
        "AreaSegmenter",
        "CandidateCropClassifier",
        "DiscoveryDetector",
        "LeafSegmenter",
        "PuddleSegmenter",
    }
)


def _torch():
    """Import torch lazily; callers must have torch available."""
    try:
        import torch
        from torch import nn
        import torch.nn.functional as functional
    except ImportError as exc:
        raise RuntimeError(
            "PyTorch is required for G4 model construction/export"
        ) from exc
    return torch, nn, functional


def _torchvision_version() -> str | None:
    try:
        import torchvision

        return torchvision.__version__
    except Exception:
        return None


def _model_classes() -> dict[str, type]:
    """Define and cache the torch-dependent model classes."""
    if _MODEL_CLASSES:
        return _MODEL_CLASSES
    torch, nn, functional = _torch()

    class _ConvBnReLU(nn.Sequential):
        def __init__(
            self,
            in_channels: int,
            out_channels: int,
            kernel_size: int = 3,
            stride: int = 1,
            padding: int = 1,
        ):
            super().__init__(
                nn.Conv2d(
                    in_channels,
                    out_channels,
                    kernel_size,
                    stride,
                    padding,
                    bias=False,
                ),
                nn.BatchNorm2d(out_channels),
                nn.ReLU(inplace=False),
            )

    class _FlatOutputMixin:
        """Flatten dict outputs into one concatenated tensor for ONNX."""

        def forward_flat(self, x):
            out = self.forward(x)
            if isinstance(out, dict):
                return torch.cat([out[name] for name in self.output_names], dim=1)
            return out

    class _FPNBackbone(nn.Module):
        """Stride-4/8 multi-scale FPN-style features for the detector."""

        def __init__(self, base: int = 32):
            super().__init__()
            self.stem = _ConvBnReLU(3, base, 3, 2, 1)
            self.conv3 = nn.Sequential(
                _ConvBnReLU(base, base, 3, 1, 1),
                _ConvBnReLU(base, base * 2, 3, 1, 1),
            )
            self.conv4 = _ConvBnReLU(base * 2, base * 4, 3, 2, 1)
            self.conv5 = _ConvBnReLU(base * 4, base * 8, 3, 2, 1)
            self.lateral4 = nn.Conv2d(base * 4, base * 2, 1)
            self.lateral5 = nn.Conv2d(base * 8, base * 2, 1)
            self.top_down = _ConvBnReLU(base * 4, base * 2, 3, 1, 1)

        def forward(self, image):
            c3 = self.conv3(self.stem(image))
            c4 = self.conv4(c3)
            c5 = self.conv5(c4)
            p5 = self.lateral5(c5)
            p4 = self.top_down(
                torch.cat(
                    (
                        functional.interpolate(
                            p5, size=c4.shape[-2:], mode="nearest"
                        ),
                        self.lateral4(c4),
                    ),
                    dim=1,
                )
            )
            return {"p4": p4, "p5": p5}

    class DiscoveryDetector(_FlatOutputMixin, nn.Module):
        """Class-agnostic litter-candidate detector (objectness/offset/bbox)."""

        task = "discovery"
        model_id = "g4_discovery_detector_v1"
        input_shape = DISCOVERY_INPUT_SHAPE
        input_names = ("image_rgb",)
        output_names = ("objectness_logits", "offset", "bbox_size")
        output_channels = {"objectness_logits": 1, "offset": 2, "bbox_size": 2}

        def __init__(self, base: int = 32):
            super().__init__()
            self.backbone = _FPNBackbone(base=base)
            self.head = nn.Sequential(
                _ConvBnReLU(base * 2, base * 2, 3, 1, 1)
            )
            self.objectness = nn.Conv2d(base * 2, 1, 3, padding=1)
            self.offset = nn.Conv2d(base * 2, 2, 3, padding=1)
            self.bbox_size = nn.Conv2d(base * 2, 2, 3, padding=1)

        def forward(self, image):
            features = self.backbone(image)
            head = self.head(features["p4"])
            return {
                "objectness_logits": self.objectness(head),
                "offset": self.offset(head),
                "bbox_size": self.bbox_size(head),
            }

    class CandidateCropClassifier(_FlatOutputMixin, nn.Module):
        """Four-class crop classifier with a global-pool classification head."""

        task = "classifier"
        model_id = "g4_candidate_crop_classifier_v1"
        input_shape = CLASSIFIER_INPUT_SHAPE
        input_names = ("crop_rgb",)
        output_names = ("logits",)
        output_channels = {"logits": len(CLASSIFIER_CLASSES)}

        def __init__(self, base: int = 32):
            super().__init__()
            self.features = nn.Sequential(
                _ConvBnReLU(3, base, 3, 2, 1),
                _ConvBnReLU(base, base * 2, 3, 2, 1),
                _ConvBnReLU(base * 2, base * 4, 3, 2, 1),
                _ConvBnReLU(base * 4, base * 4, 3, 1, 1),
            )
            self.pool = nn.AdaptiveAvgPool2d(1)
            self.head = nn.Sequential(
                nn.Flatten(),
                nn.Linear(base * 4, base * 4),
                nn.ReLU(inplace=False),
                nn.Dropout(0.3),
                nn.Linear(base * 4, len(CLASSIFIER_CLASSES)),
            )

        def forward(self, crop):
            return self.head(self.pool(self.features(crop)))

    class _AreaEncoder(nn.Module):
        """RGB-D encoder for the area segmenters (4 input channels)."""

        def __init__(self, in_channels: int = 4, base: int = 24):
            super().__init__()
            self.base = base
            self.stem = _ConvBnReLU(in_channels, base, 3, 2, 1)
            self.enc1 = nn.Sequential(
                _ConvBnReLU(base, base * 2, 3, 1, 1),
                _ConvBnReLU(base * 2, base * 2, 3, 1, 1),
            )
            self.enc2 = _ConvBnReLU(base * 2, base * 4, 3, 2, 1)
            self.enc3 = _ConvBnReLU(base * 4, base * 8, 3, 2, 1)
            self.bottleneck = nn.Sequential(
                _ConvBnReLU(base * 8, base * 8, 3, 1, 1)
            )
            self.out_channels = base * 8

        def forward(self, rgbd):
            stem = self.stem(rgbd)
            enc1 = self.enc1(stem)
            enc2 = self.enc2(enc1)
            enc3 = self.enc3(enc2)
            return {
                "stem": stem,
                "enc1": enc1,
                "enc2": enc2,
                "enc3": enc3,
                "bottleneck": self.bottleneck(enc3),
            }

    class _AreaDecoder(nn.Module):
        """Independent area decoder; consumes encoder features only."""

        def __init__(self, encoder_out_channels: int, base: int = 24):
            super().__init__()
            self.up2 = nn.Sequential(
                _ConvBnReLU(
                    encoder_out_channels + base * 4, base * 4, 3, 1, 1
                )
            )
            self.up1 = nn.Sequential(
                _ConvBnReLU(base * 4 + base * 2, base * 2, 3, 1, 1)
            )
            self.up0 = nn.Sequential(
                _ConvBnReLU(base * 2 + base, base, 3, 1, 1)
            )
            self.out = nn.Conv2d(base, 1, 1)

        def forward(self, features):
            x = functional.interpolate(
                features["bottleneck"],
                size=features["enc2"].shape[-2:],
                mode="nearest",
            )
            x = self.up2(torch.cat((x, features["enc2"]), dim=1))
            x = functional.interpolate(
                x, size=features["enc1"].shape[-2:], mode="nearest"
            )
            x = self.up1(torch.cat((x, features["enc1"]), dim=1))
            x = functional.interpolate(
                x, size=features["stem"].shape[-2:], mode="nearest"
            )
            x = self.up0(torch.cat((x, features["stem"]), dim=1))
            fused = functional.interpolate(
                x,
                size=(
                    features["stem"].shape[-2] * 2,
                    features["stem"].shape[-1] * 2,
                ),
                mode="nearest",
            )
            return {"logits": self.out(fused), "fused": fused}

    class _BoundaryHead(nn.Module):
        """Independent boundary head kept separate from the area decoder."""

        def __init__(self, in_channels: int):
            super().__init__()
            self.conv = _ConvBnReLU(in_channels, in_channels, 3, 1, 1)
            self.out = nn.Conv2d(in_channels, 1, 1)

        def forward(self, x):
            return self.out(self.conv(x))

    class AreaSegmenter(_FlatOutputMixin, nn.Module):
        """Shared base for leaf/puddle binary segmentation.

        A shared encoder is allowed, but every segmenter owns an independent
        decoder and boundary head so area-specific features stay separate.
        """

        task: str | None = None
        model_id: str | None = None
        input_shape = SEGMENTER_INPUT_SHAPE
        input_names = ("rgbd",)
        output_names = ("logits", "boundary_logits")
        output_channels = {"logits": 1, "boundary_logits": 1}

        def __init__(self, encoder=None, base: int = 24):
            super().__init__()
            self.encoder = encoder if encoder is not None else _AreaEncoder(base=base)
            self.decoder = _AreaDecoder(self.encoder.out_channels, base=base)
            self.boundary_head = _BoundaryHead(base)

        def forward(self, rgbd):
            features = self.encoder(rgbd)
            decoded = self.decoder(features)
            return {
                "logits": decoded["logits"],
                "boundary_logits": self.boundary_head(decoded["fused"]),
            }

    class LeafSegmenter(AreaSegmenter):
        task = "leaf"
        model_id = "g4_leaf_segmenter_v1"

    class PuddleSegmenter(AreaSegmenter):
        task = "puddle"
        model_id = "g4_puddle_segmenter_v1"

    class _FlatForwardWrapper(nn.Module):
        """Wraps a model for single-tensor ONNX export."""

        def __init__(self, model):
            super().__init__()
            self.model = model

        def forward(self, x):
            return self.model.forward_flat(x)

    _MODEL_CLASSES.update(
        {
            "_AreaEncoder": _AreaEncoder,
            "_FlatForwardWrapper": _FlatForwardWrapper,
            "AreaSegmenter": AreaSegmenter,
            "CandidateCropClassifier": CandidateCropClassifier,
            "DiscoveryDetector": DiscoveryDetector,
            "LeafSegmenter": LeafSegmenter,
            "PuddleSegmenter": PuddleSegmenter,
        }
    )
    return _MODEL_CLASSES


def __getattr__(name: str):
    if name in _LAZY_MODEL_TYPES:
        return _model_classes()[name]
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def build_g4_models(shared_encoder: bool = False) -> dict:
    """Build the four G4 models; ``shared_encoder`` shares only the encoder."""
    classes = _model_classes()
    models = {
        "discovery": classes["DiscoveryDetector"](),
        "classifier": classes["CandidateCropClassifier"](),
    }
    if shared_encoder:
        encoder = classes["_AreaEncoder"]()
        models["leaf"] = classes["LeafSegmenter"](encoder=encoder)
        models["puddle"] = classes["PuddleSegmenter"](encoder=encoder)
    else:
        models["leaf"] = classes["LeafSegmenter"]()
        models["puddle"] = classes["PuddleSegmenter"]()
    return models


def _output_shapes(flat_shape: tuple[int, ...], model) -> list[list[int]]:
    channels = model.output_channels
    if len(flat_shape) == 2:
        return [[1, channels[name]] for name in model.output_names]
    _, _, height, width = flat_shape
    return [
        [1, channels[name], height, width] for name in model.output_names
    ]


def model_summary(models: dict | None = None, dummy_inputs: dict | None = None) -> dict:
    """Return model cards with parameter counts and IO shapes/dtypes."""
    torch, _, _ = _torch()
    if models is None:
        models = build_g4_models()
    torchvision_version = _torchvision_version()
    cards = {}
    for task, model in models.items():
        dummy = (
            dummy_inputs[task]
            if dummy_inputs is not None
            else torch.zeros(tuple(model.input_shape))
        )
        model.eval()
        with torch.no_grad():
            flat = model.forward_flat(dummy)
        cards[task] = {
            "model_id": model.model_id,
            "task": task,
            "state": "not_trained",
            "parameter_count": int(
                sum(parameter.numel() for parameter in model.parameters())
            ),
            "inputs": [
                {
                    "name": model.input_names[0],
                    "shape": list(model.input_shape),
                    "dtype": str(dummy.dtype).replace("torch.", ""),
                }
            ],
            "outputs": [
                {
                    "name": name,
                    "shape": shape,
                    "dtype": str(flat.dtype).replace("torch.", ""),
                }
                for name, shape in zip(
                    model.output_names,
                    _output_shapes(tuple(flat.shape), model),
                )
            ],
            "framework": {
                "torch": torch.__version__,
                "torchvision": torchvision_version,
            },
        }
    return cards


def export_fixed_onnx(model, dummy_input, output_path, opset: int = DEFAULT_ONNX_OPSET) -> str:
    """Export a fixed-shape ONNX graph (``dynamic_axes=None``, no custom ops)."""
    torch, _, _ = _torch()
    classes = _model_classes()
    model.eval()
    if dummy_input is None:
        dummy_input = torch.zeros(tuple(model.input_shape))
    wrapper = classes["_FlatForwardWrapper"](model)
    torch.onnx.export(
        wrapper,
        dummy_input,
        str(output_path),
        input_names=list(model.input_names),
        output_names=["outputs"],
        opset_version=int(opset),
        dynamic_axes=None,
        do_constant_folding=True,
    )
    import onnx

    onnx_model = onnx.load(str(output_path))
    onnx.checker.check_model(onnx_model)
    for graph_input in onnx_model.graph.input:
        for dim in graph_input.type.tensor_type.shape.dim:
            if dim.dim_param:
                raise RuntimeError(
                    "exported ONNX must use fixed shapes (dynamic_axes=None)"
                )
    return str(output_path)


def operator_inventory(onnx_path) -> dict:
    """Count ONNX operator types in an exported graph."""
    import onnx

    onnx_model = onnx.load(str(onnx_path))
    counts: dict[str, int] = {}
    for node in onnx_model.graph.node:
        counts[node.op_type] = counts.get(node.op_type, 0) + 1
    return counts


def decode_discovery_flat(
    flat,
    *,
    score_threshold: float = 0.5,
    max_detections: int | None = None,
):
    """Decode the flattened detector output ``[N, 5, H, W]`` to detections."""
    flat = np.asarray(flat, dtype=np.float32)
    if flat.ndim != 4 or flat.shape[1] != 5:
        raise ValueError("discovery flat output must be Nx5xHxW")
    objectness = 1.0 / (1.0 + np.exp(-flat[:, 0:1]))
    offset = flat[:, 1:3]
    bbox_size = flat[:, 3:5]
    return decode_centernet_outputs(
        objectness[0],
        offset[0],
        bbox_size[0],
        stride=4,
        score_threshold=score_threshold,
        max_detections=max_detections,
    )


def _argmax_agreement(torch_flat, onnx_flat) -> float | None:
    torch_np = torch_flat.detach().cpu().numpy()
    onnx_np = np.asarray(onnx_flat)
    if torch_np.ndim == 4 and torch_np.shape[1] > 1:
        return float((torch_np.argmax(axis=1) == onnx_np.argmax(axis=1)).mean())
    if torch_np.ndim == 2 and torch_np.shape[1] > 1:
        return float((torch_np.argmax(axis=1) == onnx_np.argmax(axis=1)).mean())
    return None


def _decoded_agreement(model, torch_flat, onnx_flat) -> bool | None:
    if getattr(model, "task", None) != "discovery":
        return None
    torch_detections = decode_discovery_flat(
        torch_flat.detach().cpu().numpy(), score_threshold=0.75
    )
    onnx_detections = decode_discovery_flat(
        np.asarray(onnx_flat), score_threshold=0.75
    )
    if abs(len(torch_detections) - len(onnx_detections)) > 1:
        return False
    matched = 0
    used: set[int] = set()
    for det in torch_detections:
        for index, other in enumerate(onnx_detections):
            if index in used:
                continue
            if (
                det.class_index == other.class_index
                and box_iou(det.bbox_xyxy, other.bbox_xyxy) >= 0.9
                and abs(det.score - other.score) <= 0.05
            ):
                used.add(index)
                matched += 1
                break
    return matched >= len(torch_detections) - 1 and matched >= len(
        onnx_detections
    ) - 1


def torch_onnx_parity(model, onnx_session: Any, inputs) -> dict:
    """Compare torch forward vs ONNX Runtime outputs on the same inputs."""
    torch, _, _ = _torch()
    model.eval()
    with torch.no_grad():
        torch_flat = model.forward_flat(inputs).detach().cpu()
    onnx_flat = np.asarray(
        onnx_session.run(None, {model.input_names[0]: inputs.numpy()})[0]
    )
    max_error = float(np.abs(torch_flat.numpy() - onnx_flat).max())
    return {
        "max_absolute_error": max_error,
        "argmax_agreement": _argmax_agreement(torch_flat, onnx_flat),
        "decoded_agreement": _decoded_agreement(model, torch_flat, onnx_flat),
    }


__all__ = [
    "AreaSegmenter",
    "CLASSIFIER_CLASSES",
    "CLASSIFIER_INPUT_SHAPE",
    "CandidateCropClassifier",
    "DEFAULT_ONNX_OPSET",
    "DISCOVERY_INPUT_SHAPE",
    "DISCOVERY_STRIDES",
    "DiscoveryDetector",
    "LeafSegmenter",
    "PuddleSegmenter",
    "SEGMENTER_INPUT_SHAPE",
    "SEGMENTER_TASKS",
    "build_g4_models",
    "decode_discovery_flat",
    "export_fixed_onnx",
    "model_summary",
    "operator_inventory",
    "torch_onnx_parity",
]
