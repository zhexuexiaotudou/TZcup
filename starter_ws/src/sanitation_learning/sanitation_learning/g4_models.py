"""G4 model zoo and ONNX contract for AUTO-05R-2/3.

Four model families are defined here:

- ``DiscoveryDetector``: class-agnostic FCOS-lite ``litter_candidate``
  detector with a pretrained ResNet18 P3/P4/P5 pyramid, objectness, quality
  and internal ltrb regression. It preserves P3/P4/P5 as separate fixed
  output channels while top-K and cross-level NMS remain outside ONNX.
- ``CandidateCropClassifier``: four-class crop classifier
  ``background / plastic_bottle / metal_can / paper_litter``
  (input ``[1, 3, 192, 192]``).
- ``LeafSegmenter`` / ``PuddleSegmenter``: independent binary area
  segmenters with a shared-encoder option; the original three-channel RGB
  pretrained stem is preserved and fused stage-wise with a seven-channel
  geometry branch. Every segmenter owns an independent decoder and a boundary
  head fed by decoder features (input ``[1, 10, 384, 512]``).
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
from .g4_onnx_parity import task_specific_parity
from .g4_pretrained import (
    from_scratch_control_record,
    pretrained_backbone_spec,
    provenance_record,
    torchvision_cache_path,
)


DISCOVERY_INPUT_SHAPE = (1, 3, 480, 640)
CLASSIFIER_INPUT_SHAPE = (1, 3, 192, 192)
SEGMENTER_INPUT_SHAPE = (1, 10, 384, 512)
CLASSIFIER_CLASSES = ("background", "plastic_bottle", "metal_can", "paper_litter")
DISCOVERY_STRIDES = (4, 8, 16)
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

        def __init__(self, base: int = 48):
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

    class _DiscoveryResNetBackbone(nn.Module):
        """Pretrained ResNet18 FPN with genuine stride-4/8/16 levels."""

        def __init__(self, base: int = 48, from_scratch_control: bool = False):
            super().__init__()
            import torchvision

            weights_enum = torchvision.models.ResNet18_Weights.IMAGENET1K_V1
            if from_scratch_control:
                resnet = torchvision.models.resnet18(weights=None)
                self.provenance = from_scratch_control_record("resnet18")
            else:
                try:
                    resnet = torchvision.models.resnet18(
                        weights=weights_enum
                    )
                except Exception as exc:
                    raise RuntimeError(
                        "official pretrained ResNet18 weights are required "
                        "(PRETRAINED_REQUIRED=true) but could not be acquired; "
                        "from_scratch_control is the only labelled ablation"
                    ) from exc
                self.provenance = provenance_record(
                    "resnet18",
                    cache_path=torchvision_cache_path("resnet18"),
                    torchvision_version=torchvision.__version__,
                )
            self.stem = nn.Sequential(
                resnet.conv1,
                resnet.bn1,
                resnet.relu,
            )
            self.pool = resnet.maxpool
            self.layer1 = resnet.layer1
            self.layer2 = resnet.layer2
            self.layer3 = resnet.layer3
            channels = base * 2
            self.lateral3 = nn.Conv2d(64, channels, 1)
            self.lateral4 = nn.Conv2d(128, channels, 1)
            self.lateral5 = nn.Conv2d(256, channels, 1)
            self.smooth3 = _ConvBnReLU(channels, channels, 3, 1, 1)
            self.smooth4 = _ConvBnReLU(channels, channels, 3, 1, 1)
            self.smooth5 = _ConvBnReLU(channels, channels, 3, 1, 1)

        def forward(self, image):
            x = self.stem(image)
            x = self.pool(x)
            c3 = self.layer1(x)
            c4 = self.layer2(c3)
            c5 = self.layer3(c4)
            p5 = self.lateral5(c5)
            p4 = self.lateral4(c4) + functional.interpolate(
                p5, size=c4.shape[-2:], mode="nearest"
            )
            p3 = self.lateral3(c3) + functional.interpolate(
                p4, size=c3.shape[-2:], mode="nearest"
            )
            return {
                "p3": self.smooth3(p3),
                "p4": self.smooth4(p4),
                "p5": self.smooth5(p5),
            }

    class DiscoveryDetector(_FlatOutputMixin, nn.Module):
        """Class-agnostic litter-candidate detector (objectness/offset/bbox)."""

        task = "discovery"
        model_id = "g4_discovery_detector_v1"
        input_shape = DISCOVERY_INPUT_SHAPE
        input_names = ("image_rgb",)
        output_names = ("objectness_logits", "offset", "bbox_size")
        output_channels = {"objectness_logits": 3, "offset": 6, "bbox_size": 6}

        def __init__(
            self,
            base: int = 48,
            from_scratch_control: bool = False,
            legacy_fpn_control: bool = False,
        ):
            super().__init__()
            if legacy_fpn_control and from_scratch_control:
                raise ValueError(
                    "legacy_fpn_control and from_scratch_control are "
                    "mutually exclusive diagnostic modes"
                )
            try:
                import torchvision  # noqa: F401

                torchvision_available = True
            except Exception:
                torchvision_available = False
            if not torchvision_available and not from_scratch_control:
                raise RuntimeError(
                    "torchvision is required for the production discovery "
                    "backbone (PRETRAINED_REQUIRED=true)"
                )
            self.backbone = (
                _FPNBackbone(base=base)
                if legacy_fpn_control
                else
                _DiscoveryResNetBackbone(
                    base=base,
                    from_scratch_control=from_scratch_control,
                )
                if torchvision_available
                else _FPNBackbone(base=base)
            )
            self.architecture_role = (
                "legacy_small_fpn_control"
                if legacy_fpn_control
                else "fcos_lite_resnet18_fpn_product_candidate"
            )
            self.head = nn.Sequential(
                _ConvBnReLU(base * 2, base * 2, 3, 1, 1)
            )
            self.objectness = nn.Conv2d(base * 2, 1, 3, padding=1)
            if legacy_fpn_control:
                self.offset = nn.Conv2d(base * 2, 2, 3, padding=1)
                self.bbox_size = nn.Sequential(
                    nn.Conv2d(base * 2, 2, 3, padding=1), nn.Softplus()
                )
            else:
                self.quality = nn.Conv2d(base * 2, 1, 3, padding=1)
                self.ltrb = nn.Sequential(
                    nn.Conv2d(base * 2, 4, 3, padding=1), nn.Softplus()
                )
                self.pyramid_levels = ("P3", "P4", "P5")
                self.pyramid_strides = DISCOVERY_STRIDES
                self.quality_head_enabled = True
                self.regression_parameterization = "ltrb"
                self.graph_external_postprocess = ("top_k", "nms")
            nn.init.constant_(self.objectness.bias, -3.0)

        def forward(self, image):
            features = self.backbone(image)
            if self.architecture_role == "legacy_small_fpn_control":
                head = self.head(features["p4"])
                return {
                    "objectness_logits": self.objectness(head),
                    "offset": self.offset(head),
                    "bbox_size": self.bbox_size(head),
                }
            target_size = features["p3"].shape[-2:]
            level_logits = []
            level_offsets = []
            level_sizes = []
            for name, stride in zip(("p3", "p4", "p5"), DISCOVERY_STRIDES):
                head = self.head(features[name])
                combined_logits = self.objectness(head) + self.quality(head)
                left, top, right, bottom = self.ltrb(head).unbind(dim=1)
                offset = torch.stack(
                    ((right - left) * 0.5, (bottom - top) * 0.5), dim=1
                )
                size = torch.stack((left + right, top + bottom), dim=1)
                level_logits.append(
                    functional.interpolate(
                        combined_logits, size=target_size, mode="nearest"
                    )
                )
                level_offsets.append(
                    functional.interpolate(offset, size=target_size, mode="nearest")
                )
                level_sizes.append(
                    functional.interpolate(size, size=target_size, mode="nearest")
                )
            return {
                "objectness_logits": torch.cat(level_logits, dim=1),
                "offset": torch.cat(level_offsets, dim=1),
                "bbox_size": torch.cat(level_sizes, dim=1),
            }

    class CandidateCropClassifier(_FlatOutputMixin, nn.Module):
        """Four-class classifier on an official MobileNetV3-small backbone."""

        task = "classifier"
        model_id = "g4_candidate_crop_classifier_v1"
        input_shape = CLASSIFIER_INPUT_SHAPE
        input_names = ("crop_rgb",)
        output_names = ("logits",)
        output_channels = {"logits": len(CLASSIFIER_CLASSES)}

        def __init__(
            self, base: int = 48, from_scratch_control: bool = False
        ):
            super().__init__()
            import torchvision

            self.from_scratch_control = bool(from_scratch_control)
            weights_enum = (
                torchvision.models.MobileNet_V3_Small_Weights.IMAGENET1K_V1
            )
            if from_scratch_control:
                mobilenet = torchvision.models.mobilenet_v3_small(weights=None)
                self.provenance = from_scratch_control_record(
                    "mobilenet_v3_small"
                )
            else:
                try:
                    mobilenet = torchvision.models.mobilenet_v3_small(
                        weights=weights_enum
                    )
                except Exception as exc:
                    raise RuntimeError(
                        "official MobileNetV3-small weights are required for "
                        "the crop classifier (PRETRAINED_REQUIRED=true)"
                    ) from exc
                self.provenance = provenance_record(
                    "mobilenet_v3_small",
                    cache_path=torchvision_cache_path("mobilenet_v3_small"),
                    torchvision_version=torchvision.__version__,
                )
            self.features = mobilenet.features
            self.pool = mobilenet.avgpool
            in_features = mobilenet.classifier[-1].in_features
            mobilenet.classifier[-1] = nn.Linear(
                in_features, len(CLASSIFIER_CLASSES)
            )
            self.head = mobilenet.classifier

        def forward(self, crop):
            features = self.pool(self.features(crop))
            return self.head(torch.flatten(features, 1))

    class _AreaEncoder(nn.Module):
        """RGB/geometry dual branch that preserves the pretrained RGB stem."""

        def __init__(
            self,
            in_channels: int = 10,
            base: int = 24,
            from_scratch_control: bool = False,
        ):
            super().__init__()
            self.base = base
            self.use_resnet = False
            try:
                import torchvision
            except Exception:
                torchvision = None
            if torchvision is not None:
                weights_enum = (
                    torchvision.models.ResNet18_Weights.IMAGENET1K_V1
                )
                if from_scratch_control:
                    resnet = torchvision.models.resnet18(weights=None)
                    self.provenance = from_scratch_control_record("resnet18")
                else:
                    try:
                        resnet = torchvision.models.resnet18(
                            weights=weights_enum
                        )
                    except Exception as exc:
                        raise RuntimeError(
                            "official pretrained ResNet18 weights are required "
                            "for the area encoder (PRETRAINED_REQUIRED=true)"
                        ) from exc
                    self.provenance = provenance_record(
                        "resnet18",
                        cache_path=torchvision_cache_path("resnet18"),
                        torchvision_version=torchvision.__version__,
                    )
                geometry_channels = in_channels - 3
                if geometry_channels <= 0:
                    raise ValueError(
                        "area input must contain RGB plus geometry channels"
                    )
                self.rgb_stem = nn.Sequential(
                    resnet.conv1,
                    resnet.bn1,
                    resnet.relu,
                )
                self.rgb_pool = resnet.maxpool
                self.rgb_enc1 = resnet.layer1
                self.rgb_enc2 = resnet.layer2
                self.rgb_enc3 = resnet.layer3
                self.rgb_enc4 = resnet.layer4
                self.geometry_stem = _ConvBnReLU(
                    geometry_channels, base, 3, 2, 1
                )
                self.geometry_pool = nn.MaxPool2d(3, 2, 1)
                self.geometry_enc1 = _ConvBnReLU(
                    base, base * 2, 3, 1, 1
                )
                self.geometry_enc2 = _ConvBnReLU(
                    base * 2, base * 4, 3, 2, 1
                )
                self.geometry_enc3 = _ConvBnReLU(
                    base * 4, base * 8, 3, 2, 1
                )
                self.geometry_enc4 = _ConvBnReLU(
                    base * 8, base * 8, 3, 2, 1
                )
                self.fuse_stem = _ConvBnReLU(64 + base, base, 1, 1, 0)
                self.fuse_enc1 = _ConvBnReLU(
                    64 + base * 2, base * 2, 1, 1, 0
                )
                self.fuse_enc2 = _ConvBnReLU(
                    128 + base * 4, base * 4, 1, 1, 0
                )
                self.fuse_enc3 = _ConvBnReLU(
                    256 + base * 8, base * 8, 1, 1, 0
                )
                self.fuse_bottleneck = _ConvBnReLU(
                    512 + base * 8, base * 8, 1, 1, 0
                )
                self.out_channels = base * 8
                self.use_resnet = True
            else:
                if not from_scratch_control:
                    raise RuntimeError(
                        "torchvision is required for the production area "
                        "encoder (PRETRAINED_REQUIRED=true)"
                    )
                self.stem = _ConvBnReLU(in_channels, base, 3, 2, 1)
                self.enc1 = _ConvBnReLU(base, base * 2, 3, 2, 1)
                self.enc2 = _ConvBnReLU(base * 2, base * 4, 3, 2, 1)
                self.enc3 = _ConvBnReLU(base * 4, base * 8, 3, 2, 1)
                self.enc4 = _ConvBnReLU(base * 8, base * 16, 3, 2, 1)
                self.bottleneck = nn.Sequential(
                    _ConvBnReLU(base * 16, base * 16, 3, 1, 1),
                    _ConvBnReLU(base * 16, base * 16, 3, 1, 1),
                )
                self.out_channels = base * 16

        def forward(self, rgbd):
            if self.use_resnet:
                if rgbd.shape[1] != 10:
                    raise ValueError(
                        f"area model requires 10 channels, got {rgbd.shape[1]}"
                    )
                rgb = rgbd[:, :3]
                geometry = rgbd[:, 3:]
                rgb_stem = self.rgb_stem(rgb)
                geometry_stem = self.geometry_stem(geometry)
                stem = self.fuse_stem(
                    torch.cat((rgb_stem, geometry_stem), dim=1)
                )
                rgb_enc1 = self.rgb_enc1(self.rgb_pool(rgb_stem))
                geometry_enc1 = self.geometry_enc1(
                    self.geometry_pool(geometry_stem)
                )
                enc1 = self.fuse_enc1(
                    torch.cat((rgb_enc1, geometry_enc1), dim=1)
                )
                rgb_enc2 = self.rgb_enc2(rgb_enc1)
                geometry_enc2 = self.geometry_enc2(geometry_enc1)
                enc2 = self.fuse_enc2(
                    torch.cat((rgb_enc2, geometry_enc2), dim=1)
                )
                rgb_enc3 = self.rgb_enc3(rgb_enc2)
                geometry_enc3 = self.geometry_enc3(geometry_enc2)
                enc3 = self.fuse_enc3(
                    torch.cat((rgb_enc3, geometry_enc3), dim=1)
                )
                rgb_enc4 = self.rgb_enc4(rgb_enc3)
                geometry_enc4 = self.geometry_enc4(geometry_enc3)
                bottleneck = self.fuse_bottleneck(
                    torch.cat((rgb_enc4, geometry_enc4), dim=1)
                )
                return {
                    "stem": stem,
                    "enc1": enc1,
                    "enc2": enc2,
                    "enc3": enc3,
                    "enc4": bottleneck,
                    "bottleneck": bottleneck,
                }
            stem = self.stem(rgbd)
            enc1 = self.enc1(stem)
            enc2 = self.enc2(enc1)
            enc3 = self.enc3(enc2)
            enc4 = self.enc4(enc3)
            return {
                "stem": stem,
                "enc1": enc1,
                "enc2": enc2,
                "enc3": enc3,
                "enc4": enc4,
                "bottleneck": self.bottleneck(enc4),
            }

    class _AreaDecoder(nn.Module):
        """Independent area decoder; consumes encoder features only."""

        def __init__(self, encoder_out_channels: int, base: int = 24):
            super().__init__()
            self.up3 = nn.Sequential(
                _ConvBnReLU(
                    encoder_out_channels + base * 8, base * 8, 3, 1, 1
                )
            )
            self.up2 = nn.Sequential(
                _ConvBnReLU(base * 8 + base * 4, base * 4, 3, 1, 1)
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
                size=features["enc3"].shape[-2:],
                mode="nearest",
            )
            x = self.up3(torch.cat((x, features["enc3"]), dim=1))
            x = functional.interpolate(
                x,
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

        def __init__(
            self,
            encoder=None,
            base: int = 24,
            from_scratch_control: bool = False,
        ):
            super().__init__()
            self.encoder = (
                encoder
                if encoder is not None
                else _AreaEncoder(
                    base=base, from_scratch_control=from_scratch_control
                )
            )
            self.decoder = _AreaDecoder(self.encoder.out_channels, base=base)
            self.boundary_head = _BoundaryHead(base)
            self.provenance = getattr(self.encoder, "provenance", None)

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

    class DeepLabAreaSegmenter(_FlatOutputMixin, nn.Module):
        """DeepLabV3-based independent area segmenter."""

        task: str | None = None
        model_id: str | None = None
        input_shape = SEGMENTER_INPUT_SHAPE
        input_names = ("rgbd",)
        output_names = ("logits", "boundary_logits")
        output_channels = {"logits": 1, "boundary_logits": 1}

        def __init__(self, from_scratch_control: bool = False):
            super().__init__()
            import torchvision

            weights_enum = (
                torchvision.models.segmentation
                .DeepLabV3_ResNet50_Weights
                .COCO_WITH_VOC_LABELS_V1
            )
            if from_scratch_control:
                self.deeplab = torchvision.models.segmentation.deeplabv3_resnet50(
                    weights=None
                )
                self.provenance = from_scratch_control_record(
                    "deeplabv3_resnet50"
                )
            else:
                try:
                    self.deeplab = (
                        torchvision.models.segmentation.deeplabv3_resnet50(
                            weights=weights_enum
                        )
                    )
                except Exception as exc:
                    raise RuntimeError(
                        "official DeepLabV3-ResNet50 pretrained weights are "
                        "required (PRETRAINED_REQUIRED=true)"
                    ) from exc
                self.provenance = provenance_record(
                    "deeplabv3_resnet50",
                    cache_path=torchvision_cache_path(
                        "deeplabv3_resnet50"
                    ),
                    torchvision_version=torchvision.__version__,
                )
            pretrained_conv = self.deeplab.backbone.conv1
            first_conv = nn.Conv2d(
                10,
                64,
                kernel_size=7,
                stride=2,
                padding=3,
                bias=False,
            )
            with torch.no_grad():
                first_conv.weight[:, :3] = pretrained_conv.weight
                nn.init.kaiming_normal_(
                    first_conv.weight[:, 3:],
                    mode="fan_out",
                    nonlinearity="relu",
                )
            self.deeplab.backbone.conv1 = first_conv
            self.deeplab.classifier[4] = nn.Conv2d(256, 1, 1)
            self.deeplab.aux_classifier = None
            self.boundary_head = _BoundaryHead(1)

        def forward(self, rgbd):
            logits = self.deeplab(rgbd)["out"]
            return {
                "logits": logits,
                "boundary_logits": self.boundary_head(logits),
            }

    class DeepLabLeafSegmenter(DeepLabAreaSegmenter):
        task = "leaf"
        model_id = "g4_leaf_segmenter_deeplab_v1"

    class DeepLabPuddleSegmenter(DeepLabAreaSegmenter):
        task = "puddle"
        model_id = "g4_puddle_segmenter_deeplab_v1"

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
            "DeepLabAreaSegmenter": DeepLabAreaSegmenter,
            "DeepLabLeafSegmenter": DeepLabLeafSegmenter,
            "DeepLabPuddleSegmenter": DeepLabPuddleSegmenter,
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


def build_g4_models(
    shared_encoder: bool = False, from_scratch_control: bool = False
) -> dict:
    """Build the four G4 models.

    ``shared_encoder`` shares only the area encoder.  Production candidates
    use official pretrained weights; ``from_scratch_control`` is an explicitly
    labelled ablation that can never produce product-ready status.
    """
    classes = _model_classes()
    try:
        import torchvision  # noqa: F401

        torchvision_available = True
    except Exception:
        torchvision_available = False
    models = {
        "discovery": classes["DiscoveryDetector"](
            from_scratch_control=from_scratch_control
        ),
        "classifier": classes["CandidateCropClassifier"](
            from_scratch_control=from_scratch_control
        ),
    }
    if shared_encoder:
        encoder = classes["_AreaEncoder"](
            from_scratch_control=from_scratch_control
        )
        models["leaf"] = classes["LeafSegmenter"](encoder=encoder)
        models["puddle"] = classes["PuddleSegmenter"](encoder=encoder)
    else:
        if not torchvision_available and not from_scratch_control:
            raise RuntimeError(
                "torchvision is required for production area segmenters "
                "(PRETRAINED_REQUIRED=true)"
            )
        models["leaf"] = classes["LeafSegmenter"](
            from_scratch_control=from_scratch_control
        )
        models["puddle"] = classes["PuddleSegmenter"](
            from_scratch_control=from_scratch_control
        )
    return models


def build_g4_model(
    task: str,
    *,
    from_scratch_control: bool = False,
    legacy_fpn_control: bool = False,
):
    """Build one task model without instantiating unrelated backbones."""
    classes = _model_classes()
    if task == "discovery":
        return classes["DiscoveryDetector"](
            from_scratch_control=from_scratch_control,
            legacy_fpn_control=legacy_fpn_control,
        )
    if legacy_fpn_control:
        raise ValueError("legacy_fpn_control is valid only for discovery")
    if task == "classifier":
        return classes["CandidateCropClassifier"](
            from_scratch_control=from_scratch_control
        )
    if task not in ("leaf", "puddle"):
        raise ValueError(f"unknown G4 model task {task!r}")
    class_name = "LeafSegmenter" if task == "leaf" else "PuddleSegmenter"
    try:
        import torchvision  # noqa: F401
    except Exception as exc:
        if not from_scratch_control:
            raise RuntimeError(
                "torchvision is required for production area segmenters "
                "(PRETRAINED_REQUIRED=true)"
            ) from exc
    return classes[class_name](
        from_scratch_control=from_scratch_control
    )


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
            "pretrained": getattr(model, "provenance", None),
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
    task = getattr(model, "task", None)
    task_parity = (
        task_specific_parity(task, torch_flat.numpy(), onnx_flat)
        if task is not None
        else {}
    )
    return {
        "max_absolute_error": max_error,
        "argmax_agreement": _argmax_agreement(torch_flat, onnx_flat),
        "decoded_agreement": (
            task_parity.get("decoded_agreement")
            if task == "discovery"
            else None
        ),
        **task_parity,
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
    "build_g4_model",
    "decode_discovery_flat",
    "export_fixed_onnx",
    "model_summary",
    "operator_inventory",
    "torch_onnx_parity",
]
