#!/usr/bin/env python3
"""Bounded official MMDetection DINO R50 4-scale fine-tune for TGARV9 T2."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))
from sanitation_learning.opr_c_rtmdet import CLASS_NAMES, patch_mmdet_cuda_nms  # noqa: E402


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--train-coco", type=Path, required=True)
    parser.add_argument("--holdout-coco", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--expected-sha256", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--container-digest", required=True)
    parser.add_argument("--epochs", type=int, default=6)
    parser.add_argument("--batch-size", type=int, default=2)
    parser.add_argument("--train-image-root", default="/")
    parser.add_argument("--holdout-image-root", default="/")
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    if sha256(args.checkpoint) != args.expected_sha256:
        raise RuntimeError("official DINO checkpoint SHA mismatch")
    train_payload = json.loads(args.train_coco.read_text())
    negative_count = sum(bool(row.get("negative_only")) for row in train_payload["images"])
    if negative_count == 0:
        raise RuntimeError("T2 requires negative-only frames and cannot filter them")
    args.output.mkdir(parents=True)
    patch_mmdet_cuda_nms()
    # This audited image has CUDA-enabled PyTorch but its MMCV wheel lacks the
    # deformable-attention CUDA extension.  Force MMCV's official differentiable
    # PyTorch fallback; it preserves DINO semantics and is slower, so the later
    # deployability pre-screen remains mandatory.
    import mmcv.ops.multi_scale_deform_attn as deform_attn
    deform_attn.IS_CUDA_AVAILABLE = False
    # Same wheel limitation for MMCV focal loss: route CUDA logits through the
    # equivalent all-PyTorch implementation with an explicit one-hot target.
    import torch.nn.functional as torch_functional
    import mmdet.models.losses.focal_loss as focal_loss
    def torch_sigmoid_focal_loss(pred, target, weight=None, gamma=2.0, alpha=0.25, reduction="mean", avg_factor=None):
        if pred.dim() != target.dim():
            target = torch_functional.one_hot(target, num_classes=pred.size(1) + 1)[:, : pred.size(1)]
        return focal_loss.py_sigmoid_focal_loss(pred, target, weight, gamma, alpha, reduction, avg_factor)
    focal_loss.sigmoid_focal_loss = torch_sigmoid_focal_loss
    from mmengine.config import Config
    from mmengine.runner import Runner
    from mmdet.utils import register_all_modules
    register_all_modules(init_default_scope=True)
    config_path = "/usr/local/lib/python3.12/dist-packages/mmdet/.mim/configs/dino/dino-4scale_r50_improved_8xb2-12e_coco.py"
    cfg = Config.fromfile(config_path)
    cfg.work_dir = str(args.output); cfg.load_from = str(args.checkpoint); cfg.resume = False
    # The full official DINO checkpoint supplies the backbone.  Disable the
    # config's separate torchvision initialization to keep the run offline and
    # hash-bound to the one audited weight file.
    cfg.model.backbone.init_cfg = None
    cfg.model.bbox_head.num_classes = 3
    cfg.model.dn_cfg.group_cfg.num_dn_queries = 100
    cfg.model.test_cfg.max_per_img = 100
    cfg.model.test_cfg.score_thr = 0.001
    train_pipeline = [
        {"type": "LoadImageFromFile", "backend_args": None},
        {"type": "LoadAnnotations", "with_bbox": True},
        {"type": "RandomFlip", "prob": 0.5},
        {"type": "RandomChoiceResize", "scales": [(480, 360), (560, 420), (640, 480), (720, 540)], "keep_ratio": False},
        {"type": "PackDetInputs"},
    ]
    eval_pipeline = [
        {"type": "LoadImageFromFile", "backend_args": None},
        {"type": "Resize", "scale": (640, 480), "keep_ratio": False},
        {"type": "LoadAnnotations", "with_bbox": True},
        {"type": "PackDetInputs", "meta_keys": ("img_id", "img_path", "ori_shape", "img_shape", "scale_factor")},
    ]
    for loader, coco, pipeline, image_root in ((cfg.train_dataloader, args.train_coco, train_pipeline, args.train_image_root), (cfg.val_dataloader, args.holdout_coco, eval_pipeline, args.holdout_image_root)):
        loader.batch_size = args.batch_size; loader.num_workers = 2; loader.persistent_workers = True
        loader.dataset.data_root = image_root; loader.dataset.ann_file = str(coco); loader.dataset.data_prefix = {"img": ""}
        loader.dataset.metainfo = {"classes": CLASS_NAMES}; loader.dataset.filter_cfg = {"filter_empty_gt": False, "min_size": 1}; loader.dataset.pipeline = pipeline
    cfg.test_dataloader = cfg.val_dataloader
    cfg.val_evaluator.ann_file = str(args.holdout_coco); cfg.test_evaluator = cfg.val_evaluator
    cfg.train_cfg = {"type": "EpochBasedTrainLoop", "max_epochs": args.epochs, "val_interval": 1}
    cfg.optim_wrapper.type = "AmpOptimWrapper"; cfg.optim_wrapper.loss_scale = "dynamic"
    cfg.optim_wrapper.optimizer.lr = 1e-5; cfg.optim_wrapper.clip_grad = {"max_norm": 0.1, "norm_type": 2}
    cfg.param_scheduler = [{"type": "LinearLR", "start_factor": 0.1, "by_epoch": False, "begin": 0, "end": 100}, {"type": "CosineAnnealingLR", "eta_min": 1e-6, "begin": 0, "end": args.epochs, "T_max": args.epochs, "by_epoch": True}]
    cfg.default_hooks.logger.interval = 25; cfg.default_hooks.checkpoint.interval = 1; cfg.default_hooks.checkpoint.max_keep_ckpts = args.epochs; cfg.default_hooks.checkpoint.save_best = "coco/bbox_mAP"; cfg.default_hooks.checkpoint.rule = "greater"
    # Deformable-DETR positional encoding calls CUDA cumsum, which PyTorch
    # does not implement under strict deterministic-algorithm mode.  The seed
    # stays frozen and every checkpoint/config is hash-bound; this flag is the
    # documented runtime limitation rather than an untracked random search.
    cfg.randomness = {"seed": 20260813, "deterministic": False}; cfg.env_cfg.cudnn_benchmark = False
    config = args.output / "tgarv9_t2_dino_r50_4scale.py"; cfg.dump(config)
    started = time.perf_counter(); Runner.from_cfg(cfg).train()
    checkpoints = sorted(args.output.glob("epoch_*.pth"))
    report = {"schema_version": 1, "protocol": "TGARV9", "stage": "T2_DINO_TRAIN", "architecture": "official_mmdetection_dino_r50_4scale_improved", "official_checkpoint_sha256": args.expected_sha256, "config_sha256": sha256(config), "container_digest": args.container_digest, "train_image_count": len(train_payload["images"]), "negative_frame_count": negative_count, "negative_frames_retained": True, "epochs": args.epochs, "epoch_checkpoint_count": len(checkpoints), "duration_s": time.perf_counter() - started, "VAL_NEW_read": False, "G5_V2_read": False}
    (args.output / "T2_TRAIN_REPORT.json").write_text(json.dumps(report, indent=2) + "\n")
    return 0 if len(checkpoints) == args.epochs else 2


if __name__ == "__main__":
    raise SystemExit(main())
