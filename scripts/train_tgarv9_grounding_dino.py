#!/usr/bin/env python3
"""Bounded official Grounding-DINO Swin-T closed-set fine-tune for T3."""

from __future__ import annotations

import argparse
import hashlib
import json
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
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--expected-sha256", required=True)
    parser.add_argument("--bert", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--container-digest", required=True)
    parser.add_argument("--epochs", type=int, default=6)
    parser.add_argument("--batch-size", type=int, default=2)
    parser.add_argument("--train-image-root", default="/")
    parser.add_argument("--preflight-iters", type=int, default=0)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    if sha256(args.checkpoint) != args.expected_sha256.lower():
        raise RuntimeError("official Grounding-DINO checkpoint SHA mismatch")
    for required in ("config.json", "model.safetensors", "tokenizer.json", "vocab.txt"):
        if not (args.bert / required).is_file():
            raise RuntimeError(f"local BERT asset missing: {required}")
    train_payload = json.loads(args.train_coco.read_text())
    negative_count = sum(bool(row.get("negative_only")) for row in train_payload["images"])
    if negative_count == 0:
        raise RuntimeError("T3 requires negative-only frames and cannot filter them")
    args.output.mkdir(parents=True)

    patch_mmdet_cuda_nms()
    import mmcv.ops.multi_scale_deform_attn as deform_attn
    deform_attn.IS_CUDA_AVAILABLE = False
    import torch.nn.functional as torch_functional
    import mmdet.models.losses.focal_loss as focal_loss

    def torch_sigmoid_focal_loss(pred, target, weight=None, gamma=2.0, alpha=0.25, reduction="mean", avg_factor=None):
        if pred.dim() != target.dim():
            target = torch_functional.one_hot(target, num_classes=pred.size(1) + 1)[:, : pred.size(1)]
        return focal_loss.py_sigmoid_focal_loss(pred, target, weight, gamma, alpha, reduction, avg_factor)

    focal_loss.sigmoid_focal_loss = torch_sigmoid_focal_loss
    # Transformers 4.44 automatically selects SDPA, whose helper only accepts
    # a 2-D attention mask. MMDetection 3.3.0 Grounding-DINO intentionally
    # supplies a 3-D sub-sentence mask. Force the upstream eager BERT path,
    # which supports that documented interface, without editing site-packages.
    from transformers import BertConfig
    original_bert_config_from_pretrained = BertConfig.from_pretrained

    def eager_bert_config_from_pretrained(*config_args, **config_kwargs):
        config = original_bert_config_from_pretrained(*config_args, **config_kwargs)
        config._attn_implementation = "eager"
        return config

    BertConfig.from_pretrained = eager_bert_config_from_pretrained
    from mmengine.config import Config
    from mmengine.runner import Runner
    from mmdet.utils import register_all_modules
    register_all_modules(init_default_scope=True)

    config_path = "/usr/local/lib/python3.12/dist-packages/mmdet/.mim/configs/grounding_dino/grounding_dino_swin-t_finetune_8xb2_20e_cat.py"
    cfg = Config.fromfile(config_path)
    cfg.work_dir = str(args.output)
    cfg.load_from = str(args.checkpoint)
    cfg.resume = False
    cfg.model.language_model.name = str(args.bert)
    cfg.model.bbox_head.num_classes = len(CLASS_NAMES)
    cfg.model.test_cfg.max_per_img = 100
    train_pipeline = [
        {"type": "LoadImageFromFile", "backend_args": None},
        {"type": "LoadAnnotations", "with_bbox": True},
        {"type": "RandomFlip", "prob": 0.5},
        {"type": "RandomChoiceResize", "scales": [(480, 360), (560, 420), (640, 480), (720, 540)], "keep_ratio": False},
        {"type": "PackDetInputs", "meta_keys": ("img_id", "img_path", "ori_shape", "img_shape", "scale_factor", "flip", "flip_direction", "text", "custom_entities")},
    ]
    loader = cfg.train_dataloader
    loader.batch_size = args.batch_size
    loader.num_workers = 2
    loader.persistent_workers = True
    loader.dataset.data_root = args.train_image_root
    loader.dataset.ann_file = str(args.train_coco)
    loader.dataset.data_prefix = {"img": ""}
    loader.dataset.metainfo = {"classes": CLASS_NAMES}
    loader.dataset.return_classes = True
    loader.dataset.filter_cfg = {"filter_empty_gt": False, "min_size": 1}
    loader.dataset.pipeline = train_pipeline
    cfg.val_cfg = None
    cfg.val_dataloader = None
    cfg.val_evaluator = None
    cfg.test_cfg = None
    cfg.test_dataloader = None
    cfg.test_evaluator = None
    if args.preflight_iters:
        cfg.train_cfg = {"type": "IterBasedTrainLoop", "max_iters": args.preflight_iters, "val_interval": args.preflight_iters + 1}
        cfg.default_hooks.checkpoint.by_epoch = False
        cfg.default_hooks.checkpoint.interval = args.preflight_iters
        cfg.default_hooks.checkpoint.max_keep_ckpts = 1
        cfg.param_scheduler = [{"type": "LinearLR", "start_factor": 0.1, "by_epoch": False, "begin": 0, "end": args.preflight_iters}]
    else:
        cfg.train_cfg = {"type": "EpochBasedTrainLoop", "max_epochs": args.epochs, "val_interval": args.epochs + 1}
        cfg.default_hooks.checkpoint.by_epoch = True
        cfg.default_hooks.checkpoint.interval = 1
        cfg.default_hooks.checkpoint.max_keep_ckpts = args.epochs
        cfg.param_scheduler = [
            {"type": "LinearLR", "start_factor": 0.1, "by_epoch": False, "begin": 0, "end": 100},
            {"type": "CosineAnnealingLR", "eta_min": 5e-6, "begin": 0, "end": args.epochs, "T_max": args.epochs, "by_epoch": True},
        ]
    cfg.default_hooks.checkpoint.save_best = None
    cfg.default_hooks.logger.interval = 1 if args.preflight_iters else 25
    cfg.optim_wrapper.type = "AmpOptimWrapper"
    cfg.optim_wrapper.loss_scale = "dynamic"
    cfg.optim_wrapper.optimizer.lr = 2e-5
    cfg.optim_wrapper.clip_grad = {"max_norm": 0.1, "norm_type": 2}
    cfg.randomness = {"seed": 20260813, "deterministic": False}
    cfg.env_cfg.cudnn_benchmark = False
    config = args.output / "tgarv9_t3_grounding_dino_swin_t.py"
    cfg.dump(config)
    started = time.perf_counter()
    Runner.from_cfg(cfg).train()
    checkpoints = sorted(args.output.glob("iter_*.pth" if args.preflight_iters else "epoch_*.pth"))
    expected_count = 1 if args.preflight_iters else args.epochs
    report = {
        "schema_version": 1,
        "protocol": "TGARV9",
        "stage": "T3_GROUNDING_DINO_PREFLIGHT" if args.preflight_iters else "T3_GROUNDING_DINO_TRAIN",
        "architecture": "official_mmdetection_grounding_dino_swin_t_closed_set",
        "official_checkpoint_sha256": args.expected_sha256.lower(),
        "bert_asset_sha256": {path.name: sha256(path) for path in sorted(args.bert.iterdir()) if path.is_file()},
        "config_sha256": sha256(config),
        "container_digest": args.container_digest,
        "bert_attention_backend": "eager_for_mmdet_3d_subsentence_mask",
        "train_image_count": len(train_payload["images"]),
        "negative_frame_count": negative_count,
        "negative_frames_retained": True,
        "batch_size": args.batch_size,
        "epochs": 0 if args.preflight_iters else args.epochs,
        "preflight_iters": args.preflight_iters,
        "checkpoint_count": len(checkpoints),
        "duration_s": time.perf_counter() - started,
        "VAL_NEW_read": False,
        "G5_V2_read": False,
    }
    (args.output / ("T3_PREFLIGHT_REPORT.json" if args.preflight_iters else "T3_TRAIN_REPORT.json")).write_text(json.dumps(report, indent=2) + "\n")
    return 0 if len(checkpoints) == expected_count else 2


if __name__ == "__main__":
    raise SystemExit(main())
