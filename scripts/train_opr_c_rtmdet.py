#!/usr/bin/env python3
"""Train the third and final bounded OPRV3 detector route."""

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
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--g6-root", type=Path, required=True)
    parser.add_argument("--prepared", type=Path, required=True)
    parser.add_argument("--upstream-checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--epochs", type=int, default=12)
    parser.add_argument("--batch-size", type=int, default=8)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    expected = "387a891e157cf0ab57d76b3ffc17bf77247089d672532427930b3140f9e789d6"
    if sha256(args.upstream_checkpoint) != expected:
        raise RuntimeError("official RTMDet-s checkpoint hash mismatch")
    patch_mmdet_cuda_nms()
    from mmengine.config import Config
    from mmengine.runner import Runner
    from mmdet.utils import register_all_modules

    register_all_modules(init_default_scope=True)
    base_config = Path(
        "/usr/local/lib/python3.12/dist-packages/mmdet/.mim/configs/rtmdet/rtmdet_s_8xb32-300e_coco.py"
    )
    cfg = Config.fromfile(base_config)
    cfg.work_dir = str(args.output)
    cfg.load_from = str(args.upstream_checkpoint)
    cfg.resume = False
    cfg.model.backbone.init_cfg = None
    cfg.model.bbox_head.num_classes = len(CLASS_NAMES)
    cfg.model.test_cfg.score_thr = 0.001
    cfg.train_dataloader.batch_size = args.batch_size
    cfg.train_dataloader.num_workers = 4
    cfg.train_dataloader.dataset.data_root = str(args.g6_root) + "/"
    cfg.train_dataloader.dataset.ann_file = str(args.prepared / "fit.json")
    cfg.train_dataloader.dataset.data_prefix = {"img": ""}
    cfg.train_dataloader.dataset.metainfo = {"classes": CLASS_NAMES}
    cfg.train_dataloader.dataset.filter_cfg = {"filter_empty_gt": False, "min_size": 32}
    cfg.val_dataloader.batch_size = args.batch_size
    cfg.val_dataloader.num_workers = 4
    cfg.val_dataloader.dataset.data_root = str(args.g6_root) + "/"
    cfg.val_dataloader.dataset.ann_file = str(args.prepared / "holdout.json")
    cfg.val_dataloader.dataset.data_prefix = {"img": ""}
    cfg.val_dataloader.dataset.metainfo = {"classes": CLASS_NAMES}
    cfg.test_dataloader = cfg.val_dataloader
    cfg.val_evaluator.ann_file = str(args.prepared / "holdout.json")
    cfg.test_evaluator = cfg.val_evaluator
    cfg.train_cfg = {"type": "EpochBasedTrainLoop", "max_epochs": args.epochs, "val_interval": 1}
    cfg.optim_wrapper.type = "AmpOptimWrapper"
    cfg.optim_wrapper.loss_scale = "dynamic"
    cfg.optim_wrapper.optimizer.lr = 0.0005
    cfg.optim_wrapper.clip_grad = {"max_norm": 10.0, "norm_type": 2}
    cfg.param_scheduler = [
        {"type": "LinearLR", "start_factor": 0.01, "by_epoch": False, "begin": 0, "end": 100},
    ]
    if args.epochs > 1:
        cfg.param_scheduler.append({
            "type": "CosineAnnealingLR",
            "eta_min": 0.00005,
            "begin": 1,
            "end": args.epochs,
            "T_max": max(args.epochs - 1, 1),
            "by_epoch": True,
            "convert_to_iter_based": True,
        })
    cfg.default_hooks.logger.interval = 25
    cfg.default_hooks.checkpoint.interval = 1
    cfg.default_hooks.checkpoint.max_keep_ckpts = 3
    cfg.default_hooks.checkpoint.save_best = "coco/bbox_mAP"
    cfg.default_hooks.checkpoint.rule = "greater"
    cfg.custom_hooks = []
    cfg.randomness = {"seed": 20260813, "deterministic": True}
    cfg.env_cfg.cudnn_benchmark = True
    config_path = args.output / "opr_c_rtmdet_s_config.py"
    cfg.dump(config_path)
    started = time.perf_counter()
    runner = Runner.from_cfg(cfg)
    runner.train()
    report = {
        "schema_version": 1,
        "stage": "OPRV3-05-OPR-C-TRAIN",
        "architecture": "official_mmdetection_v3.3.0_rtmdet_s",
        "epochs": args.epochs,
        "batch_size": args.batch_size,
        "duration_s": time.perf_counter() - started,
        "official_checkpoint_sha256": expected,
        "config_sha256": sha256(config_path),
        "G5_SEALED_FINAL_read": False,
    }
    (args.output / "OPR_C_TRAIN_REPORT.json").write_text(json.dumps(report, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
