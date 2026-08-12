#!/usr/bin/env python3
"""Run the single bounded GA1 fine-tune from MA1 on real-Gazebo TRAIN."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
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


def repository_commit() -> str:
    injected = os.environ.get("TZCUP_SOURCE_COMMIT", "").strip()
    if injected:
        if not re.fullmatch(r"[0-9a-fA-F]{40}", injected):
            raise RuntimeError("TZCUP_SOURCE_COMMIT must be a full git SHA")
        return injected.lower()
    return subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
    ).strip()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--prepared", type=Path, required=True)
    parser.add_argument("--initial-checkpoint", type=Path, required=True)
    parser.add_argument("--expected-initial-sha256", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--container-digest", required=True)
    parser.add_argument("--epochs", type=int, default=6)
    parser.add_argument("--batch-size", type=int, default=8)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    if sha256(args.initial_checkpoint) != args.expected_initial_sha256:
        raise RuntimeError("GA1 MA1 warm-start SHA-256 mismatch")
    prep = json.loads(
        (args.prepared / "GOCV7_GA1_DATA_PREP.json").read_text(encoding="utf-8")
    )
    if prep.get("GA1_PREP_PASS") is not True:
        raise RuntimeError("GA1 data preparation did not pass")
    if prep.get("G5_read") is not False or prep.get("G5_V2_read") is not False:
        raise RuntimeError("GA1 data violated sealed-final boundaries")
    if prep["splits"]["GA1_TRAIN"]["world_ids"] == prep["splits"]["GA1_HOLDOUT"]["world_ids"]:
        raise RuntimeError("GA1 TRAIN and HOLDOUT are not world isolated")

    args.output.mkdir(parents=True)
    patch_mmdet_cuda_nms()
    from mmengine.config import Config
    from mmengine.runner import Runner
    from mmdet.utils import register_all_modules

    register_all_modules(init_default_scope=True)
    cfg = Config.fromfile(
        "/usr/local/lib/python3.12/dist-packages/mmdet/.mim/configs/rtmdet/rtmdet_s_8xb32-300e_coco.py"
    )
    cfg.work_dir = str(args.output)
    cfg.load_from = str(args.initial_checkpoint)
    cfg.resume = False
    cfg.model.backbone.init_cfg = None
    cfg.model.bbox_head.num_classes = len(CLASS_NAMES)
    cfg.model.test_cfg.score_thr = 0.001
    train_pipeline = [
        {"type": "LoadImageFromFile", "backend_args": None},
        {"type": "LoadAnnotations", "with_bbox": True},
        {"type": "Resize", "scale": (640, 480), "keep_ratio": False},
        {"type": "RandomFlip", "prob": 0.5},
        {"type": "PackDetInputs"},
    ]
    eval_pipeline = [
        {"type": "LoadImageFromFile", "backend_args": None},
        {"type": "Resize", "scale": (640, 480), "keep_ratio": False},
        {"type": "LoadAnnotations", "with_bbox": True},
        {
            "type": "PackDetInputs",
            "meta_keys": (
                "img_id",
                "img_path",
                "ori_shape",
                "img_shape",
                "scale_factor",
            ),
        },
    ]
    for loader, annotation, pipeline in (
        (cfg.train_dataloader, "train.json", train_pipeline),
        (cfg.val_dataloader, "holdout.json", eval_pipeline),
    ):
        loader.batch_size = args.batch_size
        loader.num_workers = 4
        loader.dataset.data_root = str(args.data_root) + "/"
        loader.dataset.ann_file = str(args.prepared / annotation)
        loader.dataset.data_prefix = {"img": ""}
        loader.dataset.metainfo = {"classes": CLASS_NAMES}
        loader.dataset.filter_cfg = {"filter_empty_gt": False, "min_size": 1}
        loader.dataset.pipeline = pipeline
    cfg.test_dataloader = cfg.val_dataloader
    cfg.val_evaluator.ann_file = str(args.prepared / "holdout.json")
    cfg.test_evaluator = cfg.val_evaluator
    cfg.train_cfg = {
        "type": "EpochBasedTrainLoop",
        "max_epochs": args.epochs,
        "val_interval": 1,
    }
    cfg.optim_wrapper.type = "AmpOptimWrapper"
    cfg.optim_wrapper.loss_scale = "dynamic"
    cfg.optim_wrapper.optimizer.lr = 0.00005
    cfg.optim_wrapper.clip_grad = {"max_norm": 10.0, "norm_type": 2}
    cfg.param_scheduler = [
        {"type": "LinearLR", "start_factor": 0.1, "by_epoch": False, "begin": 0, "end": 50},
        {
            "type": "CosineAnnealingLR",
            "eta_min": 0.000005,
            "begin": 1,
            "end": args.epochs,
            "T_max": args.epochs - 1,
            "by_epoch": True,
            "convert_to_iter_based": True,
        },
    ]
    cfg.default_hooks.logger.interval = 20
    cfg.default_hooks.checkpoint.interval = 1
    cfg.default_hooks.checkpoint.max_keep_ckpts = 2
    cfg.default_hooks.checkpoint.save_best = "coco/bbox_mAP"
    cfg.default_hooks.checkpoint.rule = "greater"
    cfg.custom_hooks = []
    cfg.randomness = {"seed": 20260812, "deterministic": True}
    cfg.env_cfg.cudnn_benchmark = True
    config_path = args.output / "gocv7_ga1_rtmdet_s_config.py"
    cfg.dump(config_path)
    started = time.perf_counter()
    Runner.from_cfg(cfg).train()
    best = sorted(args.output.glob("best_coco_bbox_mAP_epoch_*.pth"))
    report = {
        "schema_version": 1,
        "protocol": "GAZEBO-ONLINE-CLOSURE-V7",
        "stage": "GOCV7-01-GA1-TRAIN",
        "repository_commit": repository_commit(),
        "container_digest": args.container_digest,
        "route": "GA1",
        "architecture": "official_mmdetection_v3.3.0_rtmdet_s",
        "initial_route": "MA1",
        "initial_checkpoint_sha256": args.expected_initial_sha256,
        "epochs": args.epochs,
        "batch_size": args.batch_size,
        "optimizer_lr": 0.00005,
        "training_dataset": "GA1_TRAIN_ONLY",
        "selection_dataset": "GA1_HOLDOUT_ONLY",
        "GA1_HOLDOUT_used_by_training_loop_for_checkpoint_selection": True,
        "existing_24_mission_used": False,
        "G5_read": False,
        "G5_V2_read": False,
        "formal_30seed_read": False,
        "config_sha256": sha256(config_path),
        "best_checkpoint_count": len(best),
        "best_checkpoint_sha256": sha256(best[0]) if len(best) == 1 else None,
        "duration_s": time.perf_counter() - started,
    }
    (args.output / "GOCV7_GA1_TRAIN_REPORT.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    return 0 if len(best) == 1 else 4


if __name__ == "__main__":
    raise SystemExit(main())
