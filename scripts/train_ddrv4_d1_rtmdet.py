#!/usr/bin/env python3
"""Train DDRV4-D1 A/B with one identical G7-only RTMDet-s protocol."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))

from sanitation_learning.ddrv4_boundary import G7_DATASET_ID, require_ddrv4_selection_inputs  # noqa: E402
from sanitation_learning.opr_c_rtmdet import CLASS_NAMES, patch_mmdet_cuda_nms  # noqa: E402


INITIALIZATIONS = {
    "D1-A": "387a891e157cf0ab57d76b3ffc17bf77247089d672532427930b3140f9e789d6",
    "D1-B": "833e6148f566aed60c27378c4c1f832bb0e3f7532dae780d12ce5424579e2dfa",
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--route", choices=tuple(INITIALIZATIONS), required=True)
    parser.add_argument("--g7-root", required=True, type=Path)
    parser.add_argument("--prepared", required=True, type=Path)
    parser.add_argument("--initial-checkpoint", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--epochs", type=int, default=6)
    parser.add_argument("--batch-size", type=int, default=8)
    args = parser.parse_args()
    require_ddrv4_selection_inputs([G7_DATASET_ID])
    if args.output.exists():
        raise FileExistsError(f"D1 training output exists: {args.output}")
    if sha256(args.initial_checkpoint) != INITIALIZATIONS[args.route]:
        raise RuntimeError(f"{args.route} initialization checkpoint SHA-256 mismatch")
    prep = json.loads((args.prepared / "D1_PREP_REPORT.json").read_text(encoding="utf-8"))
    if prep.get("dataset_id") != G7_DATASET_ID or prep.get("untouched_val_used_for_selection") is not False or prep.get("G6_used_for_selection") is not False:
        raise RuntimeError("D1 prepared data boundary is invalid")
    args.output.mkdir(parents=True, exist_ok=False)
    patch_mmdet_cuda_nms()
    from mmengine.config import Config
    from mmengine.runner import Runner
    from mmdet.utils import register_all_modules

    register_all_modules(init_default_scope=True)
    base_config = Path("/usr/local/lib/python3.12/dist-packages/mmdet/.mim/configs/rtmdet/rtmdet_s_8xb32-300e_coco.py")
    cfg = Config.fromfile(base_config)
    cfg.work_dir = str(args.output)
    cfg.load_from = str(args.initial_checkpoint)
    cfg.resume = False
    cfg.model.backbone.init_cfg = None
    cfg.model.bbox_head.num_classes = len(CLASS_NAMES)
    cfg.model.test_cfg.score_thr = 0.001
    safe_train_pipeline = [
        {"type": "LoadImageFromFile", "backend_args": None},
        {"type": "LoadAnnotations", "with_bbox": True},
        {"type": "Resize", "scale": (640, 480), "keep_ratio": False},
        {"type": "PackDetInputs"},
    ]
    safe_eval_pipeline = [
        {"type": "LoadImageFromFile", "backend_args": None},
        {"type": "Resize", "scale": (640, 480), "keep_ratio": False},
        {"type": "LoadAnnotations", "with_bbox": True},
        {"type": "PackDetInputs", "meta_keys": ("img_id", "img_path", "ori_shape", "img_shape", "scale_factor")},
    ]
    cfg.train_dataloader.batch_size = args.batch_size
    cfg.train_dataloader.num_workers = 4
    cfg.train_dataloader.dataset.data_root = str(args.g7_root) + "/"
    cfg.train_dataloader.dataset.ann_file = str(args.prepared / "fit.json")
    cfg.train_dataloader.dataset.data_prefix = {"img": ""}
    cfg.train_dataloader.dataset.metainfo = {"classes": CLASS_NAMES}
    cfg.train_dataloader.dataset.filter_cfg = {"filter_empty_gt": False, "min_size": 1}
    cfg.train_dataloader.dataset.pipeline = safe_train_pipeline
    cfg.val_dataloader.batch_size = args.batch_size
    cfg.val_dataloader.num_workers = 4
    cfg.val_dataloader.dataset.data_root = str(args.g7_root) + "/"
    cfg.val_dataloader.dataset.ann_file = str(args.prepared / "holdout.json")
    cfg.val_dataloader.dataset.data_prefix = {"img": ""}
    cfg.val_dataloader.dataset.metainfo = {"classes": CLASS_NAMES}
    cfg.val_dataloader.dataset.pipeline = safe_eval_pipeline
    cfg.test_dataloader = cfg.val_dataloader
    cfg.val_evaluator.ann_file = str(args.prepared / "holdout.json")
    cfg.test_evaluator = cfg.val_evaluator
    cfg.train_cfg = {"type": "EpochBasedTrainLoop", "max_epochs": args.epochs, "val_interval": 1}
    cfg.optim_wrapper.type = "AmpOptimWrapper"
    cfg.optim_wrapper.loss_scale = "dynamic"
    cfg.optim_wrapper.optimizer.lr = 0.0005
    cfg.optim_wrapper.clip_grad = {"max_norm": 10.0, "norm_type": 2}
    cfg.param_scheduler = [{"type": "LinearLR", "start_factor": 0.01, "by_epoch": False, "begin": 0, "end": 100}]
    if args.epochs > 1:
        cfg.param_scheduler.append({"type": "CosineAnnealingLR", "eta_min": 0.00005, "begin": 1, "end": args.epochs, "T_max": args.epochs - 1, "by_epoch": True, "convert_to_iter_based": True})
    cfg.default_hooks.logger.interval = 25
    cfg.default_hooks.checkpoint.interval = 1
    cfg.default_hooks.checkpoint.max_keep_ckpts = 3
    cfg.default_hooks.checkpoint.save_best = "coco/bbox_mAP"
    cfg.default_hooks.checkpoint.rule = "greater"
    cfg.custom_hooks = []
    cfg.randomness = {"seed": 20260814, "deterministic": True}
    cfg.env_cfg.cudnn_benchmark = True
    config_path = args.output / "ddrv4_d1_rtmdet_s_config.py"
    cfg.dump(config_path)
    started = time.perf_counter()
    Runner.from_cfg(cfg).train()
    best = sorted(args.output.glob("best_coco_bbox_mAP_epoch_*.pth"))
    report = {
        "schema_version": 1, "stage": "DDRV4-03-TRAIN", "route": args.route,
        "architecture": "official_mmdetection_v3.3.0_rtmdet_s",
        "initial_checkpoint_sha256": INITIALIZATIONS[args.route],
        "epochs": args.epochs, "batch_size": args.batch_size,
        "duration_s": time.perf_counter() - started,
        "training_dataset": G7_DATASET_ID, "selection_dataset": "G7_IN_DOMAIN_HOLDOUT",
        "G7_VAL_used": False, "G6_used": False, "G5_read": False, "G5_V2_read": False,
        "augmentation": {"native_G7_domain_variation": True, "resize_only": [640, 480], "arbitrary_stylization": False, "label_desynchronizing_transform": False},
        "config_sha256": sha256(config_path),
        "best_checkpoint_count": len(best),
        "best_checkpoint_sha256": sha256(best[0]) if len(best) == 1 else None,
    }
    (args.output / "D1_TRAIN_REPORT.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0 if len(best) == 1 else 4


if __name__ == "__main__":
    raise SystemExit(main())
