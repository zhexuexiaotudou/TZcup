#!/usr/bin/env python3
"""Bounded MA1 RTMDet-s fine-tune on MOVING TRAIN with HOLDOUT-only selection."""

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
        for chunk in iter(lambda: stream.read(1024*1024), b""): digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--route", choices=["MA1"], required=True)
    parser.add_argument("--data-root", required=True, type=Path); parser.add_argument("--prepared", required=True, type=Path)
    parser.add_argument("--initial-checkpoint", required=True, type=Path); parser.add_argument("--expected-initial-sha256", required=True)
    parser.add_argument("--output", required=True, type=Path); parser.add_argument("--epochs", type=int, default=6); parser.add_argument("--batch-size", type=int, default=8)
    args = parser.parse_args()
    if args.output.exists(): raise FileExistsError(args.output)
    if sha256(args.initial_checkpoint) != args.expected_initial_sha256: raise RuntimeError("MA1 warm-start hash mismatch")
    prep = json.loads((args.prepared / "CRV6_MOVING_ADAPTATION_PREP.json").read_text(encoding="utf-8"))
    if prep["MOVING_VAL_read"] is not False: raise RuntimeError("MOVING VAL leakage")
    args.output.mkdir(parents=True); patch_mmdet_cuda_nms()
    from mmengine.config import Config
    from mmengine.runner import Runner
    from mmdet.utils import register_all_modules
    register_all_modules(init_default_scope=True)
    cfg = Config.fromfile("/usr/local/lib/python3.12/dist-packages/mmdet/.mim/configs/rtmdet/rtmdet_s_8xb32-300e_coco.py")
    cfg.work_dir = str(args.output); cfg.load_from = str(args.initial_checkpoint); cfg.resume = False
    cfg.model.backbone.init_cfg = None; cfg.model.bbox_head.num_classes = len(CLASS_NAMES); cfg.model.test_cfg.score_thr = 0.001
    train_pipeline = [{"type":"LoadImageFromFile","backend_args":None},{"type":"LoadAnnotations","with_bbox":True},{"type":"Resize","scale":(640,480),"keep_ratio":False},{"type":"RandomFlip","prob":0.5},{"type":"PackDetInputs"}]
    eval_pipeline = [{"type":"LoadImageFromFile","backend_args":None},{"type":"Resize","scale":(640,480),"keep_ratio":False},{"type":"LoadAnnotations","with_bbox":True},{"type":"PackDetInputs","meta_keys":("img_id","img_path","ori_shape","img_shape","scale_factor")}]
    for loader, ann, pipeline in ((cfg.train_dataloader,"train.json",train_pipeline),(cfg.val_dataloader,"holdout.json",eval_pipeline)):
        loader.batch_size=args.batch_size; loader.num_workers=4; loader.dataset.data_root=str(args.data_root)+"/"; loader.dataset.ann_file=str(args.prepared/ann); loader.dataset.data_prefix={"img":""}; loader.dataset.metainfo={"classes":CLASS_NAMES}; loader.dataset.filter_cfg={"filter_empty_gt":False,"min_size":1}; loader.dataset.pipeline=pipeline
    cfg.test_dataloader=cfg.val_dataloader; cfg.val_evaluator.ann_file=str(args.prepared/"holdout.json"); cfg.test_evaluator=cfg.val_evaluator
    cfg.train_cfg={"type":"EpochBasedTrainLoop","max_epochs":args.epochs,"val_interval":1}
    cfg.optim_wrapper.type="AmpOptimWrapper"; cfg.optim_wrapper.loss_scale="dynamic"; cfg.optim_wrapper.optimizer.lr=0.0001; cfg.optim_wrapper.clip_grad={"max_norm":10.0,"norm_type":2}
    cfg.param_scheduler=[{"type":"LinearLR","start_factor":0.1,"by_epoch":False,"begin":0,"end":50},{"type":"CosineAnnealingLR","eta_min":0.00001,"begin":1,"end":args.epochs,"T_max":args.epochs-1,"by_epoch":True,"convert_to_iter_based":True}]
    cfg.default_hooks.logger.interval=20; cfg.default_hooks.checkpoint.interval=1; cfg.default_hooks.checkpoint.max_keep_ckpts=2; cfg.default_hooks.checkpoint.save_best="coco/bbox_mAP"; cfg.default_hooks.checkpoint.rule="greater"; cfg.custom_hooks=[]; cfg.randomness={"seed":20260814,"deterministic":True}; cfg.env_cfg.cudnn_benchmark=True
    config=args.output/"crv6_ma1_rtmdet_s_config.py"; cfg.dump(config); started=time.perf_counter(); Runner.from_cfg(cfg).train()
    best=sorted(args.output.glob("best_coco_bbox_mAP_epoch_*.pth")); report={"schema_version":1,"stage":"CRV6-05-MA1-TRAIN","route":"MA1","architecture":"official_mmdetection_v3.3.0_rtmdet_s","initial_checkpoint_sha256":args.expected_initial_sha256,"epochs":args.epochs,"batch_size":args.batch_size,"optimizer_lr":0.0001,"training_dataset":"G7_MOVING_TRAIN_ONLY","selection_dataset":"G7_MOVING_HOLDOUT_ONLY","MOVING_VAL_used":False,"G5_read":False,"G5_V2_read":False,"config_sha256":sha256(config),"best_checkpoint_count":len(best),"best_checkpoint_sha256":sha256(best[0]) if len(best)==1 else None,"duration_s":time.perf_counter()-started}
    (args.output/"CRV6_MA1_TRAIN_REPORT.json").write_text(json.dumps(report,indent=2)+"\n",encoding="utf-8")
    return 0 if len(best)==1 else 4


if __name__ == "__main__": raise SystemExit(main())
