#!/usr/bin/env python3
"""Warm-start official RTMDet-s for bounded RGDRV8 Route A."""
from __future__ import annotations
import argparse, hashlib, json, os, re, subprocess, sys, time
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'starter_ws/src/sanitation_learning'))
from sanitation_learning.opr_c_rtmdet import CLASS_NAMES, patch_mmdet_cuda_nms  # noqa:E402

def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def commit():
    value=os.environ.get('TZCUP_SOURCE_COMMIT','').strip()
    if value:
        if not re.fullmatch(r'[0-9a-f]{40}',value): raise RuntimeError('invalid source commit')
        return value
    return subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()

def main():
    p=argparse.ArgumentParser(); p.add_argument('--prepared',type=Path,required=True); p.add_argument('--checkpoint',type=Path,required=True); p.add_argument('--expected-sha256',required=True); p.add_argument('--output',type=Path,required=True); p.add_argument('--container-digest',required=True); p.add_argument('--epochs',type=int,default=6); p.add_argument('--batch-size',type=int,default=8); a=p.parse_args()
    if a.output.exists(): raise FileExistsError(a.output)
    if sha(a.checkpoint)!=a.expected_sha256: raise RuntimeError('warm-start SHA mismatch')
    policy=json.loads((a.prepared/'ROUTE_A_SAMPLING_POLICY.json').read_text())
    if policy.get('ROUTE_A_SAMPLING_PASS') is not True or policy.get('VAL_NEW_read') is not False: raise RuntimeError('invalid sampling boundary')
    a.output.mkdir(parents=True); patch_mmdet_cuda_nms()
    from mmengine.config import Config
    from mmengine.runner import Runner
    from mmdet.utils import register_all_modules
    register_all_modules(init_default_scope=True)
    cfg=Config.fromfile('/usr/local/lib/python3.12/dist-packages/mmdet/.mim/configs/rtmdet/rtmdet_s_8xb32-300e_coco.py')
    cfg.work_dir=str(a.output); cfg.load_from=str(a.checkpoint); cfg.resume=False; cfg.model.backbone.init_cfg=None; cfg.model.bbox_head.num_classes=3; cfg.model.test_cfg.score_thr=.001
    train=[{'type':'LoadImageFromFile','backend_args':None},{'type':'LoadAnnotations','with_bbox':True},{'type':'Resize','scale':(640,480),'keep_ratio':False},{'type':'PhotoMetricDistortion','brightness_delta':16,'contrast_range':(.85,1.15),'saturation_range':(.9,1.1),'hue_delta':5},{'type':'RandomFlip','prob':.5},{'type':'PackDetInputs'}]
    evaluation=[{'type':'LoadImageFromFile','backend_args':None},{'type':'Resize','scale':(640,480),'keep_ratio':False},{'type':'LoadAnnotations','with_bbox':True},{'type':'PackDetInputs','meta_keys':('img_id','img_path','ori_shape','img_shape','scale_factor')}]
    for loader,name,pipeline in ((cfg.train_dataloader,'fit.json',train),(cfg.val_dataloader,'holdout.json',evaluation)):
        loader.batch_size=a.batch_size; loader.num_workers=4; loader.dataset.data_root=str(a.prepared)+'/'; loader.dataset.ann_file=str(a.prepared/name); loader.dataset.data_prefix={'img':''}; loader.dataset.metainfo={'classes':CLASS_NAMES}; loader.dataset.filter_cfg={'filter_empty_gt':False,'min_size':1}; loader.dataset.pipeline=pipeline
    cfg.test_dataloader=cfg.val_dataloader; cfg.val_evaluator.ann_file=str(a.prepared/'holdout.json'); cfg.test_evaluator=cfg.val_evaluator
    cfg.train_cfg={'type':'EpochBasedTrainLoop','max_epochs':a.epochs,'val_interval':1}; cfg.optim_wrapper.type='AmpOptimWrapper'; cfg.optim_wrapper.loss_scale='dynamic'; cfg.optim_wrapper.optimizer.lr=.00005; cfg.optim_wrapper.clip_grad={'max_norm':10.0,'norm_type':2}
    cfg.param_scheduler=[{'type':'LinearLR','start_factor':.1,'by_epoch':False,'begin':0,'end':100},{'type':'CosineAnnealingLR','eta_min':.000005,'begin':1,'end':a.epochs,'T_max':max(a.epochs-1,1),'by_epoch':True,'convert_to_iter_based':True}]
    cfg.default_hooks.logger.interval=25; cfg.default_hooks.checkpoint.interval=1; cfg.default_hooks.checkpoint.max_keep_ckpts=6; cfg.default_hooks.checkpoint.save_best='coco/bbox_mAP'; cfg.default_hooks.checkpoint.rule='greater'; cfg.custom_hooks=[]; cfg.randomness={'seed':20260813,'deterministic':True}; cfg.env_cfg.cudnn_benchmark=True
    config=a.output/'rgdrv8_route_a_rtmdet_s.py'; cfg.dump(config); started=time.perf_counter(); Runner.from_cfg(cfg).train()
    epochs=sorted(a.output.glob('epoch_*.pth')); best=sorted(a.output.glob('best_coco_bbox_mAP_epoch_*.pth'))
    report={'schema_version':1,'stage':'RGDRV8-02-ROUTE-A-TRAIN','repository_commit':commit(),'container_digest':a.container_digest,'architecture':'official_mmdetection_v3.3.0_rtmdet_s','warm_start_sha256':a.expected_sha256,'config_sha256':sha(config),'sampling_policy_sha256':sha(a.prepared/'ROUTE_A_SAMPLING_POLICY.json'),'epochs':a.epochs,'batch_size':a.batch_size,'epoch_checkpoint_count':len(epochs),'best_checkpoint_count':len(best),'best_checkpoint_sha256':sha(best[0]) if len(best)==1 else None,'HOLDOUT_NEW_used_by_training_loop_for_checkpoint_selection':True,'VAL_NEW_read':False,'G5_V2_read':False,'duration_s':time.perf_counter()-started}
    (a.output/'ROUTE_A_TRAIN_REPORT.json').write_text(json.dumps(report,indent=2)+'\n'); print(json.dumps(report,indent=2)); return 0 if len(best)==1 else 2
if __name__=='__main__': raise SystemExit(main())
