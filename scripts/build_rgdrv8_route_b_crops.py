#!/usr/bin/env python3
"""Generate fixed Route B proposal crops from TRAIN and HOLDOUT_NEW."""
from __future__ import annotations
import argparse, hashlib, json, random, sys
from collections import Counter, defaultdict
from pathlib import Path
import cv2
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT/'starter_ws/src/sanitation_learning'))
from sanitation_learning.opr_c_rtmdet import patch_mmdet_cuda_nms  # noqa:E402
NAMES=('plastic_bottle','metal_can','paper_litter')
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def iou(a,b):
    x1,y1,x2,y2=max(a[0],b[0]),max(a[1],b[1]),min(a[2],b[2]),min(a[3],b[3]); q=max(0,x2-x1)*max(0,y2-y1); aa=max(0,a[2]-a[0])*max(0,a[3]-a[1]); bb=max(0,b[2]-b[0])*max(0,b[3]-b[1]); return q/max(aa+bb-q,1e-12)
def process(name,payload,root,model,out,threshold,batch_size):
    truths=defaultdict(list); images={int(x['id']):x for x in payload['images']}
    for row in payload['annotations']:
        x,y,w,h=row['bbox']; truths[int(row['image_id'])].append({'bbox':[x,y,x+w,y+h],'label':int(row['category_id'])-1,'instance_id':int(row.get('instance_id',row['id']))})
    from mmdet.apis import inference_detector
    rows=[]; encounter_hits=defaultdict(bool); encounter_truth=set()
    for image_id,items in truths.items():
        mission=images[image_id].get('mission_key',f"train:{images[image_id].get('scene_seed')}:{image_id}")
        for gt in items: encounter_truth.add((mission,gt['instance_id'],gt['label']))
    crop_dir=out/'images'/name.lower(); crop_dir.mkdir(parents=True,exist_ok=True)
    for off in range(0,len(payload['images']),batch_size):
        batch=payload['images'][off:off+batch_size]; paths=[root/x['file_name'] for x in batch]; outputs=inference_detector(model,[str(p) for p in paths]); outputs=outputs if isinstance(outputs,list) else [outputs]
        for image,path,result in zip(batch,paths,outputs):
            bgr=cv2.imread(str(path)); pred=result.pred_instances.to('cpu'); candidates=[]
            for box,score,label in zip(pred.bboxes.tolist(),pred.scores.tolist(),pred.labels.tolist()):
                if score<threshold: continue
                ranked=sorted(((iou(box,gt['bbox']),gt) for gt in truths[int(image['id'])]),reverse=True,key=lambda x:x[0]); best=ranked[0] if ranked else None
                if best and best[0]>=.5:
                    target=best[1]; class_id=target['label']+1; taxonomy='positive' if int(label)==target['label'] else 'wrong_class_positive'
                    mission=image.get('mission_key',f"train:{image.get('scene_seed')}:{image['id']}"); encounter_hits[(mission,target['instance_id'],target['label'])]=True
                else:
                    class_id=0; target=None; taxonomy='near_iou_confuser' if best and best[0]>=.1 else 'unmatched_background'
                candidates.append((box,float(score),int(label),class_id,taxonomy,target,best[0] if best else 0.0))
            for box,score,label,class_id,taxonomy,target,overlap in candidates:
                x1,y1,x2,y2=box; px=max(6,(x2-x1)*.25); py=max(6,(y2-y1)*.25); left=max(0,int(x1-px)); top=max(0,int(y1-py)); right=min(bgr.shape[1],int(x2+px+1)); bottom=min(bgr.shape[0],int(y2+py+1)); crop=bgr[top:bottom,left:right]
                if not crop.size: continue
                path_out=crop_dir/f"crop_{len(rows)+1:07d}.png"; cv2.imwrite(str(path_out),crop)
                rows.append({'id':len(rows)+1,'split':name,'crop_path':path_out.relative_to(out).as_posix(),'class_id':class_id,'class_name':'background' if class_id==0 else NAMES[class_id-1],'taxonomy':taxonomy,'source_image_id':int(image['id']),'mission_key':image.get('mission_key'),'frame_index':image.get('frame_index'),'negative_only':bool(image.get('negative_only')),'proposal_bbox_xyxy':box,'detector_score':score,'detector_label':label+1,'iou':overlap,'truth_instance_id':target['instance_id'] if target else None})
    return rows,{'image_count':len(payload['images']),'proposal_crop_count':len(rows),'class_counts':dict(Counter(x['class_name'] for x in rows)),'proposal_eventual_recall':sum(encounter_hits.values())/max(len(encounter_truth),1),'encounter_truth_count':len(encounter_truth),'encounter_hit_count':sum(encounter_hits.values())}
def main():
    p=argparse.ArgumentParser(); p.add_argument('--train-coco',type=Path,required=True); p.add_argument('--train-root',type=Path,required=True); p.add_argument('--holdout-coco',type=Path,required=True); p.add_argument('--holdout-root',type=Path,required=True); p.add_argument('--config',type=Path,required=True); p.add_argument('--checkpoint',type=Path,required=True); p.add_argument('--expected-sha256',required=True); p.add_argument('--output',type=Path,required=True); p.add_argument('--threshold',type=float,default=.03); p.add_argument('--batch-size',type=int,default=8); a=p.parse_args()
    if a.output.exists(): raise FileExistsError(a.output)
    if sha(a.checkpoint)!=a.expected_sha256: raise RuntimeError('Route A checkpoint SHA mismatch')
    a.output.mkdir(parents=True); patch_mmdet_cuda_nms(); from mmdet.apis import init_detector
    model=init_detector(str(a.config),str(a.checkpoint),device='cuda:0'); all_rows=[]; stats={}
    for name,coco,root in (('TRAIN',a.train_coco,a.train_root),('HOLDOUT_NEW',a.holdout_coco,a.holdout_root)):
        rows,stat=process(name,json.loads(coco.read_text()),root,model,a.output,a.threshold,a.batch_size); all_rows.extend(rows); stats[name]=stat
        (a.output/f'{name.lower()}_crops.json').write_text(json.dumps(rows,indent=2)+'\n')
    train=stats['TRAIN']['class_counts']; positive=sum(train.get(n,0) for n in NAMES); negative=train.get('background',0)
    report={'schema_version':1,'stage':'RGDRV8-03-ROUTE-B-CROPS','detector_checkpoint_sha256':a.expected_sha256,'observation_threshold':a.threshold,'splits':stats,'unique_train_positive_crops':positive,'unique_train_hard_negative_crops':negative,'HOLDOUT_proposals_fixed_once':True,'VAL_NEW_read':False,'G5_V2_read':False,'ROUTE_B_CROPS_PASS':positive>=3000 and negative>=3000 and stats['HOLDOUT_NEW']['proposal_eventual_recall']>=.98}
    (a.output/'ROUTE_B_CROP_REPORT.json').write_text(json.dumps(report,indent=2)+'\n'); print(json.dumps(report,indent=2)); return 0 if report['ROUTE_B_CROPS_PASS'] else 2
if __name__=='__main__': raise SystemExit(main())
