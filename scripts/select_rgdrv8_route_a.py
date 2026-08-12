#!/usr/bin/env python3
"""Select Route A checkpoint/threshold using HOLDOUT_NEW only."""
from __future__ import annotations
import argparse, hashlib, json, sys
from collections import defaultdict
from pathlib import Path
import cv2
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT/'starter_ws/src/sanitation_learning'))
from sanitation_learning.opr_c_rtmdet import patch_mmdet_cuda_nms  # noqa:E402
CLASSES=('plastic_bottle','metal_can','paper_litter'); THRESHOLDS=tuple(round(x/100,2) for x in range(3,96,2))
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def iou(a,b):
    x1,y1,x2,y2=max(a[0],b[0]),max(a[1],b[1]),min(a[2],b[2]),min(a[3],b[3]); inter=max(0,x2-x1)*max(0,y2-y1); aa=max(0,a[2]-a[0])*max(0,a[3]-a[1]); bb=max(0,b[2]-b[0])*max(0,b[3]-b[1]); return inter/max(aa+bb-inter,1e-12)
def truth(payload):
    images={int(x['id']):x for x in payload['images']}; by_frame=defaultdict(list); groups=defaultdict(list)
    for row in payload['annotations']:
        x,y,w,h=row['bbox']; item={'bbox':[x,y,x+w,y+h],'label':int(row['category_id'])-1,'small':float(row.get('bbox_short_side_px',min(w,h)))<18,'image_id':int(row['image_id'])}; by_frame[item['image_id']].append(item); key=(images[item['image_id']]['mission_key'],int(row.get('instance_id',row['id'])),item['label']); groups[key].append(item)
    return images,by_frame,groups
def metrics(payload,raw,t):
    images,by_frame,groups=truth(payload); eligible=[]
    for (mission,instance,label),rows in groups.items():
        detected=correct=False
        for gt in rows:
            matches=[p for p in raw[gt['image_id']] if p['score']>=t and iou(p['bbox'],gt['bbox'])>=.5]; detected|=bool(matches); correct|=any(p['label']==label for p in matches)
        eligible.append({'label':label,'detected':detected,'correct':correct,'small':any(r['small'] for r in rows)})
    predictions=correct_predictions=negative_predictions=0
    for image_id,preds in raw.items():
        selected=sorted((p for p in preds if p['score']>=t),key=lambda p:p['score'],reverse=True); used=set()
        for p in selected:
            predictions+=1; candidates=sorted(((iou(p['bbox'],gt['bbox']),idx,gt) for idx,gt in enumerate(by_frame[image_id]) if idx not in used and p['label']==gt['label']),reverse=True,key=lambda x:x[0])
            if candidates and candidates[0][0]>=.5: used.add(candidates[0][1]); correct_predictions+=1
            elif images[image_id].get('negative_only'): negative_predictions+=1
    small=[x for x in eligible if x['small']]; per={}
    for idx,name in enumerate(CLASSES):
        rows=[x for x in eligible if x['label']==idx]; per[name]=sum(x['correct'] for x in rows)/max(len(rows),1)
    result={'threshold':t,'eligible_encounters':len(eligible),'eventual_detection_recall':sum(x['detected'] for x in eligible)/max(len(eligible),1),'eventual_correct_class_recall':sum(x['correct'] for x in eligible)/max(len(eligible),1),'small_eventual_correct_recall':sum(x['correct'] for x in small)/max(len(small),1),'actionable_predictions':predictions,'correct_actionable_predictions':correct_predictions,'actionable_precision':correct_predictions/max(predictions,1),'wrong_actionable_rate':(predictions-correct_predictions)/max(predictions,1),'negative_only_actionable_rate':negative_predictions/max(sum(bool(x.get('negative_only')) for x in images.values()),1),'per_class_eventual_correct_recall':per}
    result['hard_constraints_pass']=all((result['eventual_correct_class_recall']>=.95,result['small_eventual_correct_recall']>=.90,result['actionable_precision']>=.95,result['wrong_actionable_rate']<=.01)); return result
def main():
    p=argparse.ArgumentParser(); p.add_argument('--prepared',type=Path,required=True); p.add_argument('--training',type=Path,required=True); p.add_argument('--output',type=Path,required=True); p.add_argument('--batch-size',type=int,default=8); a=p.parse_args()
    if a.output.exists(): raise FileExistsError(a.output)
    payload=json.loads((a.prepared/'holdout.json').read_text()); policy=json.loads((a.prepared/'ROUTE_A_SAMPLING_POLICY.json').read_text());
    if policy.get('VAL_NEW_read') is not False: raise RuntimeError('VAL boundary invalid')
    patch_mmdet_cuda_nms(); from mmdet.apis import init_detector,inference_detector
    checkpoints=sorted(a.training.glob('epoch_*.pth')); config=a.training/'rgdrv8_route_a_rtmdet_s.py'; results={}
    for checkpoint in checkpoints:
        model=init_detector(str(config),str(checkpoint),device='cuda:0'); raw={}
        for off in range(0,len(payload['images']),a.batch_size):
            batch=payload['images'][off:off+a.batch_size]; outputs=inference_detector(model,[str(a.prepared/x['file_name']) for x in batch]); outputs=outputs if isinstance(outputs,list) else [outputs]
            for image,out in zip(batch,outputs):
                pred=out.pred_instances.to('cpu'); raw[int(image['id'])]=[{'bbox':b,'score':float(s),'label':int(l)} for b,s,l in zip(pred.bboxes.tolist(),pred.scores.tolist(),pred.labels.tolist())]
        sweep=[metrics(payload,raw,t) for t in THRESHOLDS]; results[checkpoint.name]={'sha256':sha(checkpoint),'threshold_sweep':sweep}; del model
    ranked=[(row['hard_constraints_pass'],min(row['eventual_correct_class_recall'],row['actionable_precision']),row['small_eventual_correct_recall'],-row['wrong_actionable_rate'],name,row) for name,data in results.items() for row in data['threshold_sweep']]
    selected=max(ranked); name,row=selected[4],selected[5]; report={'schema_version':1,'stage':'RGDRV8-02-ROUTE-A-HOLDOUT-SELECTION','selection_data':'HOLDOUT_NEW_ONLY','checkpoint_results':results,'selected_checkpoint':name,'selected_checkpoint_sha256':results[name]['sha256'],'selected_threshold':row['threshold'],'selected_metrics':row,'VAL_NEW_read':False,'G5_V2_read':False,'RGDRV8_ROUTE_A_HOLDOUT_PASS':row['hard_constraints_pass']}
    a.output.parent.mkdir(parents=True,exist_ok=True); a.output.write_text(json.dumps(report,indent=2)+'\n'); print(json.dumps({k:report[k] for k in ('selected_checkpoint','selected_checkpoint_sha256','selected_threshold','selected_metrics','RGDRV8_ROUTE_A_HOLDOUT_PASS')},indent=2)); return 0 if report['RGDRV8_ROUTE_A_HOLDOUT_PASS'] else 4
if __name__=='__main__': raise SystemExit(main())
