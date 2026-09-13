#!/usr/bin/env python3
"""Offline P/R and false-positive samples; never supplies truth to a ROS node.

--frames: complete list of {frame_id, image_path(optional), predictions, truth}.
prediction={class_id,confidence,xyxy:[x1,y1,x2,y2]};
truth={object_id,class_id,xyxy:[x1,y1,x2,y2]}. Include empty background frames.
"""
import argparse
import collections
import json
import math
from pathlib import Path
import sys
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'starter_ws/src/sanitation_perception'))
from sanitation_perception.formal_random_scene_evaluator_core import BoxObservation,TruthBox,match_boxes
CLASSES=('plastic_bottle','metal_can','paper_litter','leaf_pile','puddle')


def score(frames):
    seen=set();total={c:collections.Counter(tp=0,fp=0,fn=0) for c in CLASSES};examples=[];misses=[];matches=[]
    if not isinstance(frames,list) or not frames:raise ValueError('nonempty complete frame list required')
    for f in frames:
        if not f['frame_id'] or f['frame_id'] in seen:raise ValueError('missing/duplicate frame id')
        seen.add(f['frame_id']);pred=f['predictions'];truth=f['truth']
        if len({r['object_id'] for r in truth})!=len(truth):raise ValueError('duplicate truth object in frame')
        for row in pred+truth:
            box=row['xyxy']
            if row['class_id'] not in CLASSES or len(box)!=4 or not all(math.isfinite(x) for x in box) or box[0]>=box[2] or box[1]>=box[3]:raise ValueError('invalid class or bbox')
        if any(not math.isfinite(r['confidence']) or not 0<=r['confidence']<=1 for r in pred):raise ValueError('invalid confidence')
        for cl in CLASSES:
            predictions=sorted([r for r in pred if r['class_id']==cl],key=lambda r:r['confidence'],reverse=True)
            r=match_boxes([BoxObservation(cl,x['confidence'],tuple(x['xyxy'])) for x in predictions],[TruthBox(x['object_id'],cl,tuple(x['xyxy'])) for x in truth if x['class_id']==cl],iou_threshold=.5)
            total[cl].update(dict(tp=r['true_positive_count'],fp=r['false_positive_count'],fn=r['false_negative_count']))
            for i in r['false_positive_indices']:
                examples.append({'frame_id':f['frame_id'],'image_path':f.get('image_path'),'prediction':predictions[i]})
            by_id={x['object_id']:x for x in truth}
            for object_id in r['unmatched_truth_object_ids']:
                misses.append({'frame_id':f['frame_id'],'image_path':f.get('image_path'),'truth':by_id[object_id]})
            for match in r['matches']:
                item={'frame_id':f['frame_id'],'class_id':cl,'object_id':match['truth_object_id'],'iou':match['iou']}
                prediction=predictions[match['prediction_index']]; reference=by_id[match['truth_object_id']]
                if 'map_xyz' in prediction and 'map_xyz' in reference:
                    item['planar_error_m']=math.hypot(prediction['map_xyz'][0]-reference['map_xyz'][0],prediction['map_xyz'][1]-reference['map_xyz'][1])
                matches.append(item)
    rows={}
    for cl,v in total.items():
        tp,fp,fn=(v[k] for k in ('tp','fp','fn'))
        rows[cl]={**v,'precision':tp/(tp+fp) if tp+fp else None,'recall':tp/(tp+fn) if tp+fn else None,'truth_support':tp+fn}
    # Missing classes remain visible; an empty prediction set does not pass.
    return {'status':'MEASURED_NOT_OFFICIAL_ACCEPTANCE','frame_count':len(frames),'class_metrics':rows,
            'macro_precision_conservative':sum(r['precision'] or 0 for r in rows.values())/len(CLASSES),
            'macro_recall_conservative':sum(r['recall'] or 0 for r in rows.values())/len(CLASSES),
            'false_positive_samples':examples,'false_negative_samples':misses,'matched_samples':matches,
            'unmeasured_classes':[c for c,r in rows.items() if not r['truth_support']],
            'iou_threshold':.5,'competition_perception_pass':False}


def main():
    p=argparse.ArgumentParser();p.add_argument('--frames',required=True,type=Path);p.add_argument('--output',required=True,type=Path);a=p.parse_args()
    if a.output.exists():raise SystemExit('fresh output required')
    result=score(json.loads(a.frames.read_text(encoding='utf-8')))
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')


if __name__=='__main__':main()
