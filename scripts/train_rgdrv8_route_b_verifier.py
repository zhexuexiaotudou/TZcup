#!/usr/bin/env python3
"""Train the single authorized MobileNetV3-small Route B verifier."""
from __future__ import annotations
import argparse, hashlib, json, random, sys, time
from collections import Counter
from pathlib import Path
import cv2, numpy as np
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT/'starter_ws/src/sanitation_learning'))
from sanitation_learning.g4_models import CandidateCropClassifier, CLASSIFIER_CLASSES  # noqa:E402
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def load_crop(root,row): return cv2.resize(cv2.cvtColor(cv2.imread(str(root/row['crop_path'])),cv2.COLOR_BGR2RGB),(192,192),interpolation=cv2.INTER_AREA)
def preload(rows,root): return np.stack([load_crop(root,row) for row in rows])
def scores(model,rows,images,device,batch=64):
    import torch; result=[]; model.eval()
    with torch.inference_mode():
        for off in range(0,len(rows),batch):
            part=rows[off:off+batch]; x=torch.from_numpy(images[off:off+batch].transpose(0,3,1,2).astype(np.float32)/255).to(device); prob=torch.softmax(model(x),1).cpu().numpy()
            result.extend({**r,'probabilities':p.tolist()} for r,p in zip(part,prob))
    return result
def evaluate(rows,threshold):
    confusion=[[0]*4 for _ in range(4)]
    for r in rows:
        p=r['probabilities']; target=max(range(1,4),key=lambda i:p[i]); pred=target if p[target]>=threshold and p[target]>p[0] else 0; confusion[int(r['class_id'])][pred]+=1
    per={}; f1=[]
    for i,name in enumerate(CLASSIFIER_CLASSES):
        tp=confusion[i][i]; truth=sum(confusion[i]); predicted=sum(row[i] for row in confusion); rec=tp/max(truth,1); prec=tp/max(predicted,1); value=2*rec*prec/max(rec+prec,1e-12); per[name]={'truth':truth,'recall':rec,'precision':prec,'f1':value}; f1.append(value)
    result={'threshold':threshold,'macro_f1':sum(f1)/4,'per_class':per,'background_specificity':per['background']['recall'],'metal_recall':per['metal_can']['recall'],'paper_precision':per['paper_litter']['precision']}; result['gate_pass']=all((result['macro_f1']>=.97,all(per[n]['recall']>=.95 for n in CLASSIFIER_CLASSES[1:]),result['background_specificity']>=.98,result['metal_recall']>=.95,result['paper_precision']>=.95)); return result
def main():
    import torch
    p=argparse.ArgumentParser(); p.add_argument('--crops-root',type=Path,required=True); p.add_argument('--output',type=Path,required=True); p.add_argument('--epochs',type=int,default=16); p.add_argument('--batch-size',type=int,default=64); a=p.parse_args()
    if a.output.exists(): raise FileExistsError(a.output)
    train=json.loads((a.crops_root/'train_crops.json').read_text()); hold=json.loads((a.crops_root/'holdout_new_crops.json').read_text()); pools={i:[r for r in train if int(r['class_id'])==i] for i in range(4)}
    if any(len(x)<500 for x in pools.values()): raise RuntimeError({i:len(x) for i,x in pools.items()})
    rng=random.Random(20260813); count=min(4000,min(len(x) for x in pools.values())); balanced=[]
    for rows in pools.values(): rng.shuffle(rows); balanced += rows[:count]
    rng.shuffle(balanced); train_images=preload(balanced,a.crops_root); hold_images=preload(hold,a.crops_root); device=torch.device('cuda:0'); model=CandidateCropClassifier().to(device); opt=torch.optim.AdamW(model.parameters(),lr=3e-4,weight_decay=1e-4); loss_fn=torch.nn.CrossEntropyLoss(); a.output.mkdir(parents=True); history=[]; best=None
    for epoch in range(1,a.epochs+1):
        model.train(); order=list(range(len(balanced))); rng.shuffle(order); total=0
        for off in range(0,len(order),a.batch_size):
            ids=order[off:off+a.batch_size]; rows=[balanced[i] for i in ids]; x=torch.from_numpy(train_images[ids].transpose(0,3,1,2).astype(np.float32)/255).to(device); y=torch.tensor([r['class_id'] for r in rows],device=device); opt.zero_grad(set_to_none=True); loss=loss_fn(model(x),y); loss.backward(); opt.step(); total+=float(loss)*len(rows)
        hold_scores=scores(model,hold,hold_images,device); sweep=[evaluate(hold_scores,x/100) for x in range(30,100,2)]; selected=max(sweep,key=lambda r:(r['gate_pass'],r['macro_f1'],r['background_specificity'],r['threshold'])); row={'epoch':epoch,'loss':total/len(balanced),'selected':selected}; history.append(row)
        if best is None or (selected['gate_pass'],selected['macro_f1'],selected['background_specificity'])>(best['selected']['gate_pass'],best['selected']['macro_f1'],best['selected']['background_specificity']): best=row; torch.save({'state_dict':model.state_dict(),'epoch':epoch,'threshold':selected['threshold']},a.output/'verifier.pt')
    checkpoint=torch.load(a.output/'verifier.pt',map_location=device,weights_only=True); model.load_state_dict(checkpoint['state_dict']); final_scores=scores(model,hold,hold_images,device); (a.output/'holdout_scores.json').write_text(json.dumps(final_scores,indent=2)+'\n'); selected=evaluate(final_scores,float(checkpoint['threshold']))
    report={'schema_version':1,'stage':'RGDRV8-03-ROUTE-B-VERIFIER','architecture':'torchvision_mobilenet_v3_small_imagenet1k_v1','classes':CLASSIFIER_CLASSES,'unique_train_counts':dict(Counter(r['class_name'] for r in train)),'balanced_unique_train_per_class':count,'train_crops_preloaded_once':True,'holdout_crops_preloaded_once':True,'epochs':a.epochs,'selected_epoch':checkpoint['epoch'],'selected_threshold':checkpoint['threshold'],'holdout_metrics':selected,'checkpoint_sha256':sha(a.output/'verifier.pt'),'VAL_NEW_read':False,'G5_V2_read':False,'ROUTE_B_VERIFIER_HOLDOUT_PASS':selected['gate_pass']}
    (a.output/'ROUTE_B_VERIFIER_REPORT.json').write_text(json.dumps(report,indent=2)+'\n'); print(json.dumps(report,indent=2)); return 0 if report['ROUTE_B_VERIFIER_HOLDOUT_PASS'] else 4
if __name__=='__main__': raise SystemExit(main())
