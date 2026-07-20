#!/usr/bin/env python3
from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import random
import time

import cv2
import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader, Dataset


RESOLUTION = (512, 384)
DISCRETE = {1: 1, 2: 2, 3: 3}
AREA = {4: 1, 5: 2}


class TinyUNet(nn.Module):
    def __init__(self, classes: int, base: int = 12):
        super().__init__()
        self.a = nn.Sequential(nn.Conv2d(3, base, 3, padding=1), nn.ReLU(), nn.Conv2d(base, base, 3, padding=1), nn.ReLU())
        self.b = nn.Sequential(nn.Conv2d(base, base*2, 3, padding=1), nn.ReLU(), nn.Conv2d(base*2, base*2, 3, padding=1), nn.ReLU())
        self.c = nn.Sequential(nn.Conv2d(base*2, base*2, 3, padding=1), nn.ReLU())
        self.d = nn.Sequential(nn.Conv2d(base*3, base, 3, padding=1), nn.ReLU(), nn.Conv2d(base, classes, 1))
    def forward(self, x):
        import torch.nn.functional as F
        a=self.a(x); b=self.b(F.max_pool2d(a,2)); c=self.c(b); u=F.interpolate(c,size=a.shape[-2:],mode="nearest"); return self.d(torch.cat((u,a),1))


def records(root: Path):
    rows=[]
    for scene in sorted((root/"scenes").glob("scene_*")):
        manifest=json.loads((scene/"scene_manifest.json").read_text()); capture=json.loads((scene/"capture_report.json").read_text())
        for record in capture["records"]: rows.append({"scene":manifest["scene_seed"],"world":manifest["world_id"],"split":manifest["split"],"negative_only":manifest["negative_only"],"rgb":scene/record["paths"]["rgb"],"semantic":scene/record["paths"]["semantic"],"instance":scene/record["paths"]["instance"]})
    return rows


class Frames(Dataset):
    def __init__(self, rows, mapping, augment=False, foreground_crop=False): self.rows=rows; self.mapping=mapping; self.augment=augment; self.foreground_crop=foreground_crop
    def __len__(self): return len(self.rows)
    def __getitem__(self,index):
        row=self.rows[index]; image=cv2.cvtColor(cv2.imread(str(row["rgb"])),cv2.COLOR_BGR2RGB); semantic=np.load(row["semantic"],allow_pickle=False)
        image=cv2.resize(image,RESOLUTION,interpolation=cv2.INTER_AREA); semantic=cv2.resize(semantic,RESOLUTION,interpolation=cv2.INTER_NEAREST)
        target=np.zeros_like(semantic,dtype=np.int64)
        for source,destination in self.mapping.items(): target[semantic==source]=destination
        if self.augment:
            rng=np.random.default_rng(20260720+index*97)
            if self.foreground_crop and np.any(target>0) and rng.random()<.7:
                ys,xs=np.nonzero(target>0); pick=int(rng.integers(0,len(xs))); cx,cy=int(xs[pick]),int(ys[pick]); crop_w,crop_h=320,240; x0=max(0,min(image.shape[1]-crop_w,cx-crop_w//2)); y0=max(0,min(image.shape[0]-crop_h,cy-crop_h//2)); image=image[y0:y0+crop_h,x0:x0+crop_w]; target=target[y0:y0+crop_h,x0:x0+crop_w]; image=cv2.resize(image,RESOLUTION,interpolation=cv2.INTER_AREA); target=cv2.resize(target.astype(np.uint8),RESOLUTION,interpolation=cv2.INTER_NEAREST).astype(np.int64)
            if rng.random()<.5: image=image[:,::-1].copy(); target=target[:,::-1].copy()
            image=np.clip(image.astype(np.float32)*rng.uniform(.75,1.25)+rng.uniform(-12,12),0,255).astype(np.uint8)
            if self.foreground_crop and rng.random()<.25: image=np.repeat(cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)[:,:,None],3,axis=2)
            elif self.foreground_crop and rng.random()<.25: image=image[:,:,rng.permutation(3)]
        tensor=np.ascontiguousarray(image.transpose(2,0,1).astype(np.float32)/255)
        return torch.from_numpy(tensor),torch.from_numpy(target),index


def confusion(truth,pred,classes):
    return np.bincount(classes*truth.reshape(-1)+pred.reshape(-1),minlength=classes**2).reshape(classes,classes)


def pixel_metrics(matrix):
    values=[]; ious=[]
    for cls in range(1,len(matrix)):
        tp=matrix[cls,cls]; fp=matrix[:,cls].sum()-tp; fn=matrix[cls,:].sum()-tp
        p=tp/max(tp+fp,1); r=tp/max(tp+fn,1); values.append(2*p*r/max(p+r,1e-12)); ious.append(tp/max(tp+fp+fn,1))
    return {"macro_f1":float(np.mean(values)),"foreground_miou":float(np.mean(ious)),"per_class_f1":values,"per_class_iou":ious}


def iou(a,b):
    ax1,ay1,ax2,ay2=a; bx1,by1,bx2,by2=b; inter=max(0,min(ax2,bx2)-max(ax1,bx1))*max(0,min(ay2,by2)-max(ay1,by1)); union=(ax2-ax1)*(ay2-ay1)+(bx2-bx1)*(by2-by1)-inter; return inter/max(union,1)


def components(mask,label,score=None):
    count,labels,stats,_=cv2.connectedComponentsWithStats((mask==label).astype(np.uint8),8); rows=[]
    for idx in range(1,count):
        x,y,w,h,area=stats[idx]
        if area<2: continue
        rows.append({"bbox":[int(x),int(y),int(x+w),int(y+h)],"area":int(area),"short":int(min(w,h)),"score":float(score[labels==idx].mean()) if score is not None else 1.0})
    return rows


def detection_metrics(model, rows, device, stress=None):
    model.eval(); predictions=[]; ground=defaultdict(list); matrix=np.zeros((4,4),dtype=np.int64)
    with torch.no_grad():
        for frame,row in enumerate(rows):
            image=cv2.cvtColor(cv2.imread(str(row["rgb"])),cv2.COLOR_BGR2RGB); sem=np.load(row["semantic"],allow_pickle=False); inst=np.load(row["instance"],allow_pickle=False)
            image=cv2.resize(image,RESOLUTION,interpolation=cv2.INTER_AREA); sem=cv2.resize(sem,RESOLUTION,interpolation=cv2.INTER_NEAREST); inst=cv2.resize(inst.astype(np.int32),RESOLUTION,interpolation=cv2.INTER_NEAREST)
            if stress=="gray": image=np.repeat(cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)[:,:,None],3,axis=2)
            elif stress=="permute": image=image[:,:,[2,0,1]]
            elif stress=="dark": image=np.clip(image.astype(np.float32)*.55,0,255).astype(np.uint8)
            target=np.zeros_like(sem,dtype=np.int64)
            for source,dest in DISCRETE.items(): target[sem==source]=dest
            tensor=torch.from_numpy(np.ascontiguousarray(image.transpose(2,0,1)[None].astype(np.float32)/255)).to(device); prob=torch.softmax(model(tensor),1)[0].cpu().numpy(); pred=prob.argmax(0).astype(np.int64); matrix+=confusion(target,pred,4)
            for cls in (1,2,3):
                for item in components(pred,cls,prob[cls]): predictions.append({**item,"frame":frame,"class":cls})
            for iid in (int(v) for v in np.unique(inst) if int(v)!=0):
                m=inst==iid; labels=target[m]; cls=int(np.bincount(labels,minlength=4).argmax())
                if cls==0: continue
                ys,xs=np.nonzero(m); ground[(frame,cls)].append({"bbox":[int(xs.min()),int(ys.min()),int(xs.max()+1),int(ys.max()+1)],"short":int(min(xs.max()-xs.min()+1,ys.max()-ys.min()+1))})
    def ap_at(threshold):
        aps=[]
        for cls in (1,2,3):
            preds=sorted((p for p in predictions if p["class"]==cls),key=lambda p:p["score"],reverse=True); total=sum(len(v) for (f,c),v in ground.items() if c==cls); used=defaultdict(set); tp=[]; fp=[]
            for p in preds:
                choices=ground[(p["frame"],cls)]; scores=[iou(p["bbox"],g["bbox"]) if idx not in used[p["frame"]] else -1 for idx,g in enumerate(choices)]; best=int(np.argmax(scores)) if scores else -1
                hit=best>=0 and scores[best]>=threshold; tp.append(int(hit)); fp.append(int(not hit));
                if hit: used[p["frame"]].add(best)
            if total==0: aps.append(0.0); continue
            t=np.cumsum(tp); f=np.cumsum(fp); recall=t/total; precision=t/np.maximum(t+f,1); aps.append(float(np.mean([max(precision[recall>=r],default=0) for r in np.linspace(0,1,101)])))
        return float(np.mean(aps))
    matched=0; total_gt=sum(len(v) for v in ground.values()); small_total=sum(g["short"]<16 for values in ground.values() for g in values); small_match=0
    for key,gts in ground.items():
        frame,cls=key; preds=sorted((p for p in predictions if p["frame"]==frame and p["class"]==cls and p["score"]>=.35),key=lambda p:p["score"],reverse=True); used=set()
        for p in preds:
            scores=[iou(p["bbox"],g["bbox"]) if idx not in used else -1 for idx,g in enumerate(gts)]; best=int(np.argmax(scores)) if scores else -1
            if best>=0 and scores[best]>=.5: used.add(best); matched+=1; small_match+=int(gts[best]["short"]<16)
    threshold_predictions=sum(p["score"]>=.35 for p in predictions); precision=matched/max(threshold_predictions,1); recall=matched/max(total_gt,1); negatives=[i for i,r in enumerate(rows) if r["negative_only"]]; negative_fp=sum(p["score"]>=.35 and p["frame"] in negatives for p in predictions)
    return {**pixel_metrics(matrix),"confidence_threshold":.35,"precision":precision,"recall":recall,"f1":2*precision*recall/max(precision+recall,1e-12),"ap50":ap_at(.5),"ap50_95":float(np.mean([ap_at(t) for t in np.arange(.5,1,.05)])),"small_object_recall":small_match/max(small_total,1),"small_object_gt_count":small_total,"same_color_negative_fp_per_frame":negative_fp/max(len(negatives),1),"negative_only_frame_count":len(negatives)}


def train_task(name,mapping,classes,train_rows,in_rows,cross_rows,output,device,attempt):
    foreground_crop=attempt>=2; epochs=8 if attempt==1 else 16 if attempt==2 else 24
    base=12 if attempt==1 else 20 if attempt==2 else 32
    dataset=Frames(train_rows,mapping,True,foreground_crop); loader=DataLoader(dataset,batch_size=2,shuffle=True,num_workers=2,pin_memory=True,generator=torch.Generator().manual_seed(20260720)); model=TinyUNet(classes,base=base).to(device)
    counts=torch.zeros(classes)
    for _,target,_ in DataLoader(Frames(train_rows,mapping),batch_size=1,num_workers=2): counts+=torch.bincount(target.reshape(-1),minlength=classes)
    weights=(counts.sum()/torch.clamp(counts*classes,min=1)).pow(.55); weights[0]=.03; weights=weights.to(device)
    optimizer=torch.optim.AdamW(model.parameters(),lr=8e-4,weight_decay=1e-4); curves=[]; started=time.perf_counter(); torch.cuda.reset_peak_memory_stats() if device.type=="cuda" else None
    for epoch in range(1,epochs+1):
        model.train(); losses=[]
        for image,target,_ in loader:
            image,target=image.to(device,non_blocking=True),target.to(device,non_blocking=True); optimizer.zero_grad(set_to_none=True); logits=model(image); ce=torch.nn.functional.cross_entropy(logits,target,weight=weights,reduction="none"); probs=torch.softmax(logits,1); true_prob=probs.gather(1,target[:,None]).squeeze(1); focal=(((1-true_prob)**1.5)*ce).mean(); onehot=torch.nn.functional.one_hot(target,classes).permute(0,3,1,2).float(); inter=(probs[:,1:]*onehot[:,1:]).sum((0,2,3)); den=probs[:,1:].sum((0,2,3))+onehot[:,1:].sum((0,2,3)); loss=focal+1-((2*inter+1)/(den+1)).mean(); loss.backward(); optimizer.step(); losses.append(float(loss.detach()))
        curves.append({"epoch":epoch,"loss":float(np.mean(losses))})
    duration=time.perf_counter()-started; peak=torch.cuda.max_memory_allocated()/1048576 if device.type=="cuda" else None
    model.eval(); onnx_path=output/f"{name}.onnx"; torch.onnx.export(model,torch.zeros((1,3,RESOLUTION[1],RESOLUTION[0]),device=device),onnx_path,input_names=["images"],output_names=["logits"],opset_version=17); model.cpu()
    import onnx,onnxruntime as ort
    graph=onnx.load(onnx_path); onnx.checker.check_model(graph); operators=defaultdict(int)
    for node in graph.graph.node: operators[node.op_type]+=1
    session=ort.InferenceSession(str(onnx_path),providers=["CPUExecutionProvider"]); sample=np.zeros((1,3,RESOLUTION[1],RESOLUTION[0]),np.float32); times=[]
    for _ in range(12): s=time.perf_counter(); session.run(None,{"images":sample}); times.append((time.perf_counter()-s)*1000)
    model=model.to(device)
    if name=="discrete_detector":
        in_metrics=detection_metrics(model,in_rows,device); cross_metrics=detection_metrics(model,cross_rows,device); stress=[detection_metrics(model,cross_rows,device,s)["f1"] for s in ("gray","permute","dark")]
    else:
        def area_eval(rows):
            matrix=np.zeros((classes,classes),np.int64); ds=Frames(rows,mapping); model.eval()
            with torch.no_grad():
                for image,target,_ in DataLoader(ds,batch_size=2,num_workers=2):
                    for truth,pred in zip(target.numpy(),model(image.to(device)).argmax(1).cpu().numpy()): matrix[:]+=confusion(truth,pred,classes)
            return pixel_metrics(matrix)
        in_metrics=area_eval(in_rows); cross_metrics=area_eval(cross_rows); stress=[]
    hypothesis=("separate discrete objects from area masks to reduce class-imbalance interference" if attempt==1 else "foreground-centered scale augmentation plus color invariance should recover tiny targets without using test data" if attempt==2 else "higher-capacity split backbones and longer optimization should convert the attempt-2 detector gain into cross-world gate compliance")
    return {"task":name,"hypothesis":hypothesis,"resolution":list(RESOLUTION),"epochs":epochs,"training_frames":len(train_rows),"duration_sec":duration,"curves":curves,"class_weights":weights.cpu().tolist(),"peak_training_vram_mib":peak,"parameter_count":sum(p.numel() for p in model.parameters()),"onnx_path":onnx_path.name,"onnx_sha256":hashlib.sha256(onnx_path.read_bytes()).hexdigest(),"onnx_bytes":onnx_path.stat().st_size,"onnx_operator_inventory":dict(operators),"onnx_cpu_latency_ms":{"p50":float(np.percentile(times,50)),"p95":float(np.percentile(times,95))},"in_domain":in_metrics,"cross_world":cross_metrics,"color_stress_f1":float(np.mean(stress)) if stress else None}


def main():
    parser=argparse.ArgumentParser(); parser.add_argument("--data-root",required=True); parser.add_argument("--output",required=True); parser.add_argument("--attempt",type=int,default=1); args=parser.parse_args(); output=Path(args.output); output.mkdir(parents=True,exist_ok=True)
    random.seed(20260720); np.random.seed(20260720); torch.manual_seed(20260720); device=torch.device("cuda" if torch.cuda.is_available() else "cpu"); rows=records(Path(args.data_root)); in_scenes={12,13,25,26,38,39}; train=[r for r in rows if r["split"]=="train" and r["scene"] not in in_scenes]; in_rows=[r for r in rows if r["scene"] in in_scenes]; cross=[r for r in rows if r["split"]=="val"]
    detector=train_task("discrete_detector",DISCRETE,4,train,in_rows,cross,output,device,args.attempt); area=train_task("area_segmenter",AREA,3,train,in_rows,cross,output,device,args.attempt)
    negative_fp=max(detector["in_domain"]["same_color_negative_fp_per_frame"],detector["cross_world"]["same_color_negative_fp_per_frame"])
    gates={"detector_in_domain_f1":detector["in_domain"]["f1"]>=.90,"detector_cross_world_f1":detector["cross_world"]["f1"]>=.70,"detector_small_object_recall":detector["cross_world"]["small_object_recall"]>=.70,"area_cross_world_miou":area["cross_world"]["foreground_miou"]>=.75,"color_stress_f1":detector["color_stress_f1"]>=.60,"same_color_negative_fp":negative_fp<=.05}
    report={"schema_version":1,"stage":f"Stage5BR3 split-model screening attempt {args.attempt}","attempt":args.attempt,"attempt_limit":3,"device":str(device),"test_split_used_for_selection":False,"same_color_negative_fp_rate_for_gate":negative_fp,"same_color_negative_fp_threshold":.05,"detector":detector,"area_segmenter":area,"gates":gates,"screening_pass":all(gates.values())}; (output/f"model_screening_attempt_{args.attempt}.json").write_text(json.dumps(report,indent=2)+"\n"); print(json.dumps({"gates":gates,"screening_pass":report["screening_pass"]},indent=2)); raise SystemExit(0 if report["screening_pass"] else 2)


if __name__=="__main__": main()
