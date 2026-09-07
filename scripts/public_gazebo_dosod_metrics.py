"""Pure DRAFT-only W6 metric math; it cannot claim competition acceptance."""
from __future__ import annotations
import math
from typing import Any
import numpy as np

CLASSES=("litter_cube","fallen_leaves","dust_or_soil","puddle")

def raw_metrics(fp32: np.ndarray, candidate: np.ndarray) -> dict[str,float]:
    if fp32.shape != candidate.shape or not np.isfinite(fp32).all() or not np.isfinite(candidate).all(): raise ValueError("raw_output_nonfinite_or_shape_invalid")
    a,b=fp32.reshape(-1).astype(float),candidate.reshape(-1).astype(float); an,bn=float(np.linalg.norm(a)),float(np.linalg.norm(b))
    cosine=1.0 if an==bn==0 else (0.0 if an==0 or bn==0 else float(np.dot(a,b)/(an*bn)))
    d=b-a; return {"cosine":cosine,"normalized_rmse":float(np.sqrt(np.mean(d*d))/max(math.sqrt(float(np.mean(a*a))),1e-12)),"pixel_mae":float(np.mean(abs(d))),"pixel_p95":float(np.percentile(abs(d),95))}

def _iou(a,b):
    x1,y1=max(a[0],b[0]),max(a[1],b[1]); x2,y2=min(a[2],b[2]),min(a[3],b[3]); inter=max(0,x2-x1)*max(0,y2-y1); union=(a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-inter
    return inter/union if union else 0.0

def evaluate(frames: list[dict[str,Any]]) -> dict[str,Any]:
    counts={c:{"tp":0,"fp":0,"fn":0,"gt":0} for c in CLASSES}; total_frames=len(frames)
    for frame in frames:
        truth=frame.get("truth",[]); pred=frame.get("predictions",[])
        for row in [*truth,*pred]:
            if row.get("class_id") not in CLASSES or len(row.get("xyxy",[]))!=4: raise ValueError("metric_row_invalid")
        used=set()
        for pindex,p in sorted(enumerate(pred),key=lambda item:(-float(item[1].get("score",0)),item[0])):
            candidates=[( _iou(p["xyxy"],t["xyxy"]),tindex) for tindex,t in enumerate(truth) if tindex not in used and t["class_id"]==p["class_id"]]
            candidates=[item for item in candidates if item[0]>=.5]
            if candidates:
                _,chosen=max(candidates,key=lambda item:(item[0],-item[1])); used.add(chosen); counts[p["class_id"]]["tp"]+=1
            else: counts[p["class_id"]]["fp"]+=1
        for index,t in enumerate(truth):
            counts[t["class_id"]]["gt"]+=1
            if index not in used: counts[t["class_id"]]["fn"]+=1
    classes={}; ps=[];rs=[];fs=[]
    for name,row in counts.items():
        tp,fp,fn=row["tp"],row["fp"],row["fn"]; p=tp/(tp+fp) if tp+fp else 0.; r=tp/(tp+fn) if tp+fn else 0.; f=2*p*r/(p+r) if p+r else 0.; classes[name]={**row,"precision":p,"recall":r,"f1":f};ps.append(p);rs.append(r);fs.append(f)
    return {"claim_scope":"NON_FORMAL_ENGINEERING_MODEL_GATE","competition_acceptance_passed":False,"product_acceptance_passed":False,"evaluated_frames":total_frames,"classes":classes,"macro_precision":sum(ps)/4,"macro_recall":sum(rs)/4,"macro_f1":sum(fs)/4,"fp_per_frame":sum(v["fp"] for v in counts.values())/max(total_frames,1)}

def gates(metrics:dict[str,Any], reference:dict[str,Any]|None=None)->bool:
    if metrics["evaluated_frames"] != 100 or any(metrics["classes"][c]["gt"]<=0 or min(metrics["classes"][c][k] for k in ("precision","recall","f1"))<.8 for c in CLASSES) or min(metrics[k] for k in ("macro_precision","macro_recall","macro_f1"))<.8 or metrics["fp_per_frame"]>.2:return False
    if reference is not None:
        if any(reference[k]-metrics[k]>.02 for k in ("macro_precision","macro_recall","macro_f1")) or metrics["fp_per_frame"]-reference["fp_per_frame"]>.05:return False
    return True
