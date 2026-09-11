#!/usr/bin/env python3
"""Fail closed before launch when the product perception closure is incomplete."""
from __future__ import annotations
import argparse, hashlib, json
from pathlib import Path

def main() -> int:
    p=argparse.ArgumentParser(); p.add_argument('--artifact-root',type=Path,required=True); p.add_argument('--output',type=Path,required=True); a=p.parse_args()
    report={'status':'BLOCKED','required_topics':['/sensors/front_rgbd/image','/sensors/front_rgbd/depth/image_rect_raw/image','/sensors/front_rgbd/depth/image_rect_raw/camera_info'],'required_outputs':['/perception/garbage/targets']}
    manifest=a.artifact_root/'artifact_manifest.json'
    try:
        value=json.loads(manifest.read_text(encoding='utf-8'))
        if not isinstance(value,dict): raise ValueError('artifact manifest is not an object')
        required=(a.artifact_root/'dosod'/'dosod_mlp3x_s_tzcup_rep.onnx',a.artifact_root/'edgesam'/'edge_sam_3x_encoder.onnx',a.artifact_root/'edgesam'/'edge_sam_3x_decoder.onnx')
        missing=[str(x) for x in required if not x.is_file()]
        if missing: raise ValueError('missing model artifact: '+', '.join(missing))
        report.update(status='READY',artifact_manifest_sha256=hashlib.sha256(manifest.read_bytes()).hexdigest())
    except (OSError,ValueError,json.JSONDecodeError) as exc: report['reason']=str(exc)
    a.output.parent.mkdir(parents=True,exist_ok=True); a.output.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8'); print(report['status']); return 0 if report['status']=='READY' else 2
if __name__=='__main__': raise SystemExit(main())
