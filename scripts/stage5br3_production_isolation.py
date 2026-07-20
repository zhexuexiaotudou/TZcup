#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
import re


def main() -> int:
    parser=argparse.ArgumentParser(); parser.add_argument("--root",required=True); parser.add_argument("--rendered-urdf",required=True); parser.add_argument("--runtime-topics",required=True); parser.add_argument("--output",required=True); args=parser.parse_args()
    root=Path(args.root); urdf=Path(args.rendered_urdf).read_text(encoding="utf-8"); topics=Path(args.runtime_topics).read_text(encoding="utf-8").splitlines(); launch=(root/"starter_ws/src/sanitation_bringup/launch/sim.launch.py").read_text(encoding="utf-8")
    control_files=[]
    for path in sorted((root/"starter_ws/src").rglob("*")):
        if path.suffix not in {".py",".cpp",".hpp",".yaml"}: continue
        text=path.read_text(encoding="utf-8",errors="ignore")
        normalized=str(path).replace("\\","/")
        if ("semantic_gt" in text or "instance_gt" in text or "/ground_truth/semantic" in text or "/ground_truth/instance" in text) and "sanitation_learning" not in normalized and not normalized.endswith("stage5br3_g2_capture.launch.py"):
            control_files.append(str(path.relative_to(root)).replace("\\","/"))
    checks={"default_render_semantic_gt_sensor_count_zero":urdf.count('name="g2_semantic_gt"')==0,"default_render_instance_gt_sensor_count_zero":urdf.count('name="g2_instance_gt"')==0,"production_launch_semantic_gt_reference_count_zero":len(re.findall(r"semantic_gt|ground_truth/semantic",launch))==0,"production_launch_instance_gt_reference_count_zero":len(re.findall(r"instance_gt|ground_truth/instance",launch))==0,"runtime_semantic_gt_topic_count_zero":not any("semantic_gt" in t or "ground_truth/semantic" in t for t in topics),"runtime_instance_gt_topic_count_zero":not any("instance_gt" in t or "ground_truth/instance" in t for t in topics),"non_learning_control_gt_subscription_count_zero":not control_files}
    report={"schema_version":1,"stage":"Stage5BR3 production GT isolation","checks":checks,"runtime_topics":topics,"non_learning_files_with_training_gt_references":control_files,"production_isolation_pass":all(checks.values())}
    output=Path(args.output); output.parent.mkdir(parents=True,exist_ok=True); output.write_text(json.dumps(report,indent=2)+"\n"); print(json.dumps(report,indent=2)); return 0 if report["production_isolation_pass"] else 2


if __name__=="__main__": raise SystemExit(main())
