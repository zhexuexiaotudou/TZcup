"""Validate a ten-frame native GT transport probe before any W6 capture."""
from __future__ import annotations
import hashlib,json
from pathlib import Path
from typing import Any
from hbm_evidence_common import atomic_json, fresh_directory

TOPICS={"rgb":"/camera/color/image_raw","semantic":"/g2/semantic_gt/labels_map","instance":"/g2/instance_gt/labels_map","camera_info":"/camera/color/camera_info"}
def validate(rows:list[dict[str,Any]])->dict[str,Any]:
 if len(rows)!=10: raise ValueError("gt_probe_requires_exactly_ten_frames")
 first=rows[0]
 required={"frame_id","stamp_ns","width","height","encoding","publisher_gid","topic_types"}
 if any(set(row)!=required for row in rows): raise ValueError("gt_probe_schema_invalid")
 for row in rows:
  if not isinstance(row['stamp_ns'],int) or row['stamp_ns']<=0 or row['width']!=848 or row['height']!=480 or row['frame_id']!=first['frame_id'] or row['encoding']!=first['encoding'] or row['publisher_gid']!=first['publisher_gid'] or row['topic_types']!={**TOPICS,'rgb_type':'sensor_msgs/msg/Image','semantic_type':'sensor_msgs/msg/Image','instance_type':'sensor_msgs/msg/Image','camera_info_type':'sensor_msgs/msg/CameraInfo'}: raise ValueError('gt_probe_identity_or_topic_contract_invalid')
 if first['encoding'] not in {'rgb8','bgr8'}: raise ValueError('gt_probe_encoding_unrecognized')
 return {'schema_version':1,'status':'GT_TRANSPORT_PROBE_VERIFIED','formal_passed':False,'sample_count':10,'frame_id':first['frame_id'],'width':848,'height':480,'encoding':first['encoding'],'publisher_gid':first['publisher_gid'],'topic_types':first['topic_types'],'rows_sha256':hashlib.sha256(json.dumps(rows,sort_keys=True,separators=(',',':')).encode()).hexdigest()}
def write(rows:list[dict[str,Any]],output:Path)->Path:
 fresh_directory(output,'gt_probe_output'); value=validate(rows); target=output/'gt_transport_probe.json'; atomic_json(target,value); return target
