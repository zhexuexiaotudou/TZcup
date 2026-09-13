#!/usr/bin/env python3
"""Score sealed sensor outputs against separately generated, visually checked geometry."""
import argparse,collections,hashlib,json,math
from pathlib import Path
import cv2
import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
from competition_perception_score import score,CLASSES

def ns(s):return s.sec*1_000_000_000+s.nanosec

def polygon(obj,camera,k):
 x,y,z=obj['center'];sx,sy,sz=obj['size'];vertices=[]
 for height in (z-sz/2,z+sz/2):
  if obj['shape']=='cylinder':
   vertices.extend((x+sx/2*math.cos(a),y+sy/2*math.sin(a),height) for a in np.linspace(0,2*math.pi,128,endpoint=False))
  else:vertices.extend((x+dx*sx/2,y+dy*sy/2,height) for dx in (-1,1) for dy in (-1,1))
 points=[]
 for px,py,pz in vertices:
  depth=camera[2]-pz
  points.append([k[0]*(px-camera[0])/depth+k[2],k[4]*(camera[1]-py)/depth+k[5]])
 return cv2.convexHull(np.asarray(points,dtype=np.float32)).reshape(-1,2)

def image_array(m,depth=False):
 dtype='<f4' if depth and m.encoding=='32FC1' else '<u2' if depth and m.encoding=='16UC1' else 'u1'
 row=np.frombuffer(bytes(m.data),dtype=dtype).reshape(m.height,m.step//np.dtype(dtype).itemsize)
 if depth:return row[:,:m.width].astype(np.float32)*(0.001 if m.encoding=='16UC1' else 1.)
 if m.encoding=='mono8':return row[:,:m.width]
 rgb=row[:,:m.width*3].reshape(m.height,m.width,3)
 return rgb[:,:,::-1] if m.encoding=='bgr8' else rgb

def boxes(message):
 if message is None:return []
 result=[]
 for d in message.detections:
  if not d.results:continue
  h=d.results[0].hypothesis;b=d.bbox;cx,cy=b.center.position.x,b.center.position.y
  result.append({'class_id':h.class_id,'confidence':h.score,'xyxy':[cx-b.size_x/2,cy-b.size_y/2,cx+b.size_x/2,cy+b.size_y/2]})
 return result

def main():
 p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--plan',type=Path,required=True);p.add_argument('--annotations',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False);(a.output/'frames').mkdir()
 plan=json.loads(a.plan.read_text());start=json.loads((a.run/'capture-start.json').read_text())['sim_s'];review=json.loads((a.run/'visibility-review.json').read_text())
 annotations=json.loads(a.annotations.read_text())['frames'];annotation_map={f['frame_id']:f for f in annotations}
 if len(annotation_map)!=30 or len(annotations)!=30:raise ValueError('exactly 30 unique reviewed frame annotations required')
 topics={'rgb':'/sensors/front_rgbd/depth/image_rect_raw/image','depth':'/sensors/front_rgbd/depth/image_rect_raw/depth_image','info':'/sensors/front_rgbd/depth/image_rect_raw/camera_info','raw':'/perception/garbage/raw_detections_2d','policy':'/perception/garbage/detections_2d','xyz':'/perception/garbage/detections_3d','seg':'/perception/garbage/segmentation','targets':'/perception/garbage/targets'}
 lookup={v:k for k,v in topics.items()};data={k:{} for k in topics};counts=collections.Counter();diagnostics=[];bagfiles=list((a.run/'bag').glob('*.mcap'))
 for path in bagfiles:
  with path.open('rb') as stream:
   reader=make_reader(stream,decoder_factories=[DecoderFactory()],validate_crcs=True)
   if reader.get_summary() is None:raise RuntimeError('unsealed MCAP')
   for _,ch,_,m in reader.iter_decoded_messages():
    counts[ch.topic]+=1
    if ch.topic in lookup:data[lookup[ch.topic]][ns(m.header.stamp)]=m
    if ch.topic=='/perception/garbage/diagnostics':diagnostics.append(json.loads(m.data))
 selected=[];rawframes=[];policyframes=[];visibility=[];missing=[];metadata=[];montage=[];pixel={c:collections.Counter(intersection=0,union=0,truth=0,predicted=0) for c in CLASSES};rgbstamps=list(data['rgb'])
 for index,offset in enumerate(range(2,61,2),1):
  target=start+offset;key=min(rgbstamps,key=lambda t:abs(t/1e9-target)) if rgbstamps else None;fid=f'frame_{index:02d}';truth=[];path=f'frames/{fid}.png'
  if key is None or abs(key/1e9-target)>.26:
   missing.append({'frame_id':fid,'expected_sim_s':target,'reason':'RGB absent in fixed tolerance'});rawframes.append({'frame_id':fid,'predictions':[],'truth':[]});policyframes.append({'frame_id':fid,'predictions':[],'truth':[]});continue
  rgb=data['rgb'][key];im=image_array(rgb);cv2.imwrite(str(a.output/path),cv2.cvtColor(im,cv2.COLOR_RGB2BGR));height,width=im.shape[:2]
  annotation=annotation_map[fid]
  if hashlib.sha256((a.output/path).read_bytes()).hexdigest()!=annotation['image_sha256']:raise ValueError('annotation image hash mismatch '+fid)
  info_key=min(data['info'],key=lambda t:abs(t-key)) if data['info'] else None;depth_key=min(data['depth'],key=lambda t:abs(t-key)) if data['depth'] else None
  if info_key is None or depth_key is None:raise RuntimeError('missing calibration/depth')
  info=data['info'][info_key];depthmsg=data['depth'][depth_key];depth=image_array(depthmsg,True)
  valid_context=(abs(depth_key-key)<=30_000_000 and abs(info_key-key)<=30_000_000 and rgb.header.frame_id==depthmsg.header.frame_id==info.header.frame_id and rgb.width==depthmsg.width==info.width and rgb.height==depthmsg.height==info.height and depthmsg.encoding in ('32FC1','16UC1'))
  metadata.append({'frame_id':fid,'sim_s':key/1e9,'rgb_depth_skew_s':abs(key-depth_key)/1e9,'rgb_info_skew_s':abs(key-info_key)/1e9,'depth_encoding':depthmsg.encoding,'dimensions':[width,height],'registered_context_valid':valid_context})
  labelmask=np.zeros((height,width),dtype=np.uint8);annotated=cv2.cvtColor(im,cv2.COLOR_RGB2BGR)
  visible_ids=set(annotation['visible_object_ids'])
  if not visible_ids.issubset({x['object_id'] for x in plan['objects']}):raise ValueError('unknown annotated object')
  if visible_ids:
   for obj in (x for x in plan['objects'] if x['object_id'] in visible_ids):
    poly=polygon(obj,plan['camera_map_xyz'],info.k);low=poly.min(axis=0);high=poly.max(axis=0)
    box=[float(max(0,low[0])),float(max(0,low[1])),float(min(width,high[0])),float(min(height,high[1]))]
    center_depth=plan['camera_map_xyz'][2]-(obj['center'][2]+obj['size'][2]/2)
    u=int(round(info.k[0]*(obj['center'][0]-plan['camera_map_xyz'][0])/center_depth+info.k[2]));v=int(round(info.k[4]*(plan['camera_map_xyz'][1]-obj['center'][1])/center_depth+info.k[5]));measured=float(depth[v,u]) if 0<=u<width and 0<=v<height else None
    visible=measured is not None and math.isfinite(measured) and abs(measured-center_depth)<.03
    visibility.append({'frame_id':fid,'object_id':obj['object_id'],'expected_depth_m':center_depth,'measured_depth_m':measured if measured is not None and math.isfinite(measured) else None,'surface_visible':visible})
    # Predeclared objects remain in the table even if visibility proof fails;
    # such a failure invalidates this dataset, never silently drops hard cases.
    truth.append({'object_id':obj['object_id'],'class_id':obj['class_id'],'xyxy':box})
    cv2.fillConvexPoly(labelmask,np.rint(poly).astype(np.int32),CLASSES.index(obj['class_id'])+1)
    cv2.rectangle(annotated,(int(box[0]),int(box[1])),(int(box[2]),int(box[3])),(0,255,255),1);cv2.putText(annotated,obj['class_id'],(int(box[0]),max(12,int(box[1])-3)),cv2.FONT_HERSHEY_SIMPLEX,.35,(0,0,0),1)
  raw=boxes(data['raw'].get(key));policy=boxes(data['policy'].get(key));rawframes.append({'frame_id':fid,'image_path':path,'predictions':raw,'truth':truth});policyframes.append({'frame_id':fid,'image_path':path,'predictions':policy,'truth':truth})
  selected.append({'frame_id':fid,'offset_s':offset,'expected_sim_s':target,'actual_sim_s':key/1e9,'raw_output_present':key in data['raw'],'policy_output_present':key in data['policy'],'truth_count':len(truth),'raw_count':len(raw),'policy_count':len(policy)})
  if key in data['seg']:
   seg=image_array(data['seg'][key])
   for ci,c in enumerate(CLASSES,1):
    predicted=seg==ci;reference=labelmask==ci;pixel[c].update(intersection=int(np.count_nonzero(predicted&reference)),union=int(np.count_nonzero(predicted|reference)),truth=int(reference.sum()),predicted=int(predicted.sum()))
  else:
   for ci,c in enumerate(CLASSES,1):pixel[c].update(union=int(np.count_nonzero(labelmask==ci)),truth=int(np.count_nonzero(labelmask==ci)))
  for d in raw:
   x1,y1,x2,y2=map(int,d['xyxy']);cv2.rectangle(annotated,(x1,y1),(x2,y2),(0,0,255),1)
  cv2.imwrite(str(a.output/'frames'/f'{fid}_evaluation.png'),annotated)
  thumb=cv2.resize(annotated,(424,240));cv2.putText(thumb,f'{fid} raw={len(raw)} policy={len(policy)} truth={len(truth)}',(5,18),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,0,0),1);montage.append(thumb)
 rawscore=score(rawframes);policyscore=score(policyframes)
 states=collections.Counter();uuids=set();finite_targets=0
 for m in data['targets'].values():
  for t in m.targets:
   states[t.track_state]+=1;uuids.add(t.uuid)
   xyz=t.map_pose.pose.position;finite_targets+=int(m.header.frame_id=='map' and all(math.isfinite(x) for x in (xyz.x,xyz.y,xyz.z)))
 internal={}
 for d in diagnostics:
  if d.get('last_source_stamp_s',-1)>=0:internal[d['last_source_stamp_s']]=d.get('track_state_counts',{})
 state_frames=collections.Counter()
 for d in internal.values():state_frames.update(d)
 valid=not missing and len(selected)==30 and all(v['surface_visible'] for v in visibility) and all(m['registered_context_valid'] for m in metadata) and set(review['visible_classes'])==set(CLASSES)
 summary={'dataset_valid':valid,'frame_protocol':'30 fixed epochs, every 2 sim seconds, no post-hoc selection','selected_frames':selected,'missing_frames':missing,'frame_metadata':metadata,'visibility_checks':visibility,'raw_detection_metrics':rawscore,'policy_detection_metrics':policyscore,'segmentation_geometry_iou':{c:{**d,'iou':d['intersection']/d['union'] if d['union'] else None} for c,d in pixel.items()},'published_target_state_message_counts':dict(states),'unique_published_target_ids':len(uuids),'internal_state_frame_counts':dict(state_frames),'finite_map_target_messages':finite_targets,'rgbd_message_chain_smoke':'PASS' if valid and finite_targets>0 and states['CONFIRMED']>0 and states['TENTATIVE']==0 else 'FAIL_OR_INCOMPLETE','scored_raw_output_frame_count':sum(f['raw_output_present'] for f in selected),'map_target_accuracy':'NOT_MEASURED; localization not accepted; fixed rig frame conversion is not navigation accuracy','competition_R01':'NOT_MEASURED; official 95 percent definition not confirmed','competition_perception_pass':False,'threshold_adjustments_after_capture':0,'topic_counts':dict(counts),'input_hashes':{str(x):hashlib.sha256(x.read_bytes()).hexdigest() for x in bagfiles+[a.annotations]},'truth_source':'independent per-frame visual annotations bound to RGB hashes, fixture geometry and depth visibility checks; not model output','sample_independence':'15 full-positive, 1 transition-positive and 14 background images; correlated repeats, not 30 independent scenes'}
 for name,value in [('summary.json',summary),('raw_frames.json',rawframes),('policy_frames.json',policyframes),('diagnostics.json',diagnostics)]:
  (a.output/name).write_text(json.dumps(value,indent=2,allow_nan=False))
 if len(montage)==30:cv2.imwrite(str(a.output/'contact_sheet.png'),np.vstack([np.hstack(montage[i:i+5]) for i in range(0,30,5)]))
 print(json.dumps({'dataset_valid':valid,'message_chain_smoke':summary['rgbd_message_chain_smoke'],'raw':rawscore['class_metrics'],'policy':policyscore['class_metrics'],'states':dict(states)},indent=2))

if __name__=='__main__':main()
