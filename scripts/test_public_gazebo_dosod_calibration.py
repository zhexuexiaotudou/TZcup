from __future__ import annotations
import importlib.util, json, subprocess, sys
from pathlib import Path
import numpy as np
import pytest

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location("subject",HERE/"public_gazebo_dosod_calibration.py"); subject=importlib.util.module_from_spec(spec); assert spec.loader; sys.modules["subject"]=subject; spec.loader.exec_module(subject)

def contract(): return {"vocabulary":{"semantic_class_ids":list(subject.CLASS_IDS)},"preprocessing":{"source_color_space":"RGB","resize":{"height":640,"width":640,"interpolation":"bilinear"},"tensor_dtype":"float32","tensor_layout":"NCHW","tensor_shape":[1,3,640,640],"value_range":[0.,1.]},"calibration":{"minimum_sample_count":500,"manifest_name":"calibration_manifest.json"}}
def plan(): return {"source_domain":"public_gazebo_sensor","class_ids":list(subject.CLASS_IDS),"scene_groups":{"calibration":["c"],"holdout":["h"]}}
def frame(scene, n=1, topic='/camera/color/image_raw', selector=False):
    payload=bytes([n%255,2,3])*4
    suffix=(str(n % 10) * 32, "a" * 64) if selector else ("", "")
    return subject.Frame(scene,topic,'front',n,payload,2,2,6,'rgb8',{"frame_id":"front","stamp_ns":n,"width":2,"height":2,"k":[1.,0.,1.,0.,1.,1.,0.,0.,1.]},*suffix)
def selection(nonce='1'):
    return {'state':'ACTIVE','scene_id':'map-0-mission-0','generation_nonce':nonce*32,'episode_manifest_sha256':'a'*64}
def test_plan_rejects_overlap(tmp_path):
    p=plan();p['scene_groups']['holdout']=['c']; f=tmp_path/'p.json';f.write_text(json.dumps(p));
    with pytest.raises(subject.CalibrationRejected,match='disjoint'): subject.load_scene_plan(f)
def test_store_records_public_hashes_and_rejects_truth(tmp_path):
    s=subject.PublicGazeboStore(tmp_path/'out',contract(),plan()); assert s.add(frame('c')); assert not s.add(frame('h',2))
    row=s.records[0]; assert row['source_role']=='calibration_only'; assert (s.output/row['provenance']).is_file(); assert len(row['sha256'])==64
    with pytest.raises(subject.CalibrationRejected,match='public_plan'): s.add(frame('c',3,'/ground_truth/image'))
def test_camera_mismatch_and_fresh_output_fail_closed(tmp_path):
    s=subject.PublicGazeboStore(tmp_path/'out',contract(),plan()); bad=frame('c'); bad=subject.Frame(bad.scene_id,bad.topic,bad.frame_id,bad.stamp_ns,bad.data,bad.width,bad.height,bad.step,bad.encoding,{"frame_id":"other","stamp_ns":1,"width":2,"height":2,"k":[1.,0.,1.,0.,1.,1.,0.,0.,1.]})
    with pytest.raises(subject.CalibrationRejected,match='camera_info'): s.add(bad)
    with pytest.raises(subject.CalibrationRejected,match='fresh'): subject.PublicGazeboStore(s.output,contract(),plan())
def test_freeze_requires_500_and_100_holdout(tmp_path):
    s=subject.PublicGazeboStore(tmp_path/'out',contract(),plan())
    with pytest.raises(subject.CalibrationRejected,match='below_minimum'): s.freeze()
def test_capacity_rejects_before_live_collection():
    with pytest.raises(subject.CalibrationRejected,match='calibration_scene_plan_capacity'):
        subject.require_collection_capacity(plan=plan(),contract=contract(),per_scene_quota=1)
def test_per_scene_quota_and_frozen_selector_provenance(tmp_path, monkeypatch):
    p={"source_domain":"public_gazebo_sensor","class_ids":list(subject.CLASS_IDS),"scene_groups":{"calibration":["c"],"holdout":["h"]}}
    c=contract(); c["calibration"]["minimum_sample_count"]=1
    monkeypatch.setattr(subject,"MIN_HOLDOUT_SAMPLES",1)
    s=subject.PublicGazeboStore(tmp_path/'out',c,p,per_scene_quota=1)
    assert s.add(frame('c',1,selector=True))
    assert not s.add(frame('c',2,selector=True))
    assert not s.add(frame('h',3,selector=True))
    manifest=json.loads(s.freeze().read_text())
    assert manifest['evaluation_holdout_scene_ids']==['h']
    assert manifest['records'][0]['generation_nonce']=='1'*32
    assert manifest['holdout_records'][0]['episode_manifest_sha256']=='a'*64
    assert (s.output/manifest['holdout_records'][0]['relative_path']).is_file()
def test_cli_is_preflight_only(tmp_path):
    p=tmp_path/'plan.json'; p.write_text(json.dumps(plan())); c=tmp_path/'contract.json'; c.write_text(json.dumps(contract()))
    out=subprocess.check_output([sys.executable,str(HERE/'public_gazebo_dosod_calibration.py'),'--scene-plan',str(p),'--contract',str(c)],text=True)
    value=json.loads(out); assert value['status']=='NON_FORMAL_PREPARED' and value['formal_passed'] is False
def test_ros_pair_conversion_is_exact():
    class Stamp: sec=2; nanosec=3
    class Header: frame_id='front'; stamp=Stamp()
    class Image: header=Header(); data=bytes([1,2,3])*4; width=2; height=2; step=6; encoding='rgb8'
    class Info: header=Header(); width=2; height=2; k=[1.,0.,1.,0.,1.,1.,0.,0.,1.]
    row=subject.frame_from_ros(scene_id='c',topic='/camera/color/image_raw',image=Image(),camera_info=Info())
    assert row.stamp_ns==2_000_000_003 and row.camera['frame_id']=='front'
def test_selector_atomic_nonlink_and_schema(tmp_path):
    p=tmp_path/'selector.json'; value={'state':'ACTIVE','scene_id':'c','episode_manifest_sha256':'a'*64,'generation_nonce':'1'*32}
    subject.atomic_scene_selector(p,value); assert subject.load_scene_selector(p)==value
    value['generation_nonce']='2'*32; subject.atomic_scene_selector(p,value); assert subject.load_scene_selector(p)==value
def test_selector_binds_exact_manifest_and_rejects_bad_nonce(tmp_path):
    manifest=tmp_path/'episode.json'; manifest.write_text('{"public":true}\n')
    selector=tmp_path/'selector.json'
    value=subject.select_scene_from_manifest(selector=selector,scene_id='map-0-mission-1',episode_manifest=manifest)
    assert value['episode_manifest_sha256']==subject.sha256_file(manifest)
    selector.write_text(json.dumps({**value,'generation_nonce':'wrong'}))
    with pytest.raises(subject.CalibrationRejected,match='selector_invalid'): subject.load_scene_selector(selector)
def test_inactive_selector_drops_all_pending_pairs(tmp_path):
    selector=tmp_path/'selector.json'; value=subject.deactivate_scene_selector(selector)
    assert subject.load_scene_selector(selector)==value
def test_explicit_formal_camera_alias_pair_is_required():
    subject.require_formal_camera_topic_pair(image_topic='/camera/color/image_raw',camera_info_topic='/camera/color/camera_info')
    with pytest.raises(subject.CalibrationRejected,match='pair_not_authorized'):
        subject.require_formal_camera_topic_pair(image_topic='/camera/color/image_raw',camera_info_topic='/camera/color/image_raw/camera_info')
def test_pair_cache_accepts_both_orders_and_evicts_flooded_unmatched_frames():
    cache=subject.FreshPairCache(limit=2); a=selection('1')
    assert cache.put_image(('front',1),a,'image') is None
    pair=cache.put_info(('front',1),a,'info'); assert pair==(a,'image','info')
    assert cache.put_info(('front',2),a,'info2') is None
    assert cache.put_image(('front',2),a,'image2')==(a,'image2','info2')
    for stamp in range(10,14): assert cache.put_image(('front',stamp),a,stamp) is None
    assert len(cache.images)==2 and len(cache.infos)==0
    b=selection('2'); assert cache.put_info(('front',14),b,'info') is None
    assert cache.put_image(('front',14),a,'stale') is None
