#!/usr/bin/env python3
"""Generate a small camera fixture and evaluator-only geometry; no model truth input."""
import argparse,json,math,xml.etree.ElementTree as ET
from pathlib import Path
from competition_perception_model_profile import (
    CONTROLLED_FIXTURE_PROFILE,
    resolve_model_profile,
)

def create(source,out):
 source=Path(source).resolve()
 out.mkdir(parents=True,exist_ok=False)
 root=ET.Element('sdf',version='1.9');world=ET.SubElement(root,'world',name='perception_fixture')
 def xml(parent,text):parent.append(ET.fromstring(text))
 xml(world,'<physics name="default" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>0.25</real_time_factor></physics>')
 for filename,name in [('Physics','Physics'),('UserCommands','UserCommands'),('SceneBroadcaster','SceneBroadcaster'),('Sensors','Sensors')]:
  library={'Physics':'physics','UserCommands':'user-commands','SceneBroadcaster':'scene-broadcaster','Sensors':'sensors'}[filename]
  xml(world,f'<plugin filename="gz-sim-{library}-system" name="gz::sim::systems::{name}">'+('<render_engine>ogre2</render_engine>' if name=='Sensors' else '')+'</plugin>')
 xml(world,'<scene><ambient>0.7 0.7 0.7 1</ambient><background>0.6 0.7 0.8 1</background><shadows>false</shadows></scene>')
 xml(world,'<light type="directional" name="sun"><pose>0 0 10 0 0 0</pose><diffuse>0.8 0.8 0.8 1</diffuse><direction>0 0 -1</direction><cast_shadows>false</cast_shadows></light>')
 xml(world,'<model name="ground"><static>true</static><link name="link"><visual name="visual"><geometry><plane><normal>0 0 1</normal><size>8 8</size></plane></geometry><material><ambient>0.35 0.35 0.35 1</ambient><diffuse>0.35 0.35 0.35 1</diffuse></material></visual></link></model>')
 camxyz=[1.65,0.,2.35]
 xml(world,f'<model name="camera_rig"><static>true</static><pose>1.65 0 2.35 0 {math.pi/2} {math.pi/2}</pose><link name="camera"><sensor name="rgbd" type="rgbd_camera"><topic>/sensors/front_rgbd/depth/image_rect_raw</topic><gz_frame_id>front_rgbd_depth_optical_frame</gz_frame_id><update_rate>2</update_rate><always_on>true</always_on><camera><horizontal_fov>1.2</horizontal_fov><image><width>848</width><height>480</height><format>R8G8B8</format></image><clip><near>0.1</near><far>10</far></clip></camera></sensor></link></model>')
 specs=[('trash_bottle','plastic_bottle',[.70,.12,.12],[.08,.08,.24],'cylinder'),('trash_can','metal_can',[1.45,.10,.06],[.07,.07,.12],'cylinder'),('trash_paper','paper_litter',[1.02,.46,.005],[.20,.14,.01],'box'),('leaf_pile','leaf_pile',[1.02,-.38,.01],[.68,.68,.02],'cylinder')]
 objects=[]
 for i,(asset,cl,pose,size,shape) in enumerate(specs):
  model=ET.parse(source/'starter_ws/src/sanitation_worlds/models'/asset/'model.sdf').getroot().find('model');model.set('name',f'sample_{i}');ET.SubElement(model,'pose').text=' '.join(map(str,pose))+' 0 0 0';world.append(model)
  objects.append({'object_id':f'sample_{i}','class_id':cl,'center':pose,'size':size,'shape':shape})
 xml(world,'<model name="sample_4"><static>true</static><pose>2.25 0.22 0.005 0 0 0</pose><link name="link"><visual name="visual"><geometry><box><size>1.2 0.75 0.01</size></box></geometry><material><ambient>0.05 0.25 0.55 1</ambient><diffuse>0.05 0.35 0.75 1</diffuse></material></visual></link></model>')
 objects.append({'object_id':'sample_4','class_id':'puddle','center':[2.25,.22,.005],'size':[1.2,.75,.01],'shape':'box'})
 xml(world,'<model name="negative_box"><static>true</static><pose>1.62 -0.55 0.2 0 0 0</pose><link name="link"><visual name="visual"><geometry><box><size>0.22 0.22 0.4</size></box></geometry><material><ambient>0.4 0.25 0.12 1</ambient><diffuse>0.4 0.25 0.12 1</diffuse></material></visual></link></model>')
 xml(world,'<model name="negative_paint"><static>true</static><pose>2.55 -0.55 0.002 0 0 0</pose><link name="link"><visual name="visual"><geometry><box><size>0.2 0.12 0.004</size></box></geometry><material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material></visual></link></model>')
 ET.indent(root);ET.ElementTree(root).write(out/'world.sdf',encoding='utf-8',xml_declaration=True)
 profile=resolve_model_profile(source,CONTROLLED_FIXTURE_PROFILE)
 plan={'scope':'controlled primitive-asset RGB-D perception; not real-world recognition acceptance','control_use_prohibited':True,'model_profile':{'id':profile.profile_id,'path':str(profile.path.relative_to(source)).replace('\\','/'),'sha256':profile.sha256,'scope':profile.scope},'camera_map_xyz':camxyz,'camera_map_optical_quaternion_xyzw':[1.,0.,0.,0.],'map_base_transform':'identity static calibrated fixture; no localization','objects':objects,'negative_objects':['negative_box','negative_paint','ground'],'positive_score_offsets_s':list(range(2,31,2)),'hide_garbage_at_offset_s':31.,'negative_score_offsets_s':list(range(32,61,2)),'end_offset_s':61.,'warmup_pause_sim_s':1.,'frame_selection':'nearest original RGB stamp to start+2k, k=1..30, within 0.26 s; missing outputs count as empty predictions','thresholds':'all five classes 0.8 fixed before capture; no holdout tuning','roi':[.4,3.,-.8,.8,-.08,.30],'annotation':'project fixture geometry, not model predictions; cylinders use dense silhouette vertices; verify visible first frame before starting score interval','statistical_boundary':'15 repeated positive views and 15 background views are correlated, not independent population samples'}
 (out/'evaluation_plan.json').write_text(json.dumps(plan,indent=2));return plan

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();create(a.source,a.output)
