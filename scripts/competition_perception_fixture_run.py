#!/usr/bin/env python3
"""Run one predeclared static-camera fixture; no target or truth-driven motion."""
import argparse,json,pathlib,subprocess,time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from cv_bridge import CvBridge
import cv2

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=pathlib.Path,required=True);a=p.parse_args();out=a.output
 rclpy.init();n=rclpy.create_node('public_camera_fixture_controller');state={'sim':None,'rgb':0,'info':None};bridge=CvBridge();events=[]
 def image(m):
  state['rgb']+=1
  if not (out/'preview.png').exists():cv2.imwrite(str(out/'preview.png'),bridge.imgmsg_to_cv2(m,desired_encoding='bgr8'))
 def info(m):
  state['info']={'width':m.width,'height':m.height,'k':list(m.k),'frame':m.header.frame_id}
 n.create_subscription(Clock,'/clock',lambda m:state.update(sim=m.clock.sec+m.clock.nanosec*1e-9),qos_profile_sensor_data)
 n.create_subscription(Image,'/sensors/front_rgbd/depth/image_rect_raw/image',image,qos_profile_sensor_data)
 n.create_subscription(CameraInfo,'/sensors/front_rgbd/depth/image_rect_raw/camera_info',info,qos_profile_sensor_data)
 broadcaster=StaticTransformBroadcaster(n);transforms=[]
 for parent,child,xyz,q in [('map','base_footprint',(0.,0.,0.),(0.,0.,0.,1.)),('base_footprint','front_rgbd_depth_optical_frame',(1.65,0.,2.35),(1.,0.,0.,0.))]:
  t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child;t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=xyz;t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=q;transforms.append(t)
 broadcaster.sendTransform(transforms)
 def service(name,typ,req):
  c=['gz','service','-s','/world/perception_fixture/'+name,'--reqtype',typ,'--reptype','gz.msgs.Boolean','--timeout','5000','--req',req]
  r=subprocess.run(c,capture_output=True,text=True,timeout=8);event={'sim_s':state['sim'],'service':name,'request':req,'rc':r.returncode,'stdout':r.stdout,'stderr':r.stderr};events.append(event)
  if r.returncode!=0 or 'data: true' not in r.stdout:raise RuntimeError('fixture service failed '+json.dumps(event))
 t0=time.monotonic();paused=False;start=None;hidden=False
 try:
  ready_deadline=time.monotonic()+45
  while time.monotonic()<ready_deadline:
   try:
    listed=subprocess.run(['gz','service','-l'],capture_output=True,text=True,timeout=5)
    if '/world/perception_fixture/control' in listed.stdout:break
   except subprocess.TimeoutExpired:pass
   time.sleep(.5)
  else:raise RuntimeError('Gazebo control service not ready')
  service('control','gz.msgs.WorldControl','pause: false')
  while time.monotonic()-t0<600:
   rclpy.spin_once(n,timeout_sec=.01);s=state['sim']
   if s is None:continue
   if not paused and start is None and s>=1. and state['rgb'] and state['info']:
    service('control','gz.msgs.WorldControl','pause: true');paused=True
    (out/'preview-ready.json').write_text(json.dumps({'sim_s':s,'camera':state['info'],'rgb_count':state['rgb']},indent=2))
   if paused and start is None and (out/'capture.go').exists():
    # Drain queued clock callbacks before binding this capture's public start.
    for _ in range(20):rclpy.spin_once(n,timeout_sec=.01)
    start=state['sim'];(out/'capture-start.json').write_text(json.dumps({'sim_s':start,'offsets_s':list(range(2,61,2))},indent=2));service('control','gz.msgs.WorldControl','pause: false')
   if start is not None and not hidden and s-start>=31.:
    for i in range(5):service('set_pose','gz.msgs.Pose',f'name: "sample_{i}" position {{x: {10+i} y: 0 z: 0.5}} orientation {{w: 1}}')
    hidden=True
   if start is not None and s-start>=61.:
    service('control','gz.msgs.WorldControl','pause: true');break
  else:raise RuntimeError('bounded fixture wall timeout')
 finally:
  (out/'fixture-events.json').write_text(json.dumps(events,indent=2));(out/'fixture-summary.json').write_text(json.dumps({'first_capture_sim_s':start,'last_sim_s':state['sim'],'rgb_count':state['rgb'],'garbage_hidden':hidden,'ground_truth_subscribed':False,'scope':'fixed public camera extrinsics; not navigation localization'},indent=2));n.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
