#!/usr/bin/env python3
"""Predeclared straight-strip diagnostic using physical raster deletion and odometry."""
import argparse,json,math,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool,Empty,Float64MultiArray,String
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from sensor_msgs.msg import JointState

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--mode',choices=['probe','steady'],required=True);a=p.parse_args()
 rclpy.init();n=Node('competition_width_efficiency_estop');rows=[];state={'sim':None,'dirt':None,'gt':None,'permit':False};start=time.monotonic()
 def log(k,d): rows.append({'wall_s':time.monotonic(),'sim_s':state['sim'],'kind':k,'data':d})
 def dirt(m):
  d=json.loads(m.data);state.update(dirt=d,sim=d['sim_time_s']);log('dirt',d)
 def gt(m):
  v=m.twist.twist;p=m.pose.pose.position
  d={'x':p.x,'y':p.y,'linear':math.sqrt(v.linear.x**2+v.linear.y**2+v.linear.z**2),'angular':math.sqrt(v.angular.x**2+v.angular.y**2+v.angular.z**2),'vx':v.linear.x,'wz':v.angular.z,'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9};state['gt']=d;log('gt',d)
 n.create_subscription(String,'/model/tzcup_formal_sanitation_vehicle/ground_dirt/status_json',dirt,50)
 n.create_subscription(Odometry,'/ground_truth/model_odom_raw',gt,qos_profile_sensor_data)
 n.create_subscription(Bool,'/safety/actuators_enabled',lambda m:state.update(permit=m.data),10)
 n.create_subscription(JointState,'/joint_states',lambda m:log('joints',{'name':list(m.name),'position':list(m.position),'velocity':list(m.velocity)}),qos_profile_sensor_data)
 n.create_subscription(String,'/safety/status',lambda m:log('safety',m.data),10)
 specs=[('power',Bool,'/formal_vehicle/simulation/command/main_power'),('estop',Bool,'/formal_vehicle/simulation/command/emergency_stop'),('reset',Bool,'/formal_vehicle/simulation/command/emergency_stop_reset'),('heartbeat',Empty,'/safety/control_heartbeat'),('brush',Float64MultiArray,'/safety/command/brush'),('lift',JointTrajectory,'/cleaning_controller/joint_trajectory'),('enable',Bool,'/model/tzcup_formal_sanitation_vehicle/ground_dirt/command/enable'),('velocity',Twist,'/cmd_vel_gate'),('qual',Bool,'/safety/dry_cleaning_qualification_active'),('dry',Bool,'/brush_enabled')]
 pubs={k:n.create_publisher(t,topic,10) for k,t,topic in specs}
 lift=False;ready_since=None;motion=None;estop=None;last=-1e9;progress=-1e9;first_sim=None
 # Fixed protocol: 5 simulation seconds acceleration, then 10 simulation seconds
 # measurement; no retrospective selection. Probe instead drives for 3 s at .2.
 contract={'mode':a.mode,'grid_m':.1,'cell_area_m2':.01,'acceleration_s':5,'measurement_s':10,'linear_threshold_mps':.01,'angular_threshold_radps':.01,'hold_s':1,'requested_speed_mps':.2 if a.mode=='probe' else 1.,'no_set_pose':True}
 (a.output/'protocol.json').write_text(json.dumps(contract,indent=2))
 try:
  while time.monotonic()-start<900:
   rclpy.spin_once(n,timeout_sec=.005);now=time.monotonic();sim=state['sim'];d=state['dirt'];g=state['gt']
   if sim is not None and first_sim is None:first_sim=sim
   ready=bool(d and all(d[k] for k in ['left_ready','right_ready','roller_ready']))
   if ready and ready_since is None:ready_since=sim;log('all_brushes_ready',d)
   if motion is None and ready_since is not None and ready and sim-ready_since>=1:motion=sim;log('motion_start',g)
   elapsed=sim-motion if motion is not None else 0
   if motion is not None and estop is None and elapsed>=(3 if a.mode=='probe' else 15):
    estop=sim;log('estop_trigger',g)
   if now-last>=.04:
    active=estop is not None
    for k,value in [('power',True),('estop',active),('reset',not active),('enable',True),('qual',ready and not active),('dry',ready and not active)]:pubs[k].publish(Bool(data=value))
    pubs['heartbeat'].publish(Empty());pubs['brush'].publish(Float64MultiArray(data=[8.,-8.,12.] if not active else [0.,0.,0.]))
    cmd=Twist();cmd.linear.x=contract['requested_speed_mps'] if motion is not None and not active else 0.;pubs['velocity'].publish(cmd);last=now
   if not lift and state['permit'] and pubs['lift'].get_subscription_count():
    point=JointTrajectoryPoint(positions=[.1]);point.time_from_start.sec=20;point.time_from_start.nanosec=900000000
    pubs['lift'].publish(JointTrajectory(joint_names=['cleaning_lift_joint'],points=[point]));lift=True;log('lift_requested',.1)
   if now-progress>20:print(json.dumps({'wall_s':now-start,'sim':sim,'ready':ready,'lift':d.get('lift_position_m') if d else None,'clearance':[d.get(k) for k in ['left_clearance_m','right_clearance_m','roller_clearance_m']] if d else None,'motion':motion,'gt':g}),flush=True);progress=now
   if estop is not None and sim-estop>=2 and (a.mode=='steady' or sim-first_sim>=60):break
   if first_sim is not None and motion is None and sim-first_sim>60:log('ready_timeout',d);break
 finally:
  for _ in range(10):
   pubs['estop'].publish(Bool(data=True));pubs['reset'].publish(Bool(data=False));pubs['velocity'].publish(Twist());pubs['brush'].publish(Float64MultiArray(data=[0.,0.,0.]));rclpy.spin_once(n,timeout_sec=.05)
  (a.output/'timeline.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
  summary={'mode':a.mode,'motion_start_sim_s':motion,'estop_trigger_sim_s':estop,'all_brushes_ready':ready_since is not None,'wall_duration_s':time.monotonic()-start,'last_dirt':state['dirt']}
  (a.output/'probe.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary),flush=True);n.destroy_node();rclpy.shutdown()
 return 0 if motion is not None and estop is not None else 1
if __name__=='__main__':raise SystemExit(main())
