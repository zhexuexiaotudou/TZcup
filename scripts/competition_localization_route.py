#!/usr/bin/env python3
"""One public-map Nav2 route; ground truth is recorded only by the external bag."""
import argparse,json,signal,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from nav2_msgs.action import NavigateToPose
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool,Empty
from geometry_msgs.msg import Twist

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--seconds',type=float,default=700);p.add_argument('--prepare-seconds',type=float,default=0);p.add_argument('--goal-x',type=float,default=6);p.add_argument('--estop-distance',type=float,default=0);a=p.parse_args();assert a.estop_distance==0
 rclpy.init(signal_handler_options=SignalHandlerOptions.NO);n=Node('competition_localization_route');state={'sim':None,'permit':False,'stop':False};events=[];tstart=time.monotonic();first=None
 def log(kind,data):events.append({'wall_s':time.monotonic(),'sim_s':state['sim'],'kind':kind,'data':data})
 def sig(*_):state['stop']=True
 signal.signal(signal.SIGINT,sig);signal.signal(signal.SIGTERM,sig)
 n.create_subscription(Clock,'/clock',lambda m:state.update(sim=m.clock.sec+m.clock.nanosec*1e-9),qos_profile_sensor_data)
 n.create_subscription(Bool,'/safety/actuators_enabled',lambda m:state.update(permit=m.data),10)
 pubs={k:n.create_publisher(t,topic,10) for k,t,topic in [('power',Bool,'/formal_vehicle/simulation/command/main_power'),('estop',Bool,'/formal_vehicle/simulation/command/emergency_stop'),('reset',Bool,'/formal_vehicle/simulation/command/emergency_stop_reset'),('heartbeat',Empty,'/safety/control_heartbeat')]}
 zero=n.create_publisher(Twist,'/cmd_vel_gate',10);action=ActionClient(n,NavigateToPose,'/navigate_to_pose');future=None;result=None;handle=None;stage=0;results=[];last=progress=-1e9
 (a.output/'route_protocol.json').write_text(json.dumps({'scope':'localization only','public_map_goals_x_m':[a.goal_x,0.],'return_not_before_sim_s':40.,'observation_duration_sim_s':60.,'ground_truth_subscribed_by_driver':False,'brush_commands':False,'estop_test':False},indent=2))
 try:
  while not state['stop'] and time.monotonic()-tstart<a.seconds:
   rclpy.spin_once(n,timeout_sec=.005);now=time.monotonic();sim=state['sim']
   if sim is not None and first is None:first=sim
   if now-last>=.04:
    for k,v in [('power',True),('estop',False),('reset',True)]:pubs[k].publish(Bool(data=v))
    pubs['heartbeat'].publish(Empty())
    if zero is not None:zero.publish(Twist())
    last=now
   if sim is not None and state['permit'] and action.server_is_ready() and future is None and (stage==0 or (stage==1 and sim>=40.)):
    goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp.sec=int(sim);goal.pose.header.stamp.nanosec=int((sim-int(sim))*1e9);goal.pose.pose.position.x=a.goal_x if stage==0 else 0.;goal.pose.pose.orientation.w=1.
    future=action.send_goal_async(goal);log('goal_sent',{'stage':stage,'x':goal.pose.pose.position.x})
   if future is not None and future.done() and handle is None:
    handle=future.result();log('goal_accepted',handle.accepted)
    if not handle.accepted:break
    if zero is not None:n.destroy_publisher(zero);zero=None
    result=handle.get_result_async()
   if result is not None and result.done():
    status=result.result().status;results.append(status);log('goal_result',{'stage':stage,'status':status});stage+=1;future=result=handle=None
    zero=n.create_publisher(Twist,'/cmd_vel_gate',10)
    if status!=4:break
   if first is not None and sim-first>=60 and stage>=2:break
   if now-progress>=20:print(json.dumps({'wall_s':now-tstart,'sim_s':sim,'permit':state['permit'],'stage':stage,'results':results}),flush=True);progress=now
 finally:
  if handle is not None and handle.accepted:handle.cancel_goal_async()
  if zero is None:zero=n.create_publisher(Twist,'/cmd_vel_gate',10)
  for _ in range(10):zero.publish(Twist());rclpy.spin_once(n,timeout_sec=.03)
  summary={'nav_results':results,'passed':results==[4,4],'first_sim_s':first,'last_sim_s':state['sim'],'wall_duration_s':time.monotonic()-tstart,'ground_truth_used_for_control':False,'stopped_by_signal':state['stop']}
  (a.output/'route.json').write_text(json.dumps(summary,indent=2));(a.output/'route_timeline.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in events));print(json.dumps(summary),flush=True);n.destroy_node();rclpy.shutdown()
 return 0 if summary['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
