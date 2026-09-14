#!/usr/bin/env python3
"""Observe live camera-to-target transport without subscribing to any truth topic."""
import argparse
import json
import math
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from sanitation_perception_interfaces.msg import GarbageTargetArray


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=90)
    # Shared competition shell passes motion options; this observer remains
    # strictly read-only and never publishes a motion or actuator command.
    parser.add_argument('--prepare-seconds',type=float,default=0)
    parser.add_argument('--goal-x',type=float,default=0)
    parser.add_argument('--estop-distance',type=float,default=0)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=Node('competition_camera_perception_observer')
    counts={'rgb':0,'depth':0,'info':0,'target_messages':0}
    targets=[];diagnostics=[];metadata={}
    def image(kind,msg):
        counts[kind]+=1
        if kind not in metadata:
            metadata[kind]={'stamp_sec':msg.header.stamp.sec,'stamp_nanosec':msg.header.stamp.nanosec,
                            'frame':msg.header.frame_id,'width':msg.width,'height':msg.height,'encoding':msg.encoding}
            (args.output/(kind+'.raw')).write_bytes(bytes(msg.data))
            metadata[kind]['step']=msg.step
    def info(msg):
        counts['info']+=1
        metadata['camera_info']={'frame':msg.header.frame_id,'width':msg.width,'height':msg.height,'k':list(msg.k)}
    def detected(msg):
        counts['target_messages']+=1
        for target in msg.targets:
            p=target.map_pose.pose.position
            if len(targets)<30:
                targets.append({'class_id':target.class_id,'confidence':float(target.confidence),
                                'frame':msg.header.frame_id,'x_m':p.x,'y_m':p.y,'z_m':p.z,
                                'stamp_sec':msg.header.stamp.sec,'stamp_nanosec':msg.header.stamp.nanosec})
    root='/sensors/front_rgbd/depth/image_rect_raw'
    node.create_subscription(Image,root+'/image',lambda msg:image('rgb',msg),qos_profile_sensor_data)
    node.create_subscription(Image,root+'/depth_image',lambda msg:image('depth',msg),qos_profile_sensor_data)
    node.create_subscription(CameraInfo,root+'/camera_info',info,qos_profile_sensor_data)
    node.create_subscription(GarbageTargetArray,'/perception/garbage/targets',detected,10)
    node.create_subscription(String,'/perception/garbage/diagnostics',lambda msg:diagnostics.append(msg.data),10)
    start=time.monotonic()
    while time.monotonic()-start<args.seconds:
        rclpy.spin_once(node,timeout_sec=.1)
    try:
        inputs=node.get_subscriber_names_and_types_by_node('competition_development_perception','/')
    except Exception as exc:
        inputs=[];metadata['graph_error']=str(exc)
    no_truth=bool(inputs) and not any(any(word in topic for word in ('ground_truth','evaluator','evaluation/')) for topic,_ in inputs)
    valid=[t for t in targets if t['class_id'] and t['frame']=='map' and all(math.isfinite(t[k]) for k in ('x_m','y_m','z_m'))]
    result={'scope':'development model camera-to-class-and-position smoke, no accuracy claim',
            'counts':counts,'targets':targets,'metadata':metadata,'product_input_topics':inputs,
            'no_truth_input':no_truth,'last_diagnostics':diagnostics[-3:],
            'passed':all(counts[k]>0 for k in ('rgb','depth','info')) and bool(valid) and no_truth}
    (args.output/'perception_smoke.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2),flush=True)
    node.destroy_node();rclpy.shutdown()
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
