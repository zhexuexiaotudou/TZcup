#!/usr/bin/env python3
"""Bounded competition probe: real Nav2 goal plus safety-gated physical brush commands."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TwistStamped
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, Empty, Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=120)
    parser.add_argument('--prepare-seconds', type=float, default=240)
    parser.add_argument('--goal-x', type=float, default=3)
    parser.add_argument('--estop-distance', type=float, default=0)
    args = parser.parse_args()
    rclpy.init()
    node = Node('competition_motion_cleaning_probe')
    pubs = {key: node.create_publisher(kind, topic, 10) for key, kind, topic in [
        ('power', Bool, '/formal_vehicle/simulation/command/main_power'),
        ('estop', Bool, '/formal_vehicle/simulation/command/emergency_stop'),
        ('reset', Bool, '/formal_vehicle/simulation/command/emergency_stop_reset'),
        ('heartbeat', Empty, '/safety/control_heartbeat'),
        ('brush', Float64MultiArray, '/safety/command/brush'),
        ('lift', JointTrajectory, '/cleaning_controller/joint_trajectory'),
        ('enable', Bool, '/model/tzcup_formal_sanitation_vehicle/ground_dirt/command/enable'),
    ]}
    rows = []
    state = {'sim_s': None, 'gt': [], 'dirt': [], 'maxima': {}, 'goal_accepted': False, 'permitted': False}
    def record(kind, data):
        rows.append({'wall_s': time.monotonic(), 'sim_s': state['sim_s'], 'kind': kind, 'data': data})
    def clock(msg):
        state['sim_s'] = msg.clock.sec + msg.clock.nanosec * 1e-9
    def gt(msg):
        p = msg.pose.pose.position
        data = [p.x, p.y, msg.twist.twist.linear.x]
        state['gt'].append(data)
        record('gt', data)
    def dirt(msg):
        data = json.loads(msg.data)
        state['dirt'].append(data)
        record('dirt', data)
    def velocity(topic, msg):
        v = msg.twist if isinstance(msg, TwistStamped) else msg
        state['maxima'][topic] = max(state['maxima'].get(topic, 0), abs(v.linear.x))
        record(topic, [v.linear.x, v.angular.z])
    node.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data)
    node.create_subscription(Bool, '/safety/actuators_enabled', lambda msg: state.update(permitted=msg.data), 10)
    node.create_subscription(Odometry, '/ground_truth/model_odom_raw', gt, qos_profile_sensor_data)
    node.create_subscription(String, '/model/tzcup_formal_sanitation_vehicle/ground_dirt/status_json', dirt, 50)
    for topic in ['/cmd_vel_nav', '/cmd_vel_smoothed', '/cmd_vel_gate', '/base_controller/cmd_vel']:
        node.create_subscription(TwistStamped if topic == '/base_controller/cmd_vel' else Twist,
                                 topic, lambda msg, t=topic: velocity(t, msg), 50)
    action = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    # Stationary actuator preparation needs a fresh zero command at the safety
    # input. Destroy this publisher before submitting the first Nav2 goal.
    preparation_zero = node.create_publisher(Twist, '/cmd_vel_gate', 10)
    goal_future = None
    result_future = None
    start = time.monotonic()
    motion_start = None
    moving_estop_started = None
    last_command = last_lift = last_progress = -1e6
    try:
        while time.monotonic() - start < args.prepare_seconds + args.seconds:
            now = time.monotonic()
            if (args.estop_distance > 0 and moving_estop_started is None and motion_start is not None
                    and len(state['gt']) > 1 and abs(state['gt'][-1][2]) > .05
                    and math.hypot(state['gt'][-1][0]-state['gt'][0][0], state['gt'][-1][1]-state['gt'][0][1]) >= args.estop_distance):
                moving_estop_started = now
                record('moving_estop_requested', state['gt'][-1])
            estop_active = moving_estop_started is not None and now-moving_estop_started < 5
            if now - last_progress >= 15:
                print(json.dumps({'elapsed_wall_s': now-start, 'sim_s': state['sim_s'],
                                  'permitted': state['permitted'],
                                  'dirt': state['dirt'][-1] if state['dirt'] else None}), flush=True)
                last_progress = now
            if motion_start is not None and now - motion_start >= args.seconds:
                break
            if now - last_command >= 0.05:
                pubs['power'].publish(Bool(data=True))
                pubs['estop'].publish(Bool(data=estop_active))
                pubs['reset'].publish(Bool(data=not estop_active))
                pubs['heartbeat'].publish(Empty())
                pubs['enable'].publish(Bool(data=True))
                pubs['brush'].publish(Float64MultiArray(data=[8., -8., 12.]))
                if preparation_zero is not None:
                    preparation_zero.publish(Twist())
                last_command = now
            if last_lift < 0 and state['permitted'] and pubs['lift'].get_subscription_count():
                trajectory = JointTrajectory(joint_names=['cleaning_lift_joint'])
                point = JointTrajectoryPoint(positions=[0.10])
                # 100 mm at the published 4.8 mm/s P16 rating: 20.9 s.
                point.time_from_start.sec = 20
                point.time_from_start.nanosec = 900000000
                trajectory.points = [point]
                pubs['lift'].publish(trajectory)
                last_lift = now
            ready = bool(state['dirt'] and state['dirt'][-1].get('roller_ready'))
            if motion_start is None and ready:
                motion_start = now
                record('work_ready', state['dirt'][-1])
                node.destroy_publisher(preparation_zero)
                preparation_zero = None
            if motion_start is None and now - start >= args.prepare_seconds:
                record('preparation_timeout', state['dirt'][-1] if state['dirt'] else None)
                break
            if goal_future is None and motion_start is not None and action.server_is_ready():
                goal = NavigateToPose.Goal()
                goal.pose.header.frame_id = 'map'
                goal.pose.pose.position.x = args.goal_x
                goal.pose.pose.orientation.w = 1.0
                goal_future = action.send_goal_async(goal)
            if goal_future is not None and goal_future.done() and result_future is None:
                handle = goal_future.result()
                state['goal_accepted'] = handle.accepted
                record('goal_accepted', handle.accepted)
                if handle.accepted:
                    result_future = handle.get_result_async()
            rclpy.spin_once(node, timeout_sec=0.01)
    finally:
        if goal_future is not None and goal_future.done() and goal_future.result().accepted:
            goal_future.result().cancel_goal_async()
        for _ in range(10):
            pubs['brush'].publish(Float64MultiArray(data=[0., 0., 0.]))
            pubs['estop'].publish(Bool(data=True))
            pubs['reset'].publish(Bool(data=False))
            rclpy.spin_once(node, timeout_sec=0.05)
        gt_rows = state['gt']
        displacement = math.hypot(gt_rows[-1][0]-gt_rows[0][0], gt_rows[-1][1]-gt_rows[0][1]) if len(gt_rows)>1 else 0
        roller = max((abs(d.get('roller_velocity_rad_s', 0)) for d in state['dirt']), default=0)
        ready = any(d.get('roller_ready') for d in state['dirt'])
        result = {'scope': 'competition diagnostic, not formal acceptance',
                  'goal_accepted': state['goal_accepted'], 'max_linear_mps': state['maxima'],
                  'nav_result_status': result_future.result().status if result_future is not None and result_future.done() else None,
                  'moving_estop_requested': moving_estop_started is not None,
                  'gt_samples': len(gt_rows), 'gt_displacement_m': displacement,
                  'roller_velocity_max_rad_s': roller, 'roller_ready': ready,
                  'dirt_samples': len(state['dirt']), 'last_dirt': state['dirt'][-1] if state['dirt'] else None,
                  'preparation_wall_s': (motion_start or time.monotonic()) - start,
                  'motion_window_wall_s': time.monotonic() - motion_start if motion_start else 0,
                  'passed': displacement > 2 and ready and roller > 0 and state['maxima'].get('/base_controller/cmd_vel', 0)>0}
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output/'probe.json').write_text(json.dumps(result, indent=2)+'\n')
        (args.output/'timeline.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows))
        print(json.dumps(result, indent=2), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
