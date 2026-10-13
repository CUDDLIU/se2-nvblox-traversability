#!/usr/bin/env python3
"""Exercise the compiled controller against a 0.5 rad/s actuator dead zone.

Synthetic inputs in an isolated ROS domain; never start an M20 sender.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool

ROOT = Path(__file__).resolve().parents[2]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    if os.environ.get('ROS_DOMAIN_ID', '0') == '0':
        raise SystemExit('Use an isolated ROS_DOMAIN_ID, not the live robot domain')
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init(); node = rclpy.create_node('se2_turn_guard_test')
    odom = node.create_publisher(Odometry, '/test/odom', qos_profile_sensor_data)
    cloud = node.create_publisher(PointCloud2, '/test/cloud', qos_profile_sensor_data)
    ref = node.create_publisher(RosPath, '/se2_scan/reference', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    permit_pub = node.create_publisher(PoseStamped, '/se2_scan/turn_permit', 1)
    alive = node.create_publisher(Bool, '/se2_scan/route_alive', 1)
    commands = []; yaw = -.24; rate = 0.; permit = PoseStamped()
    def command(m):
        nonlocal rate
        rate = m.angular.z
        commands.append((time.monotonic(), m.linear.x, m.linear.y, rate))
    node.create_subscription(Twist, '/se2_scan/cmd_vel', command, qos_profile_sensor_data)
    log = (args.output/'bridge.log').open('w')
    proc = subprocess.Popen(['ros2', 'run', 'se2_scan_bridge', 'scan_bridge', '--ros-args',
        '--params-file', str(ROOT/'scanplanner_integration/planner.yaml'),
        '-r', '__ns:=/se2_scan', '-r', 'raw_cloud:=/test/cloud', '-r', 'raw_body_pose:=/test/odom',
        '-r', 'cloud:=/unused', '-r', 'sensor_pose:=/unused_pose', '-p', 'min_in_place_yaw_speed:=0.5'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    def pump(seconds, simulate=False, pose=True, obstacle=False):
        nonlocal yaw
        end = time.monotonic()+seconds; previous=time.monotonic(); last_cloud=0.
        while time.monotonic() < end:
            now=time.monotonic(); dt=now-previous; previous=now
            # Model the documented M20 standard/basic gait dead zone.
            if simulate and abs(rate)>=.5: yaw += rate*dt
            alive.publish(Bool(data=True)); permit_pub.publish(permit)
            if pose:
                m=Odometry(); m.header.frame_id='nvblox_odom'; m.child_frame_id='base_link_dog'
                m.header.stamp=node.get_clock().now().to_msg()
                m.pose.pose.orientation.z=math.sin(yaw/2); m.pose.pose.orientation.w=math.cos(yaw/2)
                odom.publish(m)
            if now-last_cloud>.1:
                last_cloud=now; stamp=node.get_clock().now().nanoseconds/1e9
                m=PointCloud2(); m.header.frame_id='lidar_link'; m.header.stamp=node.get_clock().now().to_msg()
                points=[(x*.1, y*.1, -.57, stamp) for x in range(-25,26) for y in range(-25,26)]
                # Place returns outside the robot self-filter (|x| < .41),
                # but inside the voxel-padded rotating torso boundary.
                if obstacle: points += [(.43, y*.03, z*.04, stamp) for y in range(-5,6) for z in range(0,7)]
                m.height=1; m.width=len(points); m.point_step=20; m.row_step=20*len(points); m.is_dense=True
                m.fields=[PointField(name=k,offset=i*4,datatype=7,count=1) for i,k in enumerate(['x','y','z'])]+[PointField(name='timestamp',offset=12,datatype=8,count=1)]
                m.data=b''.join(struct.pack('<fffd',*p) for p in points); cloud.publish(m)
            rclpy.spin_once(node, timeout_sec=.005)
    def start_turn(initial):
        nonlocal yaw, rate, permit
        yaw=initial; rate=0.
        path=RosPath(); path.header.frame_id='nvblox_odom'; path.header.stamp=node.get_clock().now().to_msg()
        for angle in (initial, 0.):
            p=PoseStamped(); p.header=path.header
            p.pose.orientation.z=math.sin(angle/2); p.pose.orientation.w=math.cos(angle/2); path.poses.append(p)
        ref.publish(path); permit=path.poses[-1]
    result={}
    try:
        until=time.monotonic()+15
        while time.monotonic()<until and not cloud.get_subscription_count(): rclpy.spin_once(node, timeout_sec=.05)
        assert cloud.get_subscription_count(), 'Bridge did not start'
        pump(1.)
        for initial in (-.24, .24):
            start_turn(initial); began=time.monotonic(); pump(1.8, simulate=True)
            recent=[v for v in commands if v[0]>began+.2]
            moving=[v[3] for v in recent if abs(v[3])>1e-6]
            assert moving and all(.5<=abs(v)<=.6 for v in moving), moving
            assert abs(yaw)<.12, ('Could not leave original turn segment', yaw)
            assert all(abs(v[1])+abs(v[2])<1e-6 for v in recent), 'Pure turn produced translation'
            assert all(abs(v[3])<1e-6 for v in recent[-4:]), 'Did not stop inside yaw tolerance'
            result[str(initial)]=dict(final_yaw=yaw, min_rate=min(map(abs,moving)), max_rate=max(map(abs,moving)))
        start_turn(-.24); pump(.5); began=time.monotonic(); pump(.6, pose=False)
        assert all(max(map(abs,v[1:]))<1e-6 for v in commands if v[0]>began+.35), 'Stale pose did not stop turn'
        empty=RosPath(); empty.header.frame_id='nvblox_odom'; ref.publish(empty); pump(.3)
        assert all(max(map(abs,v[1:]))<1e-6 for v in commands[-5:]), 'Cancel did not stop turn'
        start_turn(-.24); pump(.6); began=time.monotonic(); pump(.6, obstacle=True)
        recent=[v for v in commands if v[0]>began+.25]
        assert recent and all(max(map(abs,v[1:]))<1e-6 for v in recent), 'Obstacle did not stop turn'
        result.update(state='passed', checks=['positive/negative dead-zone convergence', 'zero inside tolerance', 'no translation', 'stale pose', 'cancel', 'new obstacle'])
    finally:
        os.killpg(proc.pid,signal.SIGINT)
        try: proc.wait(timeout=8)
        except subprocess.TimeoutExpired: os.killpg(proc.pid,signal.SIGKILL); proc.wait()
        (args.output/'turn_guard.json').write_text(json.dumps(result,indent=2)); log.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    print(json.dumps(result), flush=True)


if __name__ == '__main__': main()
