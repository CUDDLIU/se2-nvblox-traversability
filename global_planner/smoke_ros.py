#!/usr/bin/env python3
"""Exercise the deployed point-goal interface; requires a running ROS viewer."""
import argparse
import json
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--goal', type=float, nargs=3, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('se2_global_interface_smoke')
    retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                          reliability=ReliabilityPolicy.RELIABLE)
    state, paths = [], []
    node.create_subscription(String, '/se2_global/status', lambda m: state.append(json.loads(m.data)), retained)
    node.create_subscription(Path, '/se2_global/path', paths.append, retained)
    pub = node.create_publisher(PointStamped, '/clicked_point', 10)

    def wait(condition, timeout=130.):
        until = time.monotonic()+timeout
        while not condition() and time.monotonic()<until:
            rclpy.spin_once(node, timeout_sec=.1)
        if not condition():
            raise AssertionError('ROS response timed out: '+str(state[-1:] or 'no status'))

    def query(xyz, expected):
        generation = state[-1]['generation']
        paths.clear()
        msg = PointStamped()
        msg.header.frame_id = state[-1]['frame']
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.point.x, msg.point.y, msg.point.z = map(float, xyz)
        pub.publish(msg)
        wait(lambda: state[-1]['generation']>generation and state[-1]['state'] in ('success','no_path'))
        assert state[-1]['state']==expected, state[-1]
        wait(lambda: bool(paths) and bool(paths[-1].poses)==(expected=='success'))
        print(json.dumps(state[-1], ensure_ascii=False), flush=True)
        if expected=='success':
            result = state[-1]['result']
            assert len(paths[-1].poses)==len(result.get('poses', [])) or len(paths[-1].poses)==result['motion_segments_verified']+1
            assert abs(paths[-1].poses[-1].pose.position.z-xyz[2])<.26
            assert result['voxel_certified_segments']+result['metric_certified_segments']==result['motion_segments_verified']

    try:
        wait(lambda: bool(state) and pub.get_subscription_count()>0, 60.)
        query(args.goal, 'success')
        query([args.goal[0], args.goal[1], 100.], 'no_path')
        query(args.goal, 'success')
        print('ROS_SMOKE_PASS: 3D goal, verified Path, invalid-goal clearing, repeated query', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
