"""Collect a bounded live smoke test without commanding the robot."""
import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=20.)
    parser.add_argument('--output', default='/tmp/se2-paper-live.json')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('se2_paper_live_audit')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    report = {'status_samples': [], 'map_versions': [], 'marker_samples': []}
    latest = {}

    def status(message):
        value = json.loads(message.data)
        report['status_samples'].append(value)

    def navmesh(message):
        value = json.loads(message.data)
        if 'polygons' not in value:
            return
        version = [value.get('generation'), value.get('revision')]
        if not report['map_versions'] or report['map_versions'][-1]['version'] != version:
            report['map_versions'].append(dict(version=version, valid=value.get('valid'),
                updating=value.get('updating'), polygons=len(value['polygons']),
                graph_nodes=len(value.get('graph', {}).get('nodes', [])),
                graph_edges=len(value.get('graph', {}).get('edges', [])),
                at_monotonic=time.monotonic()))
        latest.clear()
        latest.update(value)

    def markers(message):
        report['marker_samples'].append([
            dict(ns=m.ns, id=m.id, action=m.action, points=len(m.points),
                 lifetime_sec=m.lifetime.sec, lifetime_nanosec=m.lifetime.nanosec)
            for m in message.markers])

    node.create_subscription(String, '/se2_terrain/status', status, qos)
    node.create_subscription(String, '/se2_navmesh/map', navmesh, qos)
    node.create_subscription(MarkerArray, '/se2_terrain/markers', markers, qos)
    deadline = time.monotonic()+args.seconds
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.2)
    report['latest_map'] = latest
    Path(args.output).write_text(json.dumps(report, ensure_ascii=False, allow_nan=False))
    print(json.dumps({'map_versions': report['map_versions'],
                      'last_status': report['status_samples'][-1] if report['status_samples'] else None,
                      'marker_publications': len(report['marker_samples']),
                      'output': args.output}, ensure_ascii=False))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
