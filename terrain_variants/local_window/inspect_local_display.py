"""Bounded read-only audit of local geometry and RViz marker payloads."""
import json
import time
from collections import Counter
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

rclpy.init()
node = rclpy.create_node('se2_local_display_audit')
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
report = dict(events=Counter(), counts=Counter(), statuses=[], markers={}, display={}, validity={}, sequences=set(),
              latencies=[], tiles={})

def local(msg):
    d = json.loads(msg.data)
    report['events'][d['event']] += 1
    if d['event'] == 'replace':
        report['sequences'].add(d['input_sequence'])
        report['latencies'].append(d['latency_oldest_ms'])
        full = (1 << d['yaw_bins'])-1
        for slab in d['updated_slabs']:
            counts = Counter('green' if c[3] == full else 'amber' if c[3] else 'rejected'
                             for c in slab['cells'])
            report['counts'].update(counts)
            report['tiles'][str(slab['key'])] = dict(counts)

def status(msg):
    report['statuses'].append(json.loads(msg.data))

def marker(msg):
    for m in msg.markers:
        report['markers'][m.ns] = dict(type=m.type, action=m.action, points=len(m.points))

node.create_subscription(String, '/se2_navmesh/local_result', local, 100)
node.create_subscription(String, '/se2_terrain/status', status, qos)
node.create_subscription(MarkerArray, '/se2_navmesh/completed_markers', marker, qos)
def view(msg, field):
    report[field] = {m.ns: dict(type=m.type, action=m.action, points=len(m.points))
                     for m in msg.markers}
node.create_subscription(MarkerArray, '/se2_navmesh/local_display',
                         lambda msg: view(msg, 'display'), qos)
node.create_subscription(MarkerArray, '/se2_navmesh/local_validity_display',
                         lambda msg: view(msg, 'validity'), qos)
end = time.monotonic()+8
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=.1)
report['statuses'] = report['statuses'][-3:]
report['sequences'] = sorted(report['sequences'])
print(json.dumps(report))
node.destroy_node()
rclpy.shutdown()
