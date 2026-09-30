#!/usr/bin/env python3
"""A delayed UDP consumer must resume fresh 30 Hz commands, without replaying a queue.

Only localhost is used. SIGSTOP reproduces a scheduler/IO stall in the sender;
the ROS publisher continues at the real controller's rate during that stall.
"""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from core import packet, decode_packet


def main():
    root = Path(__file__).resolve().parents[1]
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', 0))
    sock.setblocking(False)
    rclpy.init()
    node = rclpy.create_node('m20_udp_backlog_test')
    latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    pub = node.create_publisher(TwistStamped, '/se2_scan/cmd_vel_stamped', latest)
    reports = []
    node.create_subscription(String, '/se2_scan/m20_status',
                             lambda m: reports.append(json.loads(m.data)), 10)
    proc = subprocess.Popen([sys.executable, str(root / 'm20_udp.py'),
        '--loopback-test', '--host', '127.0.0.1', '--port', str(sock.getsockname()[1])],
        start_new_session=True)
    peer = None
    received = []
    began = time.monotonic()
    next_publish = began
    last_status = 0.
    paused = False
    paused_at = resumed_at = None
    try:
        while time.monotonic() - began < 9:
            rclpy.spin_once(node, timeout_sec=.001)
            now = time.monotonic()
            while True:
                try:
                    data, peer = sock.recvfrom(65536)
                except BlockingIOError:
                    break
                _, doc = decode_packet(data)
                if (doc['Type'], doc['Command']) == (2, 25):
                    received.append((now, doc['Items']))
            if peer and now - last_status >= .08:
                basic = dict(ControlUsageMode=1, HES=0, MotionState=17, Direction=0, Sleep=0)
                sock.sendto(packet(1002, 6, dict(BasicStatus=basic)), peer)
                sock.sendto(packet(1002, 4, dict(MotionStatus=dict(Height=.57))), peer)
                last_status = now
            if now >= next_publish:
                msg = TwistStamped()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = 'base_link_dog'
                msg.twist.linear.x = .1
                if 6 <= now - began < 7.5:
                    msg.header.stamp.sec -= 2
                pub.publish(msg)
                next_publish = now + 1 / 30
            if paused_at is None and now - began > 2:
                assert any(v['X'] == .1 for _, v in received), 'No initial fresh command'
                os.kill(proc.pid, signal.SIGSTOP)
                paused = True
                paused_at = now
            if paused and now - paused_at >= .45:
                os.kill(proc.pid, signal.SIGCONT)
                paused = False
                resumed_at = now
        recovered = [t for t, v in received if resumed_at + .2 < t < began + 6 and v['X'] == .1]
        stale = [v for t, v in received if began + 6.2 < t < began + 7.5]
        recovered_again = [v for t, v in received if t > began + 7.8 and v['X'] == .1]
        result = dict(state='passed' if recovered and stale and recovered_again else 'failed',
            post_stall_fresh_packets=len(recovered), stale_input_packets=len(stale),
            post_stale_fresh_packets=len(recovered_again), last_status=reports[-1:] if reports else [],
            destination='127.0.0.1')
        print(json.dumps(result), flush=True)
        assert recovered, 'Fresh 30 Hz commands never recovered after a 450 ms consumer stall'
        assert stale and all(v['X'] == 0 and v['Y'] == 0 and v['Yaw'] == 0 for v in stale)
        assert recovered_again, 'Fresh commands did not recover after stale input'
    finally:
        if proc.poll() is None:
            if paused:
                os.kill(proc.pid, signal.SIGCONT)
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=5)
        sock.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
