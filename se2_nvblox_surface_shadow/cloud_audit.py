import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2


class CloudAudit(Node):
    def __init__(self):
        super().__init__('se2_cloud_audit')
        self.topic = self.declare_parameter('input_topic', '/LIDAR/POINTS_NX').value
        cloud_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                               history=HistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(PointCloud2, self.topic, self.cb, cloud_qos)
        self.last_stamp = None
        self.count = 0
        self.t0 = time.monotonic()
        self.log = self.get_logger()
        self.log.info(f'AUDIT_START topic={self.topic}')

    def cb(self, msg):
        now = time.monotonic()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = None if self.last_stamp is None else stamp - self.last_stamp
        self.last_stamp = stamp
        self.count += 1
        fields = ','.join(f.name for f in msg.fields)
        self.log.info(
            f'CLOUD seq={self.count} stamp={stamp:.9f} '
            f'dt={"none" if dt is None else f"{dt:.6f}"} frame={msg.header.frame_id} '
            f'point_step={msg.point_step} width={msg.width} height={msg.height} '
            f'fields={fields} elapsed_ms={(now - self.t0) * 1000.0:.3f} '
            f'age_ms={(self.get_clock().now().nanoseconds * 1e-9 - stamp) * 1000.0:.3f}')


def main(args=None):
    rclpy.init(args=args)
    node = CloudAudit()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
