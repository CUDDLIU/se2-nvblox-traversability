import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Header


class LocalizationGate(Node):
    """Keep the gated shadow launch stopped until relocation is accepted."""

    def __init__(self):
        super().__init__('se2_nvblox_localization_gate')
        self.declare_parameter('verification_topic', '/lio/relocation/verification')
        self.declare_parameter('log_period_s', 5.0)
        topic = self.get_parameter('verification_topic').value
        self.accepted = False
        self.started = time.monotonic()
        self.last_log = 0.0
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(Header, topic, self.cb, qos)
        self.get_logger().info(
            f'GATE_WAIT topic={topic} accepted_frame_id=accepted '
            'nvblox_start=blocked')

    def cb(self, msg):
        decision = str(msg.frame_id).strip().lower()
        stamp = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        if decision == 'accepted':
            self.accepted = True
            self.get_logger().info(
                f'GATE_ACCEPTED verification_stamp={stamp:.9f} '
                f'wait_s={time.monotonic() - self.started:.3f}')
            raise SystemExit(0)
        self.get_logger().warning(
            f'GATE_REJECTED frame_id={msg.frame_id!r} verification_stamp={stamp:.9f} '
            'nvblox_start=blocked')

    def heartbeat(self):
        now = time.monotonic()
        period = float(self.get_parameter('log_period_s').value)
        if now - self.last_log >= period:
            self.last_log = now
            self.get_logger().info(
                f'GATE_WAIT elapsed_s={now - self.started:.1f} '
                'last_decision=not_accepted nvblox_start=blocked')


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationGate()
    try:
        while rclpy.ok() and not node.accepted:
            rclpy.spin_once(node, timeout_sec=0.2)
            node.heartbeat()
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    main()
