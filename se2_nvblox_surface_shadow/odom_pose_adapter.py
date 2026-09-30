import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster


class OdomPoseAdapter(Node):
    def __init__(self):
        super().__init__('se2_nvblox_odom_pose_adapter')
        self.declare_parameter('odom_topic', '/lio/odom')
        self.declare_parameter('pose_topic', '/se2_nvblox/pose')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link_dog')
        self.declare_parameter('publish_tf', True)
        self.pub = self.create_publisher(PoseStamped, self.get_parameter('pose_topic').value, 10)
        self.tf_pub = TransformBroadcaster(self) if self.get_parameter('publish_tf').value else None
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(Odometry, self.get_parameter('odom_topic').value,
                                            self.cb, qos)
        self.count = 0
        self.get_logger().info(
            f'POSE_ADAPTER_START odom={self.get_parameter("odom_topic").value} '
            f'pose={self.get_parameter("pose_topic").value} '
            f'frame={self.get_parameter("frame_id").value}')
        self.get_logger().info('POSE_TF mode=%s child=%s' % (
            'enabled' if self.tf_pub else 'disabled', self.get_parameter('child_frame_id').value))

    def cb(self, msg):
        out = PoseStamped()
        out.header = msg.header
        out.header.frame_id = self.get_parameter('frame_id').value
        out.pose = msg.pose.pose
        self.pub.publish(out)
        if self.tf_pub is not None:
            tf = TransformStamped()
            tf.header = msg.header
            tf.header.frame_id = self.get_parameter('frame_id').value
            tf.child_frame_id = self.get_parameter('child_frame_id').value
            tf.transform.translation.x = msg.pose.pose.position.x
            tf.transform.translation.y = msg.pose.pose.position.y
            tf.transform.translation.z = msg.pose.pose.position.z
            tf.transform.rotation = msg.pose.pose.orientation
            self.tf_pub.sendTransform(tf)
        self.count += 1
        if self.count % 200 == 0:
            self.get_logger().info(f'POSE seq={self.count} stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}')


def main(args=None):
    rclpy.init(args=args)
    node = OdomPoseAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
