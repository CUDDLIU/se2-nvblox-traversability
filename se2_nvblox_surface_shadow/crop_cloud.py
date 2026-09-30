import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2

class CropCloud(Node):
    def __init__(self):
        super().__init__('se2_nvblox_cloud_crop')
        self.declare_parameter('input_topic', '/LIDAR/POINTS_NX')
        self.declare_parameter('output_topic', '/se2_nvblox/lidar_points_cropped')
        self.declare_parameter('z_min_m', -0.8)
        self.declare_parameter('z_max_m', 1.5)
        self.declare_parameter('max_xy_range_m', 2.0)
        self.declare_parameter('front_only', True)
        self.declare_parameter('front_x_min_m', 0.0)
        self.pub = self.create_publisher(PointCloud2, self.get_parameter('output_topic').value, QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST))
        self.sub = self.create_subscription(PointCloud2, self.get_parameter('input_topic').value,
            self.cb, QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST))
        self.count = 0
        self.get_logger().info('CROP_START frame=lidar_link z_min=%.3f z_max=%.3f max_xy=%.3f front_only=%s front_x_min=%.3f input=%s output=%s' % (
            float(self.get_parameter('z_min_m').value), float(self.get_parameter('z_max_m').value),
            float(self.get_parameter('max_xy_range_m').value), bool(self.get_parameter('front_only').value),
            float(self.get_parameter('front_x_min_m').value),
            self.get_parameter('input_topic').value, self.get_parameter('output_topic').value))
    def cb(self, msg):
        started = time.perf_counter()
        try:
            fx = next(f for f in msg.fields if f.name == 'x')
            fy = next(f for f in msg.fields if f.name == 'y')
            fz = next(f for f in msg.fields if f.name == 'z')
            if any(f.datatype != 7 for f in (fx, fy, fz)):
                raise ValueError('x/y/z must be FLOAT32')
            zmin, zmax = float(self.get_parameter('z_min_m').value), float(self.get_parameter('z_max_m').value)
            max_r2 = float(self.get_parameter('max_xy_range_m').value) ** 2
            front_only = bool(self.get_parameter('front_only').value)
            front_x_min = float(self.get_parameter('front_x_min_m').value)
            endian = '>' if msg.is_bigendian else '<'
            dtype = np.dtype({'names': ['x', 'y', 'z'],
                              'formats': [endian + 'f4'] * 3,
                              'offsets': [fx.offset, fy.offset, fz.offset],
                              'itemsize': msg.point_step})
            points = np.ndarray((msg.height, msg.width), dtype=dtype,
                                buffer=msg.data, strides=(msg.row_step, msg.point_step))
            x, y, z = points['x'], points['y'], points['z']
            keep = (np.isfinite(x) & np.isfinite(y) & np.isfinite(z) &
                    (z >= zmin) & (z <= zmax) & (x*x + y*y <= max_r2))
            if front_only:
                keep &= x >= front_x_min
            # Copy complete records, preserving ring, timestamp and unknown fields.
            records = np.ndarray((msg.height, msg.width, msg.point_step),
                                 dtype=np.uint8, buffer=msg.data,
                                 strides=(msg.row_step, msg.point_step, 1))
            out = records[keep].tobytes()
            result = PointCloud2()
            result.header = msg.header
            result.height, result.width = 1, len(out) // msg.point_step
            result.fields = msg.fields
            result.is_bigendian, result.point_step = msg.is_bigendian, msg.point_step
            result.row_step, result.is_dense, result.data = len(out), msg.is_dense, bytes(out)
            self.pub.publish(result)
            self.count += 1
            if self.count % 10 == 0:
                self.get_logger().info('CROP frame=%d input=%d output=%d ratio=%.3f xy<=%.2fm front=%s z=[%.3f,%.3f] callback_ms=%.3f' %
                    (self.count, msg.width * msg.height, result.width,
                     result.width / max(1, msg.width * msg.height),
                     float(self.get_parameter('max_xy_range_m').value), front_only, zmin, zmax,
                     (time.perf_counter() - started) * 1000.0))
        except Exception as exc:
            self.get_logger().error('CROP_REJECT reason=%r' % (exc,))
def main(args=None):
    rclpy.init(args=args); node = CropCloud()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node(); rclpy.shutdown()
