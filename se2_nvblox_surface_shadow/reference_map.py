"""Read-only display copy of the actual relocation map; never invents TF."""
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


def sparse_xyz(msg, leaf, limit):
    fields = {f.name: f for f in msg.fields}
    if any(k not in fields or fields[k].datatype != PointField.FLOAT32 for k in ('x', 'y', 'z')):
        raise ValueError('expected FLOAT32 x/y/z')
    dtype = np.dtype({'names': ['x', 'y', 'z'],
                      'formats': [('>' if msg.is_bigendian else '<') + 'f4'] * 3,
                      'offsets': [fields[k].offset for k in ('x', 'y', 'z')],
                      'itemsize': msg.point_step})
    data = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                      strides=(msg.row_step, msg.point_step))
    xyz = np.stack([data[k].ravel() for k in ('x', 'y', 'z')], axis=1)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if len(xyz):
        _, idx = np.unique(np.floor(xyz.astype(np.float64) / leaf), axis=0, return_index=True)
        xyz = xyz[np.sort(idx)]
    if len(xyz) > limit:
        xyz = xyz[np.linspace(0, len(xyz) - 1, limit, dtype=int)]
    return np.asarray(xyz, dtype='<f4')


class ReferenceMap(Node):
    def __init__(self):
        super().__init__('se2_reference_map')
        self.declare_parameter('input_topic', '/lio/global_map')
        self.declare_parameter('voxel_m', 0.4)
        self.declare_parameter('max_points', 150000)
        self.last = -float('inf')
        self.pub = self.create_publisher(PointCloud2, '/se2_nvblox/reference_map',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.sub = self.create_subscription(PointCloud2,
            self.get_parameter('input_topic').value, self.receive, 1)
        self.get_logger().info('REFERENCE_START input=/lio/global_map output=/se2_nvblox/reference_map display_only=true preserve_frame=true')

    def receive(self, msg):
        if time.monotonic() - self.last < 5.0:
            return
        self.last = time.monotonic()
        try:
            leaf = float(self.get_parameter('voxel_m').value)
            limit = int(self.get_parameter('max_points').value)
            if leaf <= 0 or limit <= 0 or not msg.header.frame_id:
                raise ValueError('invalid voxel size, point cap or empty frame')
            xyz = sparse_xyz(msg, leaf, limit)
            out = PointCloud2()
            out.header = msg.header
            out.height, out.width = 1, len(xyz)
            out.fields = [PointField(name=k, offset=i*4, datatype=PointField.FLOAT32, count=1)
                          for i, k in enumerate(('x', 'y', 'z'))]
            out.point_step, out.row_step = 12, len(xyz)*12
            out.is_dense = True
            out.data = xyz.tobytes()
            self.pub.publish(out)
            self.get_logger().info(f'REFERENCE_MAP frame={out.header.frame_id} input={msg.width*msg.height} output={len(xyz)} voxel_m={leaf} ms={(time.monotonic()-self.last)*1000:.1f}')
        except Exception as exc:
            self.get_logger().error(f'REFERENCE_REJECT reason={exc!r}')


def main(args=None):
    rclpy.init(args=args)
    node = ReferenceMap()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
