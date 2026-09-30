"""Retain reference layers for late RViz subscriptions and post-EOF comparison."""
from pathlib import Path
import struct
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'se2_terrain_check'))
from mesh_wire import Reader, check_schema


class MeshCache:
    def __init__(self):
        self.blocks = {}
        self.prefix = None
        self.identity = None

    def apply(self, raw):
        r = Reader(raw)
        r.scalar('i'); r.scalar('I')
        frame = bytes(r.take(r.scalar('I'), 1))
        size = r.scalar('f')
        prefix = bytes(r.raw[:r.offset])
        indices = r.array('i4', 3)
        count = r.scalar('I')
        if count != len(indices):
            raise ValueError('Mesh index/block count mismatch')
        updates = []
        for key in indices:
            begin = r.offset
            vertices = r.array('f4', 3, skip=True)
            r.array('f4', 3, skip=True)
            r.array('f4', 4, skip=True)
            r.array('i4', skip=True)
            updates.append((tuple(map(int, key)), bytes(r.raw[begin:r.offset]) if vertices else None))
        clear = r.scalar('B')
        if clear not in (0, 1) or len(raw)-r.offset > 3:
            raise ValueError('Invalid Mesh trailer')
        identity = (r.endian, frame, size)
        if clear or identity != self.identity:
            self.blocks.clear()
        self.identity, self.prefix = identity, prefix
        for key, block in updates:
            if block is None:self.blocks.pop(key, None)
            else:self.blocks[key] = block

    def snapshot(self):
        if self.prefix is None:
            return None
        endian = self.identity[0]
        count = struct.pack(endian + 'I', len(self.blocks))
        indices = b''.join(struct.pack(endian + 'iii', *k) for k in self.blocks)
        return b''.join((self.prefix, count, indices, count, *self.blocks.values(), b'\x01'))


def main():
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from nvblox_msgs.msg import Mesh, MeshBlock, Index3D
    from visualization_msgs.msg import MarkerArray
    from std_msgs.msg import String
    check_schema(Mesh, MeshBlock, Index3D)
    rclpy.init()
    node = rclpy.create_node('se2_bag_reference_cache')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    mesh = node.create_publisher(Mesh, '/bag_rebuild/reference_mesh', 100)
    cache = MeshCache()
    def receive(raw):
        cache.apply(raw)
        if mesh.get_subscription_count():mesh.publish(raw)
    node.create_subscription(Mesh, '/bag_reference/nvblox/height_mesh', receive, 100, raw=True)
    for cls, source, dest in (
        (MarkerArray, '/bag_reference/se2_navmesh/local_display', '/bag_rebuild/reference_display'),
        (String, '/bag_reference/se2_terrain/status', '/bag_rebuild/reference_status')):
        pub = node.create_publisher(cls, dest, qos)
        node.create_subscription(cls, source, lambda raw, pub=pub: pub.publish(raw), qos, raw=True)
    last = [0]
    def check_subscribers():
        count = mesh.get_subscription_count()
        if count > last[0]:
            snapshot = cache.snapshot()
            if snapshot is not None:mesh.publish(snapshot)
        last[0] = count
    node.create_timer(.2, check_subscribers)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__ == '__main__':
    main()
