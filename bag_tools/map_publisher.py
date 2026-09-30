"""Publish the bag-owned PCD as an optional, retained RViz comparison layer."""
from pathlib import Path
import sys

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


def main():
    with Path(sys.argv[1]).open('rb') as stream:
        header = {}
        while True:
            line = stream.readline()
            if not line:
                raise ValueError('PCD 头部不完整')
            parts = line.decode().strip().split()
            if parts and not parts[0].startswith('#'):
                header[parts[0]] = parts[1:]
            if parts and parts[0] == 'DATA':
                break
        if header.get('FIELDS') != ['x', 'y', 'z'] or header.get('DATA') != ['binary']:
            raise ValueError('不是此录制工具保存的 XYZ binary PCD')
        data = stream.read()
    count = int(header['POINTS'][0])
    if len(data) != count * 12:
        raise ValueError('PCD 数据长度不匹配')
    rclpy.init()
    node = rclpy.create_node('se2_bag_saved_map')
    msg = PointCloud2()
    msg.header.frame_id = 'nvblox_odom'
    msg.height, msg.width, msg.point_step, msg.row_step = 1, count, 12, count*12
    msg.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                  for i, name in enumerate(('x', 'y', 'z'))]
    msg.is_dense = True
    msg.data = data
    pub = node.create_publisher(PointCloud2, '/bag_reference/superlio_map', QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    pub.publish(msg)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
