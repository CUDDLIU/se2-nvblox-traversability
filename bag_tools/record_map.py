"""Save this recording's SuperLIO world points, trajectory and input health."""
import json
from pathlib import Path
import signal
import time

import numpy as np


def write_json(path, data):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2))
    temporary.replace(path)


class VoxelMap:
    def __init__(self, voxel=.2):
        self.voxel = voxel
        self.points = {}

    def add(self, xyz):
        xyz = np.asarray(xyz, dtype=np.float32)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        keys = np.floor(xyz / self.voxel).astype(np.int64)
        _, indices = np.unique(keys, axis=0, return_index=True)
        for i in indices:
            self.points.setdefault(tuple(keys[i]), tuple(xyz[i]))

    def save(self, path):
        path = Path(path)
        tmp = path.with_suffix('.pcd.tmp')
        count = len(self.points)
        header = ('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n'
                  f'TYPE F F F\nCOUNT 1 1 1\nWIDTH {count}\nHEIGHT 1\n'
                  f'VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {count}\nDATA binary\n')
        with tmp.open('wb') as stream:
            stream.write(header.encode())
            # Avoid an additional full-size list of all points at save time.
            chunk = []
            for point in self.points.values():
                chunk.append(point)
                if len(chunk) == 65536:
                    stream.write(np.asarray(chunk, dtype='<f4').tobytes())
                    chunk.clear()
            if chunk:
                stream.write(np.asarray(chunk, dtype='<f4').tobytes())
        tmp.replace(path)


def cloud_xyz(msg):
    fields = {f.name: f for f in msg.fields}
    endian = '>' if msg.is_bigendian else '<'
    formats = []
    offsets = []
    for name in ('x', 'y', 'z'):
        field = fields[name]
        if field.datatype not in (7, 8) or field.count != 1:
            raise ValueError('不支持的点云坐标格式')
        formats.append(endian + ('f4' if field.datatype == 7 else 'f8'))
        offsets.append(field.offset)
    dtype = np.dtype(dict(names=['x', 'y', 'z'], formats=formats,
                         offsets=offsets, itemsize=msg.point_step))
    points = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                        strides=(msg.row_step, msg.point_step))
    return np.column_stack([points[name].ravel() for name in ('x', 'y', 'z')])


def main():
    import sys
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2, Image, Imu
    from nav_msgs.msg import Odometry

    folder = Path(sys.argv[1])
    folder.mkdir(parents=True, exist_ok=True)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('se2_record_map')
    voxels = VoxelMap()
    counts, last, last_stamp = {}, {}, {}
    errors = []
    stopping = False
    started = time.monotonic()
    last_save = started
    last_pose = -float('inf')
    first_cloud_stamp = None
    events = (folder / 'input_events.jsonl').open('w', buffering=1)
    trajectory = (folder / 'trajectory.csv').open('w', buffering=1)
    trajectory.write('stamp,x,y,z,qx,qy,qz,qw\n')
    previous_warning = None
    had_input_gaps = False

    def received(key, msg):
        nonlocal last_pose, first_cloud_stamp
        now = time.monotonic()
        counts[key] = counts.get(key, 0) + 1
        last[key] = now
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        last_stamp[key] = stamp
        if key == 'map':
            if msg.header.frame_id != 'nvblox_odom':
                errors.append('点云坐标系不是 nvblox_odom，拒绝混入地图')
                return
            if first_cloud_stamp is None:
                first_cloud_stamp = stamp
            try:
                voxels.add(cloud_xyz(msg))
            except Exception as exc:
                errors.append(str(exc))
        elif key == 'odom' and stamp - last_pose >= .1:
            last_pose = stamp
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            trajectory.write(f'{stamp:.9f},{p.x},{p.y},{p.z},{q.x},{q.y},{q.z},{q.w}\n')

    topics = [('map', '/nvblox_lio/cloud_world', PointCloud2),
              ('odom', '/nvblox_lio/odom', Odometry),
              ('lidar', '/LIDAR/POINTS_NX', PointCloud2),
              ('imu', '/IMU', Imu),
              ('depth', '/camera/d435i/depth/image_rect_raw', Image)]
    subs = [node.create_subscription(typ, topic, lambda msg, k=key: received(k, msg),
                                     qos_profile_sensor_data) for key, topic, typ in topics]

    def health(final=False):
        nonlocal previous_warning, last_save, had_input_gaps
        now = time.monotonic()
        ages = {key: round(now-last[key], 2) if key in last else None for key, _, _ in topics}
        names = dict(map='地图点云', odom='位姿', lidar='雷达', imu='LIO IMU', depth='深度')
        stale = [names[k] for k, age in ages.items() if age is None or age > 2.]
        warning = ('输入断流/未就绪：' + '、'.join(stale)) if stale else ''
        if errors:
            warning += '；地图保存错误：' + errors[-1]
        if warning and now-started > 5:
            had_input_gaps = True
        if warning != previous_warning:
            events.write(json.dumps(dict(elapsed_s=round(now-started, 2), warning=warning), ensure_ascii=False)+'\n')
            previous_warning = warning
        if final or now-last_save >= 20:
            voxels.save(folder / 'superlio_map.pcd')
            last_save = now
        write_json(folder / 'map.json', dict(frame='nvblox_odom', source='/nvblox_lio/cloud_world',
            mode='online_mapping', loaded_old_map=False, voxel_size_m=voxels.voxel,
            points=len(voxels.points), first_cloud_stamp=first_cloud_stamp,
            counts=counts, last_stamps=last_stamp, ages_s=ages, warning=warning,
            had_input_gaps=had_input_gaps,
            errors=errors[-10:], final=final, updated_unix=time.time(),
            scope='本 bag 录制窗口内的 SuperLIO 世界坐标点云，0.2m 体素去重；不含回环优化'))

    def stop(*_):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    node.create_timer(1., health)
    health()
    try:
        while not stopping:
            rclpy.spin_once(node, timeout_sec=.2)
    finally:
        health(final=True)
        trajectory.close()
        events.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
