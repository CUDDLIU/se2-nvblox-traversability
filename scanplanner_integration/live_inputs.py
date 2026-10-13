"""Bring up session-owned odometry when live navigation starts after a reboot."""
import json
import math
import time


class InputHealth:
    def __init__(self):
        self.samples = {name: [] for name in ('cloud', 'imu', 'odom')}
        self.invalid = {}

    def receive(self, name, stamp, now, valid=True):
        samples = self.samples[name]
        if not valid or not math.isfinite(stamp) or not -.1 <= now - stamp < .5:
            self.invalid[name] = 'wrong frame, invalid pose, or stale timestamp'
            return
        if samples and stamp <= samples[-1]:
            return
        samples.append(stamp)
        del samples[:-20]
        self.invalid.pop(name, None)

    def fresh(self, name, now):
        samples = self.samples[name]
        return len(samples) >= 2 and -.1 <= now - samples[-1] < .5

    def summary(self, now):
        return {name: dict(samples=len(samples), fresh=self.fresh(name, now),
                           age_s=now-samples[-1] if samples else None,
                           rejected=self.invalid.get(name))
                for name, samples in self.samples.items()}


def ensure_live_inputs(root, output, spawn, running, timeout=45.):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import PointCloud2, Imu
    from nav_msgs.msg import Odometry

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('se2_scan_live_inputs')
    health = InputHealth()
    lio = None
    began = time.monotonic()
    last_report = 0.
    publisher_count = 0
    def receive(name, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        valid = True
        if name == 'odom':
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
            valid = (msg.header.frame_id == 'nvblox_odom' and
                     msg.child_frame_id == 'base_link_dog' and
                     all(math.isfinite(v) for v in values) and
                     .9 < sum(v*v for v in values[3:]) < 1.1)
        elif name == 'cloud':
            valid = (msg.header.frame_id in ('lidar_link', 'base_link_dog') and
                     msg.width * msg.height > 0)
        health.receive(name, stamp, node.get_clock().now().nanoseconds/1e9, valid)

    for name, topic, kind in [('cloud', '/LIDAR/POINTS_NX', PointCloud2),
                              ('imu', '/IMU', Imu),
                              ('odom', '/nvblox_lio/odom', Odometry)]:
        node.create_subscription(kind, topic, lambda msg, n=name: receive(n, msg),
                                 qos_profile_sensor_data)

    def report(state):
        data = dict(state=state, owned_lio=lio is not None,
                    odom_publishers=publisher_count,
                    inputs=health.summary(node.get_clock().now().nanoseconds/1e9))
        temporary = output / 'live_inputs.tmp'
        temporary.write_text(json.dumps(data, indent=2))
        temporary.replace(output / 'live_inputs.json')
        return data

    try:
        print('Checking live LiDAR and odometry...', flush=True)
        while running() and time.monotonic()-began < timeout:
            rclpy.spin_once(node, timeout_sec=.1)
            now = node.get_clock().now().nanoseconds/1e9
            elapsed = time.monotonic()-began
            publisher_count = len(node.get_publishers_info_by_topic('/nvblox_lio/odom'))
            if publisher_count > 1:
                raise RuntimeError('Multiple /nvblox_lio/odom publishers; cannot select a unique pose source')
            if lio is not None and lio.poll() is not None:
                raise RuntimeError(f'SuperLIO exited ({lio.returncode}); see {output / "lio.log"}')
            # Allow DDS discovery before deciding whether a new source is needed.
            if elapsed >= 3. and publisher_count == 1 and all(health.fresh(n, now) for n in ('cloud', 'odom')):
                result = report('ready')
                print('Live LiDAR and body odometry ready' + (' (session-owned SuperLIO)' if lio else ' (existing SuperLIO)'), flush=True)
                return result
            if (elapsed >= 3. and publisher_count == 0 and lio is None and
                    all(health.fresh(n, now) for n in ('cloud', 'imu'))):
                print('No live odometry publisher; starting session-owned SuperLIO...', flush=True)
                lio = spawn('lio', ['bash', str(root/'scanplanner_integration/start_lio.sh')])
            if elapsed-last_report >= 2.:
                report('waiting')
                last_report = elapsed
        if not running():
            report('cancelled')
            return None
        raise RuntimeError('Live inputs did not become ready: ' + json.dumps(report('failed')['inputs']) +
                           f'; see {output / "lio.log"}')
    except Exception:
        report('failed')
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()
