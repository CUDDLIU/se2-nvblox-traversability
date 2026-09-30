"""Replay-only adapter: sim timestamps with wall timers for paused-frame tuning."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'se2_terrain_check'))
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosgraph_msgs.msg import Clock as ClockMessage
from realtime_node import RealtimeNode, ExternalShutdownException


class ReplayTerrain(RealtimeNode):
    def __init__(self):
        super().__init__()
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.replay_clock_pub=self.create_publisher(ClockMessage,'/bag_rebuild/clock',qos)
        # Keep the last bag timestamp queryable after the rosbag player exits.
        self.create_timer(.2,lambda: self.replay_clock_pub.publish(
            ClockMessage(clock=self.get_clock().now().to_msg())))

    def create_timer(self, timer_period_sec, callback, *args, **kwargs):
        # Input is paused with /clock, but parameter reclassification must keep
        # running. Output timestamps still come from the node's ROS clock.
        if not hasattr(self, '_wall_clock'):
            self._wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        kwargs['clock'] = self._wall_clock
        return super().create_timer(timer_period_sec, callback, *args, **kwargs)


def main():
    rclpy.init()
    node = ReplayTerrain()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
