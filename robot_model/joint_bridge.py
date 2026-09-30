"""Produce nvblox-only JointState from raw telemetry, in live or bag time."""
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'bag_tools'))
from motion_status import NAMES, TOPICS, split_joints
from kinematics import JOINT_NAMES, model_angles, WheelIntegrator
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import String


def main():
    rclpy.init();node=rclpy.create_node('nvblox_measured_joint_bridge')
    publisher=node.create_publisher(JointState,'/nvblox_robot/joint_states',30)
    wheel=WheelIntegrator();last_warning=[0.]
    def receive(msg):
        try:
            record=json.loads(msg.data);patrol=record['PatrolDevice']
            if (patrol['Type'],patrol['Command'])!=(1002,4):return
            angles,speeds=split_joints(patrol);raw=dict(angles+speeds)
            values=[raw[name] for name in NAMES];positions=model_angles(values)
            stamp_ns=record['receive_time_ns']
            if not isinstance(stamp_ns,int) or stamp_ns<=0:raise ValueError('invalid receive timestamp')
            rotation,_=wheel.update(values,stamp_ns)
            for i,p in zip((3,7,11,15),rotation):positions[i]=p
            state=JointState();state.header.stamp=rclpy.time.Time(nanoseconds=stamp_ns).to_msg()
            state.name=list(JOINT_NAMES);state.position=positions
            publisher.publish(state)
        except (ValueError,KeyError,TypeError) as e:
            import time
            now=time.monotonic()
            if now-last_warning[0]>2:node.get_logger().warning(str(e));last_warning[0]=now
    node.create_subscription(String,TOPICS[0],receive,30)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
