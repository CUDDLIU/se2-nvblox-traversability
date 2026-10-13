#!/usr/bin/env python3
"""Read-only measured M20 model in FIRST's own TF namespace."""
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]

def description():
    directory=ROOT/'robot_model/upstream/M20/urdf'
    model=ET.parse(directory/'M20.urdf').getroot()
    for link in model.findall('link'):link.set('name','first_robot/'+link.get('name'))
    for joint in model.findall('joint'):
        joint.set('name','first_robot/'+joint.get('name'))
        for tag in ('parent','child'):
            item=joint.find(tag);item.set('link','first_robot/'+item.get('link'))
    for mesh in model.findall('.//mesh'):
        path=(directory/mesh.get('filename')).resolve()
        if not path.is_file():raise FileNotFoundError(path)
        mesh.set('filename',path.as_uri())
    for tag in ('mujoco','gazebo'):
        for item in model.findall(tag):model.remove(item)
    return ET.tostring(model,encoding='unicode')

def main():
    import math
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster
    sys.path.insert(0,str(ROOT/'robot_model'))
    sys.path.insert(0,str(ROOT/'bag_tools'))
    from kinematics import JOINT_NAMES,model_angles,WheelIntegrator
    from motion_status import NAMES,split_joints
    rclpy.init();node=rclpy.create_node('first_measured_robot_visual')
    joint_pub=node.create_publisher(JointState,'/first/joint_states',10)
    broadcaster=TransformBroadcaster(node);wheel=WheelIntegrator();last_warning=[0.]
    def receive(msg):
        try:
            record=json.loads(msg.data);patrol=record['PatrolDevice']
            if (patrol['Type'],patrol['Command'])!=(1002,4):return
            stamp=record['receive_time_ns'];age=(node.get_clock().now().nanoseconds-stamp)/1e9
            if not isinstance(stamp,int) or not 0<=age<.35:return
            angles,speeds=split_joints(patrol);raw=dict(angles+speeds)
            values=[raw[name] for name in NAMES];positions=model_angles(values)
            rotation,_=wheel.update(values,stamp)
            for i,value in zip((3,7,11,15),rotation):positions[i]=value
            state=JointState();state.header.stamp=rclpy.time.Time(nanoseconds=stamp).to_msg()
            state.name=[name.replace('nvblox_robot/','first_robot/',1) for name in JOINT_NAMES]
            state.position=positions;joint_pub.publish(state)
        except (KeyError,ValueError,TypeError) as exc:
            now=node.get_clock().now().nanoseconds/1e9
            if now-last_warning[0]>2:node.get_logger().warning(str(exc));last_warning[0]=now
    def body(msg):
        if msg.header.frame_id!='se2_scan_map':return
        age=(node.get_clock().now().nanoseconds-(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        if not 0<=age<.2 or not all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w)):return
        t=TransformStamped();t.header=msg.header;t.child_frame_id='first_robot/base_link'
        t.transform.translation.x=p.x;t.transform.translation.y=p.y;t.transform.translation.z=p.z
        t.transform.rotation=q;broadcaster.sendTransform(t)
    node.create_subscription(String,'/nvblox_robot/motion_status/raw',receive,10)
    node.create_subscription(Odometry,'/first/aligned_odom',body,qos_profile_sensor_data)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
