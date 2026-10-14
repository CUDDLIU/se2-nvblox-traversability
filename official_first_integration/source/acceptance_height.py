"""Isolated ROS transport acceptance; synthetic sensors, no motion publisher."""
import json,math,sys,time
from pathlib import Path
import numpy as np
import rclpy
from sensor_msgs.msg import Imu,PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import Header,String,Int32
from std_srvs.srv import SetBool
from rclpy.qos import qos_profile_sensor_data
from height_adapter import HeightAdapter
from runtime import cloud_message,xyz_cloud

def run(output):
    rclpy.init();node=rclpy.create_node('height_acceptance');adapter=HeightAdapter(node,output)
    imu=node.create_publisher(Imu,'/IMU',qos_profile_sensor_data)
    odom=node.create_publisher(Odometry,'/nvblox_lio/odom_lidar',qos_profile_sensor_data)
    cloud=node.create_publisher(PointCloud2,'/nvblox_lio/body_cloud',qos_profile_sensor_data)
    terrain=node.create_publisher(Int32,'/first/source_height/terrain',1)
    statuses=[];nav=[]
    node.create_subscription(String,'/first/source_height/status',lambda m:statuses.append(json.loads(m.data)),10)
    node.create_subscription(PointCloud2,'/first/source_height/cloud_nav',lambda m:nav.append(m),10)
    client=node.create_client(SetBool,'/first/source_height/stair_oa')
    points=np.array([(x,y,-.55 if x<1. else -.42) for x in np.arange(.5,2.,.03) for y in np.arange(-.6,.6,.03)],np.float32)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    def header(t):
        h=Header();h.stamp.sec=math.floor(t);h.stamp.nanosec=round((t-h.stamp.sec)*1e9);h.frame_id='base_link_dog';return h
    def frame(i,match=True):
        t=time.time()-.05
        for s in (t-.01,t+.01):
            m=Imu();m.header=header(s);m.orientation.w=1.;imu.publish(m)
        m=Odometry();m.header=header(t if match else t-.01);m.header.frame_id='nvblox_odom';m.child_frame_id='base_link_dog';m.pose.pose.orientation.w=1.;odom.publish(m)
        cloud.publish(cloud_message(points,header(t)));spin(.14)
    try:
        spin(.3);terrain.publish(Int32(data=2));spin(.1)
        for i in range(6):frame(i)
        a=[s for s in statuses if s['state']=='height_map_ready'];assert a and all(s['gravity_matched'] for s in a),a
        assert len(nav) and nav[-1].header.frame_id=='first_height/gravity'
        assert len(xyz_cloud(nav[-1]))==0,xyz_cloud(nav[-1])
        before=adapter.frames;frame(7,match=False);assert adapter.frames==before
        future=client.call_async(SetBool.Request(data=True));spin(.15);assert future.done() and future.result().success
        for i in range(8,12):frame(i)
        assert len(xyz_cloud(nav[-1]))>0,'OA=.08 must reject the 13 cm step'
        assert all(p.topic_name not in ('/NAV_CMD','/first/cmd_vel_stamped','/first/request_gait') for p in node.publishers)
        result=dict(frames=adapter.frames,missing_pose_pair_ignored=True,gravity_matches=True,
                    terrain=adapter.frontend.policy.terrain,stair_oa_service=True,
                    wall_cells_when_oa_enabled=len(xyz_cloud(nav[-1])),motion_publishers=0)
        Path(output,'acceptance.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
    finally:adapter.close();node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':run(Path(sys.argv[1]))
