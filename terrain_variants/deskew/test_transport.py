"""ROS regression: actual binary, timed poses, raw 26-byte cloud and waiting."""
import argparse
import os
import signal
import struct
import subprocess
import time
import math

os.environ.update(ROS_DOMAIN_ID='75',ROS_LOCALHOST_ONLY='1')
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2,PointField
from nav_msgs.msg import Odometry


def main():
    p=argparse.ArgumentParser();p.add_argument('binary');p.add_argument('--restore-scan-time',action='store_true');args=p.parse_args()
    rclpy.init();node=rclpy.create_node('deskew_transport_test')
    cloud_pub=node.create_publisher(PointCloud2,'/LIDAR/POINTS_NX',qos_profile_sensor_data)
    pose_pub=node.create_publisher(Odometry,'/nvblox_lio/odom',300)
    received=[]
    sub=node.create_subscription(PointCloud2,'/terrain_variants/lidar_deskewed',received.append,qos_profile_sensor_data)
    process=subprocess.Popen([args.binary,'--ros-args','-p','restore_scan_time:='+str(args.restore_scan_time).lower()])
    def until(condition,seconds=7.):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.01)
            if condition():return
        raise AssertionError('ROS timeout')
    def set_stamp(stamp,t):stamp.sec=int(t);stamp.nanosec=round((t-int(t))*1e9)
    def pose(t):
        m=Odometry();set_stamp(m.header.stamp,t);m.header.frame_id='nvblox_odom';m.child_frame_id='base_link_dog'
        m.pose.pose.position.x=t-100.;m.pose.pose.orientation.w=1.;pose_pub.publish(m)
    def cloud(reference,times,frame='lidar_link',copies=1):
        c=PointCloud2();set_stamp(c.header.stamp,reference);c.header.frame_id=frame
        c.height=1;c.width=len(times)*copies;c.point_step=26;c.row_step=26*c.width
        c.fields=[PointField(name=n,offset=off,datatype=dt,count=1) for n,off,dt in
            [('x',0,7),('y',4,7),('z',8,7),('intensity',12,7),('ring',16,4),('timestamp',18,8)]]
        c.data=b''.join(struct.pack('<ffffHd',1.-(t-100.),2.,3.,17.,9,t) for t in times)*copies
        cloud_pub.publish(c)
    try:
        until(lambda:cloud_pub.get_subscription_count()==1 and pose_pub.get_subscription_count()==1 and node.count_publishers('/terrain_variants/lidar_deskewed')==1)
        # DDS discovery visibility may precede best-effort writer matching.
        start=time.monotonic()
        while time.monotonic()-start<.3:rclpy.spin_once(node,timeout_sec=.01)
        for i in range(21):
            pose(100.+i*.005);rclpy.spin_once(node,timeout_sec=.002)
        cloud(100.,[100.,100.05,100.1]);until(lambda:len(received)==1)
        out=received[-1]
        for i in range(3):
            x,y,z,intensity,ring,t=struct.unpack_from('<ffffHd',out.data,26*i)
            assert abs(x-1.)<1e-5 and y==2 and z==3 and intensity==17 and ring==9 and t==100.
        # One full-size 3.9 MB cloud detects undersized transport buffers and
        # exercises the same non-aligned point stride as the actual recording.
        cloud(100.,[100.,100.05,100.1],copies=50000);until(lambda:len(received)==2)
        assert received[-1].width==150000
        for i in (0,49999,99999,149999):
            assert abs(struct.unpack_from('<f',received[-1].data,26*i)[0]-1.)<1e-5
        cloud(100.2,[100.2,100.25])
        start=time.monotonic()
        while time.monotonic()-start<.08:rclpy.spin_once(node,timeout_sec=.01)
        assert len(received)==2,'must not extrapolate missing future odometry'
        for i in range(21,52):pose(100.+i*.005)
        until(lambda:len(received)==3)
        for i in range(2):assert abs(struct.unpack_from('<f',received[-1].data,26*i)[0]-.8)<1e-5
        expected=3
        if args.restore_scan_time:
            c=PointCloud2();set_stamp(c.header.stamp,100.1);c.header.frame_id='lidar_link'
            c.height=1;c.width=1801;c.point_step=26;c.row_step=26*c.width
            c.fields=[PointField(name=n,offset=off,datatype=dt,count=1) for n,off,dt in
                [('x',0,7),('y',4,7),('z',8,7),('intensity',12,7),('ring',16,4),('timestamp',18,8)]]
            c.data=b''.join(struct.pack('<ffffHd',2.,2.*math.sin(i*2*math.pi/900),
                2.*math.cos(i*2*math.pi/900)-.013,17.,24,100.1+i/18000.) for i in range(1801))
            cloud_pub.publish(c);until(lambda:len(received)==4);expected=4
            for i in (0,450,900,1350,1800):
                x,y,z,intensity,ring,t=struct.unpack_from('<ffffHd',received[-1].data,26*i)
                assert abs(x-(1.9+i/9000.))<.0003,(i,x)
                assert intensity==17 and ring==24 and t==100.1
        cloud(100.2,[100.2],'unverified_frame')
        start=time.monotonic()
        while time.monotonic()-start<.15:rclpy.spin_once(node,timeout_sec=.01)
        assert len(received)==expected,'unverified frame must be rejected'
        print('deskew ROS layout, correction, wait, frame rejection and optional clock restoration passed')
    finally:
        process.send_signal(signal.SIGINT)
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:process.kill();process.wait()
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
