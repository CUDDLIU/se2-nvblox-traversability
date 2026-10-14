#!/usr/bin/env python3
"""ROS transport for the reconstructed AOS height-map component.

Standalone inspection entry for the same frontend used in source stair mode.
Publishes height obstacles only; does not publish motion/gait requests.
Cloud and full odometry must refer to the same LIO scan and body frame.
"""
import argparse,copy,json,signal,sys,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu,PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32,String
from std_srvs.srv import SetBool
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from runtime import xyz_cloud,cloud_message
from height_frontend import HeightFrontend

def key(m):return (m.header.stamp.sec,m.header.stamp.nanosec)
def stamp(k):return k[0]+k[1]*1e-9
def xyzw(q):return (q.x,q.y,q.z,q.w)

class HeightAdapter:
    def __init__(self,node,output):
        self.node=node;self.frontend=HeightFrontend();self.clouds={};self.odoms={}
        self.last=(-1,0);self.errors={};self.frames=0;self.output=Path(output)
        self.output.mkdir(parents=True,exist_ok=True)
        self.trace=(self.output/'height_frames.jsonl').open('w')
        self.nav=node.create_publisher(PointCloud2,'/first/source_height/cloud_nav',1)
        self.ground=node.create_publisher(PointCloud2,'/first/source_height/cloud_gravity',1)
        self.status=node.create_publisher(String,'/first/source_height/status',1)
        self.tf=TransformBroadcaster(node)
        node.create_subscription(PointCloud2,'/nvblox_lio/body_cloud',self.cloud,qos_profile_sensor_data)
        node.create_subscription(Odometry,'/nvblox_lio/odom_lidar',self.odom,qos_profile_sensor_data)
        node.create_subscription(Imu,'/IMU',self.imu,QoSProfile(depth=200,reliability=ReliabilityPolicy.BEST_EFFORT))
        node.create_subscription(Int32,'/first/source_height/terrain',lambda m:self.frontend.policy.set_terrain(m.data),1)
        node.create_service(SetBool,'/first/source_height/stair_oa',self.stair_oa)
        node.create_timer(.1,self.tick)
    def stair_oa(self,request,response):
        self.frontend.policy.set_stair_oa(int(request.data));response.success=True
        response.message='STAIR_OA avoids stairs; it is not a gait/climbing switch'
        return response
    def imu(self,m):
        try:self.frontend.add_imu(stamp(key(m)),xyzw(m.orientation));self.errors.pop('imu',None)
        except ValueError as exc:self.errors['imu']=str(exc)
    def cloud(self,m):
        if m.header.frame_id not in ('base_link_dog','base_link'):self.errors['cloud']='registered cloud must be in the body frame';return
        self.clouds[key(m)]=m;self.errors.pop('cloud',None)
        while len(self.clouds)>5:self.clouds.pop(next(iter(self.clouds)))
    def odom(self,m):
        if m.child_frame_id not in ('base_link_dog','base_link'):self.errors['odom']='odometry child must be the body frame';return
        self.odoms[key(m)]=m;self.errors.pop('odom',None)
        while len(self.odoms)>30:self.odoms.pop(next(iter(self.odoms)))
    def tick(self):
        keys=[k for k in self.clouds.keys()&self.odoms.keys() if k>self.last]
        if not keys:
            self.status.publish(String(data=json.dumps(dict(state='waiting_synchronized_body_scan',frames=self.frames,errors=self.errors))))
            return
        k=max(keys);m=self.clouds[k];odom=self.odoms[k];t=time.monotonic()
        try:
            if odom.child_frame_id!=m.header.frame_id:raise ValueError('cloud and odometry body frames differ')
            p=odom.pose.pose.position;q=odom.pose.pose.orientation
            ground=self.frontend.process(xyz_cloud(m),stamp(k),(p.x,p.y,p.z,*xyzw(q)))
            result=self.frontend.height_map();header=copy.deepcopy(m.header);header.frame_id='first_height/gravity'
            for publisher,points in ((self.nav,result['cloud_nav']),(self.ground,ground)):
                cloud=cloud_message(points,header);cloud.header=header;publisher.publish(cloud)
            tf=TransformStamped();tf.header=copy.deepcopy(m.header);tf.child_frame_id=header.frame_id
            g=self.frontend.rotation();g[:3]*=-1
            tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=map(float,g)
            self.tf.sendTransform(tf)
            self.frames+=1;self.errors.pop('process',None)
            data=dict(state='height_map_ready',frames=self.frames,stamp=stamp(k),gravity_matched=result['gravity_matched'],
                      accumulated_points=len(ground),obstacle_cells=len(result['cloud_nav']),pitch_rad=self.frontend.pitch,
                      terrain=self.frontend.policy.terrain,stair_oa=self.frontend.policy.stair_oa,
                      height_range=self.frontend.policy.height_range,process_ms=(time.monotonic()-t)*1000,errors=self.errors)
            self.trace.write(json.dumps(data)+'\n');self.trace.flush();self.status.publish(String(data=json.dumps(data)))
        except (ValueError,RuntimeError,StopIteration,TypeError) as exc:
            self.errors['process']=str(exc);self.status.publish(String(data=json.dumps(dict(state='invalid_input',errors=self.errors))))
        finally:
            self.last=k;self.clouds={s:v for s,v in self.clouds.items() if s>k};self.odoms={s:v for s,v in self.odoms.items() if s>k}
    def close(self):self.frontend.close();self.trace.close()

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=rclpy.create_node('first_source_height');adapter=HeightAdapter(node,a.output)
    running=True
    def stop(*_):
        nonlocal running
        running=False
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    try:
        while running and rclpy.ok():rclpy.spin_once(node,timeout_sec=.05)
    finally:adapter.close();node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
