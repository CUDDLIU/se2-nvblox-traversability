#!/usr/bin/env python3
"""Isolated regression: two coincident hits keep both physical ray origins."""
import argparse,json,os,signal,struct,subprocess,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2,PointField
from sensor_msgs_py.point_cloud2 import read_points_numpy
ROOT=Path(__file__).resolve().parents[2]
ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
if os.environ.get('ROS_DOMAIN_ID','0')=='0':raise SystemExit('Requires isolated ROS domain')
args.output.mkdir(parents=True,exist_ok=True);rclpy.init();n=rclpy.create_node('test_ray_origins')
pub=n.create_publisher(PointCloud2,'/test/cloud',qos_profile_sensor_data);op=n.create_publisher(Odometry,'/test/odom',qos_profile_sensor_data);data={}
def receive(k,fields,m):data[k]=read_points_numpy(m,field_names=fields,skip_nans=True)
n.create_subscription(PointCloud2,'/se2_scan/world_cloud',lambda m:receive('world',('x','y','z','origin_x','origin_y','origin_z'),m),qos_profile_sensor_data)
n.create_subscription(PointCloud2,'/se2_scan/guard/observed_voxels',lambda m:receive('observed',('x','y','z','resolution','state'),m),qos_profile_sensor_data)
f=(args.output/'bridge.log').open('w')
p=subprocess.Popen(['ros2','run','se2_scan_bridge','scan_bridge','--ros-args','--params-file',str(ROOT/'scanplanner_integration/planner.yaml'),'-r','__ns:=/se2_scan','-r','raw_cloud:=/test/cloud','-r','raw_body_pose:=/test/odom','-r','cloud:=/unused','-r','sensor_pose:=/unused_pose','-p','grid_map.resolution:=0.05','-p','grid_map.compute_inflation:=false'],stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
result={}
try:
 end=time.monotonic()+7
 while time.monotonic()<end:
  stamp=n.get_clock().now().to_msg();seconds=stamp.sec+stamp.nanosec/1e9
  o=Odometry();o.header.stamp=stamp;o.header.frame_id='nvblox_odom';o.child_frame_id='base_link_dog';o.pose.pose.orientation.w=1.;op.publish(o)
  c=PointCloud2();c.header=o.header;c.header.frame_id='lidar_link';c.height=1;c.width=2;c.point_step=22;c.row_step=44;c.is_dense=True
  c.fields=[PointField(name=k,offset=i*4,datatype=7,count=1) for i,k in enumerate(('x','y','z'))]+[PointField(name='timestamp',offset=12,datatype=8,count=1),PointField(name='ring',offset=20,datatype=4,count=1)]
  c.data=struct.pack('<fffdHfffdH',1.,1.,0.,seconds,0,1.,1.,0.,seconds,96);pub.publish(c)
  until=time.monotonic()+.1
  while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.01)
 assert 'world' in data and 'observed' in data,'missing diagnostic data'
 world=data['world'];assert len(world)==2,'coincident endpoints discarded one sensor ray'
 np.testing.assert_allclose(world[:,:3],[[1,1,0],[1,1,0]])
 np.testing.assert_allclose(world[:,3:],[[.32028,0,-.013],[-.32028,0,-.013]],atol=1e-6)
 a=data['observed'];ids=np.floor(a[:,:3]/.05+1e-5).astype(int)
 states={tuple(k):int(v) for k,v in zip(ids,a[:,4])}
 assert states.get((12,8,-1))==0,'front physical ray is not free'
 assert states.get((2,2,-1),-1)==-1,'unobserved body-centre ray was carved'
 assert states.get((20,20,0))==1,'real endpoint lost'
 result={'passed':True,'world_points':len(world),'observed_voxels':len(a),'phantom_central_ray':'unknown','front_ray':'free','hit':'occupied'}
 print(json.dumps(result),flush=True)
finally:
 (args.output/'result.json').write_text(json.dumps(result));os.killpg(p.pid,signal.SIGINT)
 try:p.wait(8)
 except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
 f.close();n.destroy_node();rclpy.shutdown()
