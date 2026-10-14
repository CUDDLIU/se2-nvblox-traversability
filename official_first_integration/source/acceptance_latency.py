"""Recorded full-size clouds through ROS/source transport; no actuator sender."""
import argparse,array,json,os,signal,subprocess,sys,time
from pathlib import Path
import numpy as np
assert os.environ.get('ROS_DOMAIN_ID')=='86' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
import rclpy
from rclpy.serialization import deserialize_message
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String,Float64MultiArray
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'global_planner'))
from navigation_modes import envelope,mode
ap=argparse.ArgumentParser();ap.add_argument('capture',type=Path);ap.add_argument('output',type=Path);ap.add_argument('--reference',action='store_true');args=ap.parse_args()
args.output.mkdir(parents=True,exist_ok=True)
clouds=[deserialize_message(f.read_bytes(),PointCloud2) for f in sorted(args.capture.glob('raw_*.cdr'))];assert clouds
rclpy.init();node=rclpy.create_node('recorded_cloud_latency_acceptance')
pubs={k:node.create_publisher(t,topic,qos_profile_sensor_data) for k,t,topic in (
 ('cloud',PointCloud2,'/LIDAR/POINTS_NX'),('odom',Odometry,'/nvblox_lio/odom'),('tf',PoseStamped,'/se2_scan/map_transform'),('robot',Float64MultiArray,'/first/robot_state'))}
routes=node.create_publisher(String,'/se2_global/navigation_route',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL));statuses=[]
node.create_subscription(String,'/first/status',lambda m:statuses.append(dict(json.loads(m.data),test_time=time.monotonic())),10)
script=Path(__file__).with_name('adapter.py')
if args.reference:
 code="import sys,runpy;sys.path.insert(0,sys.argv[1]);from core import Core;original=Core.cloud;Core.cloud=lambda self,xyz,passability=False:original(self,xyz,passability,reference=True);sys.argv=sys.argv[2:];runpy.run_path(sys.argv[0],run_name='__main__')"
 cmd=[sys.executable,'-c',code,str(script.parent),str(script)]
else:cmd=[sys.executable,str(script)]
log=(args.output/'adapter.log').open('w');proc=subprocess.Popen([*cmd,'--output',str(args.output/'planner')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
start=time.monotonic();scan=0;last_scan=last_odom=0.;sent=None
try:
 while time.monotonic()-start<14:
  now=time.monotonic();stamp=node.get_clock().now().to_msg()
  if now-last_odom>.025:
   tf=PoseStamped();tf.header.frame_id='se2_scan_map';tf.header.stamp=stamp;tf.pose.orientation.w=1.;pubs['tf'].publish(tf)
   odom=Odometry();odom.header.frame_id='nvblox_odom';odom.child_frame_id='base_link_dog';odom.header.stamp=stamp;odom.pose.pose.position.z=.55;odom.pose.pose.orientation.w=1.;pubs['odom'].publish(odom)
   pubs['robot'].publish(Float64MultiArray(data=[.55,float(0x3002),17.,.01,.01,0.]));last_odom=now
  if now-last_scan>.099:
   m=clouds[scan%len(clouds)];f=next(f for f in m.fields if f.name=='timestamp');buf=bytearray(m.data)
   ts=np.ndarray((m.height,m.width),dtype='<f8',buffer=buf,offset=f.offset,strides=(m.row_step,m.point_step));old=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
   header_ns=time.time_ns()-110000000;ts+=header_ns*1e-9-old;m.header.stamp.sec,m.header.stamp.nanosec=divmod(header_ns,10**9);m.data=array.array('B',buf)
   pubs['cloud'].publish(m);last_scan=now;scan+=1
  if sent is None and now-start>2 and statuses:
   routes.publish(String(data=json.dumps(envelope([[0,0,0,0],[5,0,0,0]],[mode(0,5)],(stamp.sec,stamp.nanosec),'se2_scan_map'))));sent=now
  for _ in range(3):rclpy.spin_once(node,timeout_sec=.001)
  if proc.poll() is not None:raise RuntimeError('adapter exited')
finally:
 if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
 try:proc.wait(timeout=7)
 except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
 log.close();node.destroy_node();rclpy.try_shutdown()
 (args.output/'statuses.json').write_text(json.dumps(statuses))
assert proc.returncode==0,'adapter failed'
rows=[s for s in statuses if sent and s['test_time']>sent+2];assert rows
from collections import Counter
report=dict(reference=args.reference,samples=len(rows),states=dict(Counter(s['state'] for s in rows)),valid_fraction=float(np.mean([s['command_valid'] for s in rows])))
for key in ('perception_ms','source_cloud_age_s','cycle_ms','planning_ms','odom_age_s'):
 report[key]={k:float(v) for k,v in zip(('p50','p95','max'),np.quantile([s.get(key,0) for s in rows],[.5,.95,1]))}
(args.output/'summary.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
if not args.reference:
 assert report['valid_fraction']>.95,report
 assert report['source_cloud_age_s']['p95']<.30,report
 assert report['cycle_ms']['p95']<100,report
