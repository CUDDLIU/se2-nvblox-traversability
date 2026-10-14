"""Replay the failed real ASA route and mesh through the source ROS adapter.

Odometry follows the recorded route (open loop); clouds sample the actual mesh,
without sensor occlusion or contact physics. Gait ACK is simulated. This proves
mode/perception handoff, not physical climb completion. No actuator is opened.
"""
import argparse,json,math,os,signal,subprocess,sys,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
assert os.environ.get('ROS_DOMAIN_ID')=='86' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'global_planner'))
from navigation_modes import TerrainAnnotations,arc_lengths,envelope
import rclpy
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped,TwistStamped
from sensor_msgs.msg import Imu,PointCloud2
from std_msgs.msg import String,Int32,Float64MultiArray,Empty
from runtime import cloud_message
ap=argparse.ArgumentParser();ap.add_argument('output',type=Path);ap.add_argument('--down',action='store_true');args=ap.parse_args()
out=args.output;out.mkdir(parents=True,exist_ok=True)
fixture=ROOT/'global_planner/fixtures/stair_scene_20261014'
doc=json.loads((fixture/'routes.json').read_text());poses=np.array(doc['queries'][-1]['poses'])
if args.down:poses=poses[::-1].copy()
with np.load(fixture/'mesh_corridors.npz') as mesh:
 vertices=mesh['vertices'];triangles=mesh['triangles'];classifier=TerrainAnnotations({0:(vertices,triangles)});faces=vertices[triangles]
modes=classifier.annotate(poses);arc=arc_lengths(poses);keep=np.r_[np.diff(arc)>1e-9,True]
world=np.concatenate([faces[:,0]*(1-a-b)+faces[:,1]*a+faces[:,2]*b for a in np.arange(5)/4 for b in np.arange(5)/4 if a+b<=1])
world=world[np.unique(np.round(world/.025).astype(int),axis=0,return_index=True)[1]]
rclpy.init();node=rclpy.create_node('captured_stair_route_replay')
pubs={k:node.create_publisher(t,topic,qos_profile_sensor_data) for k,t,topic in (
 ('odom',Odometry,'/nvblox_lio/odom'),('lidar_odom',Odometry,'/nvblox_lio/odom_lidar'),('tf',PoseStamped,'/se2_scan/map_transform'),
 ('body',PointCloud2,'/nvblox_lio/body_cloud'),('raw',PointCloud2,'/LIDAR/POINTS_NX'),('imu',Imu,'/IMU'),('robot',Float64MultiArray,'/first/robot_state'))}
routes=node.create_publisher(String,'/se2_global/navigation_route',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
cancel=node.create_publisher(Empty,'/first/cancel',10)
actual=wanted=0x3002;request_at=0.;requests=[];statuses=[];commands=[]
def gait(m):
 global wanted,request_at
 if m.data and m.data!=wanted:wanted=m.data;request_at=time.monotonic();requests.append([request_at,wanted])
node.create_subscription(Int32,'/first/request_gait',gait,1)
node.create_subscription(String,'/first/status',lambda m:statuses.append(dict(json.loads(m.data),test_time=time.monotonic())),10)
node.create_subscription(TwistStamped,'/first/cmd_vel_stamped',lambda m:commands.append([time.monotonic(),m.twist.linear.x,m.twist.linear.y,m.twist.angular.z]),qos_profile_sensor_data)
f=(out/'adapter.log').open('w');proc=subprocess.Popen([sys.executable,str(Path(__file__).with_name('adapter.py')),'--output',str(out/'planner')],stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
began=last=time.monotonic();s=0.;last_scan=0.;sent=cancelled=None
try:
 while time.monotonic()-began<36:
  now=time.monotonic();dt=min(.05,now-last);last=now
  # Advance only after the software has acquired the selected profile. This
  # is imposed odometry, explicitly not an integration of planner velocities.
  if sent and now-sent>1 and actual==wanted and statuses and statuses[-1].get('command_valid'):s=min(arc[-1]-.05,s+.7*dt)
  if now-request_at>.65:actual=wanted
  xyz=np.array([np.interp(s,arc[keep],poses[keep,k]) for k in range(3)]);xyz[2]+=.55
  a=float(np.interp(s,arc[keep],np.unwrap(poses[keep,3])));rot=Rotation.from_euler('z',a);q=rot.as_quat()
  stamp=node.get_clock().now().to_msg();tf=PoseStamped();tf.header.frame_id='se2_scan_map';tf.header.stamp=stamp;tf.pose.orientation.w=1.;pubs['tf'].publish(tf)
  odom=Odometry();odom.header.frame_id='nvblox_odom';odom.child_frame_id='base_link_dog';odom.header.stamp=stamp
  odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z=map(float,xyz)
  odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w=map(float,q);pubs['odom'].publish(odom)
  pubs['robot'].publish(Float64MultiArray(data=[.55,float(actual),17.,.01,.01,float(wanted if wanted!=actual else 0)]))
  if now-last_scan>.10:
   pubs['lidar_odom'].publish(odom);ns=stamp.sec*1000000000+stamp.nanosec
   for delta in (-10000000,10000000):
    imu=Imu();imu.header.stamp.sec,imu.header.stamp.nanosec=divmod(ns+delta,1000000000);imu.orientation=odom.pose.pose.orientation;pubs['imu'].publish(imu)
   near=world[(np.linalg.norm(world[:,:2]-xyz[:2],axis=1)<3.)&(abs(world[:,2]-xyz[2])<1.8)]
   cloud=cloud_message(rot.inv().apply(near-xyz).astype(np.float32),odom.header);pubs['body'].publish(cloud);pubs['raw'].publish(cloud);last_scan=now
  if sent is None and now-began>2 and statuses:
   routes.publish(String(data=json.dumps(envelope(poses.tolist(),modes,(stamp.sec,stamp.nanosec),'se2_scan_map'))));sent=now
  if sent and s>=arc[-1]-.06 and cancelled is None:cancelled=now;cancel.publish(Empty())
  if cancelled and now-cancelled>1.:break
  for _ in range(4):rclpy.spin_once(node,timeout_sec=.002)
  if proc.poll() is not None:raise RuntimeError('adapter exited')
finally:
 if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
 try:proc.wait(timeout=7)
 except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
 f.close();node.destroy_node();rclpy.try_shutdown()
 (out/'result.json').write_text(json.dumps(dict(modes=modes,statuses=statuses,commands=commands,gaits=requests,progress=s)))
assert proc.returncode==0,'adapter failed'
assert cancelled,('replay incomplete',s,statuses[-1] if statuses else None)
sequence=[]
for status in statuses:
 if status.get('command_valid') and status.get('asa_mode'):
  terrain=status['asa_mode']['terrain']
  if not sequence or sequence[-1]!=terrain:sequence.append(terrain)
  if terrain!='flat':
   assert status['perception_mode']=='aos_height' and status['height_frames']>0 and status['gravity_matched']
   assert status['actual_gait']==0x3003 and status['height_range']==.25 and status['height_stair_oa'] is False
 if status.get('requested_gait') and status['actual_gait']!=status['requested_gait']:assert not status['command_valid']
assert sequence==[m['terrain'] for m in modes],sequence
assert all(max(map(abs,c[1:]))==0 for c in commands if c[0]>cancelled+.2)
stairs=[s for s in statuses if s.get('asa_mode',{} ) and s['asa_mode']['terrain']!='flat']
report=dict(passed=True,scope=__doc__,direction='down' if args.down else 'up',modes=modes,sequence=sequence,
 annotation=classifier.diagnostics,height_frames=max(s.get('height_frames',0) for s in statuses),
 first_stair_progress=min(s['route_progress_m'] for s in stairs),gaits=[r[1] for r in requests],
 planner_states=sorted({s.get('planner_state') for s in stairs}),max_height_ms=max(s.get('height_process_ms',0) for s in statuses))
(out/'summary.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
