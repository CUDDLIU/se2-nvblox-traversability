"""Whole source stair chain with synthetic geometry and a localhost UDP robot.

Includes ASA mesh annotation, full pose/cloud/IMU, gait acknowledgement, sensor
loss, upper/lower landing, and cancellation. No real robot interface is opened.
This checks software execution, not contact dynamics or physical climbing.
"""
import argparse,importlib.util,json,math,os,signal,socket,subprocess,sys,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
assert os.environ.get('ROS_DOMAIN_ID')=='86' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'global_planner'))
from navigation_modes import TerrainAnnotations,envelope
from test_navigation_modes import plane
spec=importlib.util.spec_from_file_location('udp_protocol',ROOT/'scanplanner_integration/core.py')
protocol=importlib.util.module_from_spec(spec);sys.modules[spec.name]=protocol;spec.loader.exec_module(protocol)
import rclpy
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped,TwistStamped
from sensor_msgs.msg import PointCloud2,PointField,Imu
from std_msgs.msg import String,Empty
from runtime import cloud_message
ap=argparse.ArgumentParser();ap.add_argument('output',type=Path);ap.add_argument('--down',action='store_true');ap.add_argument('--long-stairs',action='store_true');args=ap.parse_args()
out=args.output;out.mkdir(parents=True,exist_ok=True)
steps=12 if args.long_stairs else 3
edges=1.7+np.arange(steps)*.3;top=float(steps*.15);goal_x=float(edges[-1]+2.7)
blocks={0:plane(-4,edges[0],0)}
for i,edge in enumerate(edges):blocks[i+1]=plane(edge,edges[i+1] if i+1<steps else goal_x+4,(i+1)*.15)
poses=[[0,0,0,0],*[[float(edge),0,(i+1)*.15,0] for i,edge in enumerate(edges)],[goal_x,0,top,0]]
if args.down:poses=[[*p[:3],math.pi] for p in poses[::-1]]
modes=TerrainAnnotations(blocks).annotate(poses)
assert [m['terrain'] for m in modes]==['flat','stair_down' if args.down else 'stair_up','flat'],modes
xx,yy=np.meshgrid(np.arange(-4,goal_x+4,.043),np.arange(-1.5,1.5,.043));zz=.15*sum((xx>=edge).astype(int) for edge in edges)
world=np.c_[xx.ravel(),yy.ravel(),zz.ravel()]
def ground(x):return float(np.interp(x,np.ravel(np.c_[edges-.1,edges+.1]),np.ravel(np.c_[np.arange(steps)*.15,(np.arange(steps)+1)*.15])))
rclpy.init();node=rclpy.create_node('source_stair_execution_acceptance')
pubs={k:node.create_publisher(t,topic,qos_profile_sensor_data) for k,t,topic in (
 ('lidar_odom',Odometry,'/nvblox_lio/odom_lidar'),('odom',Odometry,'/nvblox_lio/odom'),('tf',PoseStamped,'/se2_scan/map_transform'),
 ('cloud',PointCloud2,'/LIDAR/POINTS_NX'),('body',PointCloud2,'/nvblox_lio/body_cloud'),('imu',Imu,'/IMU'))}
routes=node.create_publisher(String,'/se2_global/navigation_route',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
cancel=node.create_publisher(Empty,'/first/cancel',10)
statuses=[];commands=[];udp=[];requests=[];visual=[]
node.create_subscription(String,'/first/status',lambda m:statuses.append(dict(json.loads(m.data),test_time=time.monotonic())),10)
node.create_subscription(TwistStamped,'/first/cmd_vel_stamped',lambda m:commands.append((time.monotonic(),m.twist.linear.x,m.twist.linear.y,m.twist.angular.z)),qos_profile_sensor_data)
node.create_subscription(PointCloud2,'/first/occupancy',lambda m:visual.append((m.header.frame_id,m.width)),qos_profile_sensor_data)
sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.bind(('127.0.0.1',0));sock.setblocking(False)
processes=[];logs=[]
for name,cmd in (
 ('adapter',[str(Path(__file__).with_name('adapter.py')),'--output',str(out/'planner')]),
 ('robot',[str(ROOT/'official_first_integration/robot.py'),'--source-planner','--loopback-test','--host','127.0.0.1','--port',str(sock.getsockname()[1]),'--execute','--output',str(out/'robot')])):
 f=(out/(name+'.log')).open('w');logs.append(f);processes.append(subprocess.Popen([sys.executable,*cmd],stdout=f,stderr=subprocess.STDOUT,start_new_session=True))
began=last=time.monotonic();x=goal_x if args.down else 0.;y=0.;a=math.pi if args.down else 0.;last_scan=last_telemetry=0.;peer=None;seq=0;actual=wanted=0x3002;request_at=0.;sent=arrived=cancelled=loss=None;restored=False;original=None
try:
 while time.monotonic()-began<42:
  now=time.monotonic();dt=min(now-last,.05);last=now
  while True:
   try:data,peer=sock.recvfrom(65536)
   except BlockingIOError:break
   _,doc=protocol.decode_packet(data)
   if (doc['Type'],doc['Command'])==(2,25):udp.append((now,doc['Items'],actual))
   if (doc['Type'],doc['Command'])==(2,23):
    wanted=doc['Items']['GaitParam'];request_at=now;requests.append((now,wanted))
  if udp and now-udp[-1][0]<.15:
   v=udp[-1][1];x+=dt*(math.cos(a)*v['X']-math.sin(a)*v['Y']);y+=dt*(math.sin(a)*v['X']+math.cos(a)*v['Y']);a+=dt*v['Yaw']
  if now-request_at>.65:actual=wanted
  if peer and now-last_telemetry>.05:
   sock.sendto(protocol.packet(1002,6,dict(BasicStatus=dict(ControlUsageMode=1,HES=0,MotionState=17,Direction=0,Sleep=0,Gait=actual)),seq),peer);seq+=1
   sock.sendto(protocol.packet(1002,4,dict(MotionStatus=dict(Height=.55)),seq),peer);seq+=1;last_telemetry=now
  stamp=node.get_clock().now().to_msg()
  tf=PoseStamped();tf.header.frame_id='se2_scan_map';tf.header.stamp=stamp;tf.pose.orientation.w=1.;pubs['tf'].publish(tf)
  if now-last_scan>.09:
   stamp=node.get_clock().now().to_msg();ns=stamp.sec*1000000000+stamp.nanosec
   pitch=((.42 if args.long_stairs else .18)*(1 if args.down else -1)) if edges[0]-.1<x<edges[-1]+.1 else 0.
   rot=Rotation.from_euler('ZYX',[a,pitch,0.]);q=rot.as_quat();z=ground(x)+.55
   odom=Odometry();odom.header.frame_id='nvblox_odom';odom.child_frame_id='base_link_dog';odom.header.stamp=stamp
   odom.pose.pose.position.x=x;odom.pose.pose.position.y=y;odom.pose.pose.position.z=z
   odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w=map(float,q)
   pubs['odom'].publish(odom);pubs['lidar_odom'].publish(odom)
   if not loss or now-loss>1.15:
    for offset in (-10000000,10000000):
     imu=Imu();imu.header.stamp.sec,imu.header.stamp.nanosec=divmod(ns+offset,1000000000);imu.orientation=odom.pose.pose.orientation;pubs['imu'].publish(imu)
    if loss:restored=True
   near=world[(np.abs(world[:,0]-x)<3.9)&(np.abs(world[:,1]-y)<2.)]
   points=rot.inv().apply(near-[x,y,z]).astype(np.float32)
   cloud=cloud_message(points,odom.header);pubs['cloud'].publish(cloud);pubs['body'].publish(cloud);last_scan=now
  if sent is None and now-began>2 and statuses and peer:
   original=envelope(poses,modes,(stamp.sec,stamp.nanosec),'se2_scan_map');routes.publish(String(data=json.dumps(original)));sent=now
  if loss is None and statuses and statuses[-1].get('perception_mode')=='aos_height' and statuses[-1].get('command_valid') and statuses[-1].get('height_frames',0)>=3:loss=now
  if statuses and statuses[-1]['state']=='arrived' and arrived is None:
   arrived=now
   # Reusing an old route or a display path must not restart the completed task.
   routes.publish(String(data=json.dumps(original)))
  if arrived and now-arrived>.3 and cancelled is None:cancelled=now;cancel.publish(Empty())
  if cancelled and now-cancelled>.4 and 'late' not in locals():
   late=dict(original,stamp=[stamp.sec,stamp.nanosec]);routes.publish(String(data=json.dumps(late)))
  if cancelled and now-cancelled>1.:break
  for _ in range(6):rclpy.spin_once(node,timeout_sec=.001)
  for p in processes:
   if p.poll() is not None:raise RuntimeError('child exited; see logs')
finally:
 for p in processes:
  if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
 for p in processes:
  try:p.wait(timeout=7)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
 for f in logs:f.close()
 sock.close();node.destroy_node();rclpy.try_shutdown()
 (out/'result.json').write_text(json.dumps(dict(statuses=statuses,commands=commands,udp=udp,gaits=requests,modes=modes,final=[x,y,ground(x)],loss=loss)))
assert all(p.returncode==0 for p in processes),'process failure'
assert arrived and cancelled,('failed to arrive',x,y,statuses[-1] if statuses else None)
sequence=[]
for s in statuses:
 if s['command_valid'] and s.get('asa_mode'):
  terrain=s['asa_mode']['terrain']
  if not sequence or sequence[-1]!=terrain:sequence.append(terrain)
  if terrain!='flat':
   assert s['perception_mode']=='aos_height' and s['height_frames']>0 and s['gravity_matched']
   assert s['actual_gait']==0x3003 and s['height_range']==.25 and s['height_stair_oa'] is False
assert sequence==[m['terrain'] for m in modes],sequence
assert loss and restored and any(loss+.5<s['test_time']<loss+1.1 and not s['command_valid'] for s in statuses),'missing IMU did not stop'
for start,gait in requests:
 pending=[v for t,v,_ in udp if start+.18<t<start+.60]
 assert pending and all(v['X']==v['Y']==v['Yaw']==0 for v in pending),'motion before hardware gait ACK'
tail=[v for t,v,_ in udp if t>cancelled+.2]
assert tail and all(v['X']==v['Y']==v['Yaw']==0 for v in tail),'motion after cancellation'
assert any(frame=='first_source/gravity' for frame,_ in visual),'no gravity-frame RViz cloud'
report=dict(passed=True,direction='down' if args.down else 'up',steps=steps,sequence=sequence,final=[x,y,ground(x)],
 height_frames=max(s.get('height_frames',0) for s in statuses),max_height_ms=max(s.get('height_process_ms',0) for s in statuses),
 gait_requests=[g for _,g in requests],udp_packets=len(udp),imu_loss_stop=True,cancel_stop=True,
 scope='ASA mesh annotation -> source height perception -> source NOS planner -> gait confirmation -> real robot.py -> localhost UDP; synthetic geometry, no contact dynamics')
(out/'summary.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
