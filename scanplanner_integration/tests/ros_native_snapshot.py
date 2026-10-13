#!/usr/bin/env python3
"""Read-only replay of a captured static scene through native SCAN; no UDP sender."""
import argparse,json,math,os,signal,struct,subprocess,time
import numpy as np
from array import array
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import Odometry,Path as RosPath
from geometry_msgs.msg import PoseStamped,Twist
from sensor_msgs.msg import PointCloud2,PointField
from sensor_msgs_py.point_cloud2 import read_points_numpy
from std_msgs.msg import Bool,String,Float64MultiArray
from scan_planner_msgs.msg import Bspline
ROOT=Path(__file__).resolve().parents[2]
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);ap.add_argument('--map',type=Path,required=True);ap.add_argument('--capture',type=Path,required=True);ap.add_argument('--fitness',type=float,default=1.);ap.add_argument('--guide-weight',type=float,default=10000.);ap.add_argument('--corridor-weight',type=float,default=1000.);ap.add_argument('--lookahead',type=float,default=1.3);ap.add_argument('--max-acc',type=float,default=1.0);args=ap.parse_args()
 snapshot=json.loads((args.capture/'snapshot.json').read_text());raw=np.load(args.capture/'clouds.npz')['raw']
 packet_points=np.empty(len(raw),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('stamp','<f8'),('ring','<u2')])
 for i,k in enumerate(('x','y','z')):packet_points[k]=raw[:,i]
 packet_points['ring']=raw[:,3] if raw.shape[1]>3 else 0
 if os.environ.get('ROS_DOMAIN_ID','0')=='0':raise SystemExit('Use an isolated ROS domain')
 args.output.mkdir(parents=True,exist_ok=True);rclpy.init();n=rclpy.create_node('native_chain_test')
 odom=n.create_publisher(Odometry,'/test/odom',qos_profile_sensor_data);cloud=n.create_publisher(PointCloud2,'/test/cloud',qos_profile_sensor_data)
 path=n.create_publisher(RosPath,'/se2_global/path',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
 height=n.create_publisher(String,'/nvblox_robot/motion_status/raw',10);cancel=n.create_publisher(Bool,'/se2_scan/cancel',10)
 observed=[]
 n.create_subscription(PointCloud2,'/se2_scan/guard/observed_voxels',lambda m:observed.append(read_points_numpy(m,field_names=('x','y','z','resolution','state'),skip_nans=True)),qos_profile_sensor_data)
 rows=[];counts={'candidates':0,'verified':0};statuses=[];fsm=[];sample={}
 n.create_subscription(Twist,'/se2_scan/cmd_vel',lambda m:rows.append([time.monotonic(),m.linear.x,m.linear.y,m.angular.z]),qos_profile_sensor_data)
 for k,t in [('candidates','planning/bspline'),('verified','verified_bspline')]:
  def cb(m,k=k):
   counts[k]+=1
   if k=='candidates':sample['candidate']={'points':[[p.x,p.y,p.z] for p in m.pos_pts],'knots':list(m.knots),'order':m.order}
  n.create_subscription(Bspline,'/se2_scan/'+t,cb,100)
 def reference(m):
  if m.poses:sample['reference']=[[p.pose.position.x,p.pose.position.y,p.pose.position.z,2*math.atan2(p.pose.orientation.z,p.pose.orientation.w)] for p in m.poses]
 n.create_subscription(RosPath,'/se2_scan/reference',reference,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
 n.create_subscription(RosPath,'/se2_scan/planning/detour',lambda m:sample.update(detour=[[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in m.poses]),10)
 n.create_subscription(String,'/se2_scan/route_diagnostics',lambda m:sample.update(diagnostics=json.loads(m.data)),10)
 n.create_subscription(String,'/se2_scan/route_status',lambda m:statuses.append(m.data),100)
 n.create_subscription(Float64MultiArray,'/se2_scan/planning/state',lambda m:fsm.append(list(m.data)),100)
 procs=[];logs=[]
 def spawn(name,cmd):
  f=(args.output/(name+'.log')).open('w');logs.append(f);procs.append(subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True))
 spawn('metrics',[str(ROOT.parent/'se2-terrain-venv/bin/python'),str(ROOT/'scanplanner_integration/metrics.py'),str(args.output)])
 common=['--ros-args','--params-file',str(ROOT/'scanplanner_integration/planner.yaml'),'-r','__ns:=/se2_scan','-p',f'control_lookahead_s:={args.lookahead}','-p','max_forward_speed:=1.5','-p','max_lateral_speed:=0.4','-p','max_yaw_speed:=1.0','-p','manager.max_vel:=1.5','-p','optimization.max_vel:=1.5','-p',f'manager.max_acc:={args.max_acc}','-p',f'optimization.max_acc:={args.max_acc}','-p',f'optimization.lambda_fitness:={args.fitness}','-p',f'optimization.lambda_route_guide:={args.guide_weight}','-p',f'optimization.lambda_se2_corridor:={args.corridor_weight}']
 spawn('bridge',['ros2','run','se2_scan_bridge','scan_bridge',*common,'-r','raw_cloud:=/test/cloud','-r','raw_body_pose:=/test/odom','-r','cloud:=/unused','-r','sensor_pose:=/unused_pose','-r','grid_map/occupancy:=guard/occupancy','-p','grid_map.compute_inflation:=false','-p','grid_map.resolution:=0.05','-p','grid_map.max_ray_length:=4.0','-p','grid_map.sliding_map_size_x:=8.0','-p','grid_map.sliding_map_size_y:=8.0','-p','grid_map.sliding_map_size_z:=2.0'])
 spawn('optimizer',['ros2','run','scan_planner','scan_planner_node',*common,'-r','initial_path:=reference','-r','cloud:=world_cloud','-r','sensor_pose:=body_pose'])
 spawn('route',[str(ROOT.parent/'se2-terrain-venv/bin/python'),str(ROOT/'scanplanner_integration/route_node.py'),str(args.map),'--frame','nvblox_odom','--ros-args','-r','/nvblox_lio/odom:=/test/odom','-p','guard_resolution:=0.05'])
 body=snapshot['body'];body_height=snapshot['diagnostics']['height']
 def pose(v):
  p=PoseStamped();p.header.frame_id='nvblox_odom';p.header.stamp=n.get_clock().now().to_msg();p.pose.position.x,p.pose.position.y,p.pose.position.z=map(float,v[:3]);p.pose.orientation.z=math.sin(v[3]/2);p.pose.orientation.w=math.cos(v[3]/2);return p
 def pump(seconds,obstacle=False):
  end=time.monotonic()+seconds;last_cloud=last_height=0.
  while time.monotonic()<end:
   now=time.monotonic();o=Odometry();p=pose(body);o.header=p.header;o.child_frame_id='base_link_dog';o.pose.pose=p.pose
   if 'body_orientation' in snapshot:
    q=o.pose.pose.orientation;q.x,q.y,q.z,q.w=map(float,snapshot['body_orientation'])
   odom.publish(o)
   if now-last_height>.1:height.publish(String(data=json.dumps({'PatrolDevice':{'Items':{'MotionStatus':{'Height':body_height}}}})));last_height=now
   if now-last_cloud>.1:
    last_cloud=now;stamp=n.get_clock().now().nanoseconds/1e9
    packet_points['stamp']=stamp
    c=PointCloud2();c.header.frame_id='lidar_link';c.header.stamp=n.get_clock().now().to_msg();c.height=1;c.width=len(raw);c.point_step=22;c.row_step=22*len(raw);c.is_dense=True
    c.fields=[PointField(name=k,offset=i*4,datatype=7,count=1) for i,k in enumerate(['x','y','z'])]+[PointField(name='timestamp',offset=12,datatype=8,count=1)]
    if raw.shape[1]>3:c.fields.append(PointField(name='ring',offset=20,datatype=4,count=1))
    c.data=array('B',packet_points.tobytes());cloud.publish(c)
   rclpy.spin_once(n,timeout_sec=.003)
 def active_since(t):return [r for r in rows if r[0]>t and max(map(abs,r[1:]))>1e-5]
 result={}
 try:
  pump(12)
  p=RosPath();p.header.frame_id='nvblox_odom';p.header.stamp=n.get_clock().now().to_msg();p.poses=[pose(v) for v in snapshot['path']];path.publish(p)
  t=time.monotonic();pump(12);result['free']={'counts':counts.copy(),'commands':len(active_since(t)),'statuses':statuses[-6:]};print(json.dumps(result),flush=True)
  result['passed']=counts['verified']>0 and bool(active_since(t))
 finally:
  if observed:np.savez_compressed(args.output/'observed.npz',observed=observed[-1])
  sample.update(body=body,height=body_height);(args.output/'candidate.json').write_text(json.dumps(sample,indent=2))
  result.update(counts=counts,statuses=statuses[-30:],fsm_tail=fsm[-10:]);(args.output/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
  for p in reversed(procs):
   os.killpg(p.pid,signal.SIGINT)
   try:p.wait(8)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
  for f in logs:f.close()
  n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
