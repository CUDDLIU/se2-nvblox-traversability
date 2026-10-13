#!/usr/bin/env python3
"""Exercise SCAN -> map verification -> bridge, with no UDP sender."""
import argparse,json,math,os,signal,struct,subprocess,time
import numpy as np
from array import array
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import Odometry,Path as RosPath
from geometry_msgs.msg import PoseStamped,Twist
from sensor_msgs.msg import PointCloud2,PointField
from std_msgs.msg import Bool,String,Float64MultiArray
from scan_planner_msgs.msg import Bspline
from sensor_msgs_py.point_cloud2 import read_points_numpy
ROOT=Path(__file__).resolve().parents[2]
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);ap.add_argument('--map',type=Path,required=True);args=ap.parse_args()
 if os.environ.get('ROS_DOMAIN_ID','0')=='0':raise SystemExit('Use an isolated ROS domain')
 args.output.mkdir(parents=True,exist_ok=True);rclpy.init();n=rclpy.create_node('native_chain_test')
 odom=n.create_publisher(Odometry,'/test/odom',qos_profile_sensor_data);cloud=n.create_publisher(PointCloud2,'/test/cloud',qos_profile_sensor_data)
 path=n.create_publisher(RosPath,'/se2_global/path',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
 height=n.create_publisher(String,'/nvblox_robot/motion_status/raw',10);cancel=n.create_publisher(Bool,'/se2_scan/cancel',10)
 rows=[];counts={'candidates':0,'verified':0};statuses=[];fsm=[];debug={}
 n.create_subscription(Twist,'/se2_scan/cmd_vel',lambda m:rows.append([time.monotonic(),m.linear.x,m.linear.y,m.angular.z]),qos_profile_sensor_data)
 for k,t in [('candidates','planning/bspline'),('verified','verified_bspline')]:
  def cb(m,k=k):
   counts[k]+=1
   debug[k]={'points':[[p.x,p.y,p.z] for p in m.pos_pts],'knots':list(m.knots),'order':m.order}
  n.create_subscription(Bspline,'/se2_scan/'+t,cb,100)
 n.create_subscription(String,'/se2_scan/route_status',lambda m:statuses.append(m.data),100)
 n.create_subscription(Float64MultiArray,'/se2_scan/planning/state',lambda m:fsm.append(list(m.data)),100)
 n.create_subscription(String,'/se2_scan/control_status',lambda m:debug.update(control=m.data),10)
 n.create_subscription(PointCloud2,'/se2_scan/guard/collision_occupancy',lambda m:debug.update(collision=read_points_numpy(m,field_names=('x','y','z','resolution'),skip_nans=True).tolist()),qos_profile_sensor_data)
 procs=[];logs=[]
 def spawn(name,cmd):
  f=(args.output/(name+'.log')).open('w');logs.append(f);procs.append(subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True))
 spawn('metrics',[str(ROOT.parent/'se2-terrain-venv/bin/python'),str(ROOT/'scanplanner_integration/metrics.py'),str(args.output)])
 common=['--ros-args','--params-file',str(ROOT/'scanplanner_integration/planner.yaml'),'-r','__ns:=/se2_scan','-p','control_lookahead_s:=1.3','-p','max_forward_speed:=1.5','-p','max_lateral_speed:=0.4','-p','max_yaw_speed:=1.0','-p','manager.max_vel:=1.5','-p','optimization.max_vel:=1.5']
 spawn('bridge',['ros2','run','se2_scan_bridge','scan_bridge',*common,'-r','raw_cloud:=/test/cloud','-r','raw_body_pose:=/test/odom','-r','cloud:=/unused','-r','sensor_pose:=/unused_pose','-r','grid_map/occupancy:=guard/occupancy','-p','grid_map.compute_inflation:=false','-p','grid_map.resolution:=0.05','-p','grid_map.max_ray_length:=4.0','-p','grid_map.sliding_map_size_x:=8.0','-p','grid_map.sliding_map_size_y:=8.0','-p','grid_map.sliding_map_size_z:=2.0'])
 spawn('optimizer',['ros2','run','scan_planner','scan_planner_node',*common,'-r','initial_path:=reference','-r','cloud:=world_cloud','-r','sensor_pose:=body_pose'])
 spawn('route',[str(ROOT.parent/'se2-terrain-venv/bin/python'),str(ROOT/'scanplanner_integration/route_node.py'),str(args.map),'--frame','nvblox_odom','--ros-args','-r','/nvblox_lio/odom:=/test/odom','-p','guard_resolution:=0.05'])
 body=[.35,-.08564458,0.,0.];goal=[.75,-.08564458,0.,0.]
 def pose(v):
  p=PoseStamped();p.header.frame_id='nvblox_odom';p.header.stamp=n.get_clock().now().to_msg();p.pose.position.x,p.pose.position.y,p.pose.position.z=map(float,v[:3]);p.pose.orientation.z=math.sin(v[3]/2);p.pose.orientation.w=math.cos(v[3]/2);return p
 def pump(seconds,obstacle=False):
  end=time.monotonic()+seconds;last_cloud=last_height=0.
  while time.monotonic()<end:
   now=time.monotonic();o=Odometry();p=pose(body);o.header=p.header;o.child_frame_id='base_link_dog';o.pose.pose=p.pose;odom.publish(o)
   if now-last_height>.1:height.publish(String(data=json.dumps({'PatrolDevice':{'Items':{'MotionStatus':{'Height':.57}}}})));last_height=now
   if now-last_cloud>.1:
    last_cloud=now;stamp=n.get_clock().now().nanoseconds/1e9
    pts=[(x*.10,y*.10,-.57,stamp) for x in range(-22,23) for y in range(-22,23)]
    x=.44 if obstacle else 3.0
    pts += ([(x,y*.06,z*.05,stamp) for y in range(-20,21) for z in range(-2,8)] if obstacle else
            [(x,y*.01*x/.44,z*.01*x/.44,stamp) for y in range(-125,126) for z in range(-15,41)])
    c=PointCloud2();c.header.frame_id='lidar_link';c.header.stamp=n.get_clock().now().to_msg();c.height=1;c.width=len(pts);c.point_step=20;c.row_step=20*len(pts);c.is_dense=True
    c.fields=[PointField(name=k,offset=i*4,datatype=7,count=1) for i,k in enumerate(['x','y','z'])]+[PointField(name='timestamp',offset=12,datatype=8,count=1)];packet=np.array(pts,dtype=float);packed=np.empty(len(pts),dtype=[('xyz','<f4',(3,)),('stamp','<f8')]);packed['xyz']=packet[:,:3];packed['stamp']=packet[:,3];c.data=array('B',packed.tobytes());cloud.publish(c)
   rclpy.spin_once(n,timeout_sec=.003)
 def active_since(t):return [r for r in rows if r[0]>t and max(map(abs,r[1:]))>1e-5]
 result={}
 try:
  pump(12)
  p=RosPath();p.header.frame_id='nvblox_odom';p.header.stamp=n.get_clock().now().to_msg();p.poses=[pose([v[0],v[1],v[2]-.57,v[3]]) for v in [body,goal]];path.publish(p)
  t=time.monotonic();pump(12);result['free']={'counts':counts.copy(),'commands':len(active_since(t)),'statuses':statuses[-6:]};print(json.dumps(result),flush=True)
  assert counts['verified']>0 and active_since(t),result
  t=time.monotonic();f=len(fsm);pump(6,True);result['blocked_commands_after_settle']=len(active_since(t+3));result['optimizer_stopped_with_same_reference']=any((v[3]==1 or v[2]==5) and v[0]>0 and abs(v[0]-fsm[f-1][0])<1e-6 for v in fsm[f:])
  assert result['blocked_commands_after_settle']==0 and result['optimizer_stopped_with_same_reference'],result
  # The requested horizon ends at the obstacle. A stopped planner must resume
  # after free rays clear it, without another clicked goal or adapter maneuver.
  before=counts['verified'];t=time.monotonic();pump(8);result['resumed_verified']=counts['verified']-before;result['resumed_commands']=len(active_since(t+3))
  assert result['resumed_verified']>0 and result['resumed_commands']>0,result
  cancel.publish(Bool(data=True));t=time.monotonic();pump(2);result['cancel_commands']=len(active_since(t+.5));assert result['cancel_commands']==0,result
  result['passed']=True
 finally:
  (args.output/'debug.json').write_text(json.dumps(debug))
  result.update(counts=counts,statuses=statuses[-30:],fsm_tail=fsm[-10:]);(args.output/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
  for p in reversed(procs):
   os.killpg(p.pid,signal.SIGINT)
   try:p.wait(8)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
  for f in logs:f.close()
  n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
