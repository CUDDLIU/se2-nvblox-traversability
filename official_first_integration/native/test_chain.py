#!/usr/bin/env python3
"""No-actuator end-to-end simulation for the original NOS planner via ROS."""
import json,math,os,signal,subprocess,sys,time
from pathlib import Path
import numpy as np
assert os.environ.get('ROS_DOMAIN_ID')=='86' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
import rclpy
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from nav_msgs.msg import Odometry,Path as RosPath
from geometry_msgs.msg import PoseStamped,TwistStamped
from sensor_msgs.msg import PointCloud2,PointField
from std_msgs.msg import Float64MultiArray,String
HERE=Path(__file__).resolve().parent
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True)
mode=sys.argv[2] if len(sys.argv)>2 else 'corner'
rclpy.init();node=rclpy.create_node('native_acceptance')
pubs={n:node.create_publisher(t,k,qos_profile_sensor_data) for n,t,k in [('odom',Odometry,'/nvblox_lio/odom'),('cloud',PointCloud2,'/LIDAR/POINTS_NX'),('tf',PoseStamped,'/se2_scan/map_transform'),('robot',Float64MultiArray,'/first/robot_state')]}
route_pub=node.create_publisher(RosPath,'/se2_global/path',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
commands=[];statuses=[];clouds=[];command=[0.,0.,0.];command_at=[0.]
def cmd(m):
    command[:]=[m.twist.linear.x,m.twist.linear.y,m.twist.angular.z];command_at[0]=time.monotonic();commands.append([time.monotonic(),*command])
node.create_subscription(TwistStamped,'/first/cmd_vel_stamped',cmd,qos_profile_sensor_data)
node.create_subscription(String,'/first/status',lambda m:statuses.append(json.loads(m.data)),10)
node.create_subscription(PointCloud2,'/first/occupancy',lambda m:clouds.append(m.width*m.height),qos_profile_sensor_data)
adapter_file=HERE.parent/'source/adapter.py' if os.environ.get('NOS_TEST_ADAPTER')=='source' else HERE/'adapter.py'
f=(out/'adapter.log').open('w');proc=subprocess.Popen([sys.executable,str(adapter_file),'--output',str(out/'native')],stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
body=np.zeros(3);trace=[];sent=False;start=time.monotonic();last=start;last_cloud=0.;goal_at=None
# L corridor: 1.6 m wide, with a closed outer corner and a visible opening.
xy=[]
for x in np.arange(-1.,3.21,.025):xy.append((x,-.8))
for x in np.arange(-1.,1.61,.025):xy.append((x,.8))
for y in np.arange(-.8,4.01,.025):xy.append((3.2,y))
for y in np.arange(.8,4.01,.025):xy.append((1.6,y))
world=np.array([[x,y,z] for x,y in xy for z in np.arange(-.15,.16,.025)],dtype=float)
try:
    while time.monotonic()-start<60:
        t=time.monotonic();dt=min(t-last,.05);last=t
        if mode=='corner' and t-command_at[0]<.15:
            vx,vy,w=command;c,s=math.cos(body[2]),math.sin(body[2]);body+=np.array([c*vx-s*vy,s*vx+c*vy,w])*dt
        stamp=node.get_clock().now().to_msg()
        m=PoseStamped();m.header.frame_id='se2_scan_map';m.header.stamp=stamp;m.pose.orientation.w=1.
        elapsed=t-goal_at if goal_at else -1.
        if mode!='gates' or not 6.<elapsed<8.3:pubs['tf'].publish(m)
        o=Odometry();o.header.frame_id='nvblox_odom';o.child_frame_id='base_link_dog';o.header.stamp=stamp;o.pose.pose.position.x,o.pose.pose.position.y=map(float,body[:2]);o.pose.pose.orientation.z=math.sin(body[2]/2);o.pose.pose.orientation.w=math.cos(body[2]/2);o.twist.twist.linear.x=command[0];o.twist.twist.linear.y=command[1];o.twist.twist.angular.z=command[2];pubs['odom'].publish(o)
        pubs['robot'].publish(Float64MultiArray(data=[.57,12290.,17.,.01,.01,0.]))
        if t-last_cloud>.1 and (mode!='gates' or not 2.<elapsed<3.):
            c,s=math.cos(body[2]),math.sin(body[2]);p=world.copy();p[:,:2]-=body[:2];p[:,:2]=p[:,:2]@np.array([[c,-s],[s,c]])
            p=p[(np.abs(p[:,0])<4)&(np.abs(p[:,1])<4)].astype('<f4')
            m=PointCloud2();m.header.frame_id='base_link_dog';m.header.stamp=stamp;m.height=1;m.width=len(p);m.point_step=12;m.row_step=len(p)*12;m.is_dense=True;m.fields=[PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(('x','y','z'))];m.data=p.tobytes();pubs['cloud'].publish(m);last_cloud=t
        if not sent and statuses and t-start>12:
            path=RosPath();path.header.frame_id='se2_scan_map';path.header.stamp=stamp
            points=[(x,0.,0.) for x in np.arange(0,2.41,.1)]+[(2.4,y,math.pi/2) for y in np.arange(.1,3.01,.1)]
            if mode=='gates':points=[(x,0.,0.) for x in np.arange(0,3.01,.1)]
            for x,y,a in points:
                p=PoseStamped();p.header=path.header;p.pose.position.x=float(x);p.pose.position.y=float(y);p.pose.position.z=-.57;p.pose.orientation.z=math.sin(a/2);p.pose.orientation.w=math.cos(a/2);path.poses.append(p)
            route_pub.publish(path);sent=True;goal_at=t
        if mode=='gates' and goal_at:
            if elapsed>4. and 'cancelled' not in locals():
                cancelled=True;empty=RosPath();empty.header=path.header;route_pub.publish(empty)
            if elapsed>4.5 and 'resent' not in locals():
                resent=True;path.header.stamp=stamp;route_pub.publish(path)
            if elapsed>10.:break
        rclpy.spin_once(node,timeout_sec=.005);trace.append([t-start,*body])
        if proc.poll() is not None:raise RuntimeError((out/'adapter.log').read_text())
        if statuses and statuses[-1]['state']=='arrived':break
finally:
    os.killpg(proc.pid,signal.SIGINT)
    try:proc.wait(timeout=6)
    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
    f.close();node.destroy_node();rclpy.try_shutdown()
    report=dict(body=body.tolist(),trace=trace,commands=commands,statuses=statuses,cloud_counts=clouds)
    (out/'result.json').write_text(json.dumps(report))
from collections import Counter
print(json.dumps(dict(body=body.tolist(),holds=dict(Counter(s['hold_reason'] for s in statuses)),cloud_count=max(clouds,default=0),command_max=np.max(np.abs(np.array(commands)[:,1:]),axis=0).tolist() if commands else [])))
assert proc.returncode==0,('adapter shutdown',proc.returncode,(out/'adapter.log').read_text())
assert any(s['command_valid'] for s in statuses),'no valid native command'
if mode=='corner':
    assert any(s['state']=='arrived' for s in statuses),'corner did not reach goal'
    collisions=0
    for _,x,y,a in trace:
        c,si=math.cos(a),math.sin(a);p=np.array(xy)-[x,y];p=p@np.array([[c,-si],[si,c]])
        collisions+=int(np.any((np.abs(p[:,0])<.42)&(np.abs(p[:,1])<.25)))
    assert collisions==0,f'{collisions} footprint/wall overlaps'
else:
    def window(a,b):return [v[1:] for v in commands if a<v[0]-goal_at<b]
    for a,b in ((2.5,3.),(4.2,4.5),(7.8,8.3)):
        rows=window(a,b);assert rows and all(max(map(abs,v))<1e-7 for v in rows),(a,b,rows)
    for a,b in ((.5,2.),(3.4,3.9),(5.,5.7),(8.7,9.8)):
        rows=window(a,b);assert any(v[0]>.1 for v in rows),(a,b,rows)
    print('cloud timeout, recovery, cancel/new goal, localization timeout/recovery: PASS')
