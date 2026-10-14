"""ROS saved-goal integration test. Simulated telemetry, no actuator process."""
import json,math,os,signal,subprocess,sys,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'global_planner'))
from navigation_modes import envelope,mode,arc_lengths
assert os.environ.get('ROS_DOMAIN_ID')=='86' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
import rclpy
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from nav_msgs.msg import Odometry,Path as RosPath
from geometry_msgs.msg import PoseStamped,TwistStamped
from sensor_msgs.msg import PointCloud2,PointField,Imu
import numpy as np
from runtime import cloud_message
from std_msgs.msg import String,Int32,Float64MultiArray

auto=len(sys.argv)>2 and sys.argv[2]=='auto'
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True)
rclpy.init();node=rclpy.create_node('source_saved_goal_acceptance')
pubs={k:node.create_publisher(t,topic,qos_profile_sensor_data) for k,t,topic in (
    ('lidar_odom',Odometry,'/nvblox_lio/odom_lidar'),('odom',Odometry,'/nvblox_lio/odom'),('tf',PoseStamped,'/se2_scan/map_transform'),
    ('body',PointCloud2,'/nvblox_lio/body_cloud'),('imu',Imu,'/IMU'),
    ('cloud',PointCloud2,'/LIDAR/POINTS_NX'),('robot',Float64MultiArray,'/first/robot_state'))}
saved=node.create_publisher(String,'/first/saved_goal',10)
route_pub=node.create_publisher(String,'/se2_global/navigation_route',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
class Routes:
    def publish(self,m):
        rows=[[p.pose.position.x,p.pose.position.y,p.pose.position.z,
               2*math.atan2(p.pose.orientation.z,p.pose.orientation.w)] for p in m.poses]
        route_pub.publish(String(data=json.dumps(envelope(rows,[mode(0,arc_lengths(rows)[-1])] if rows else [],
            (m.header.stamp.sec,m.header.stamp.nanosec),m.header.frame_id))))
routes=Routes()
commands=[];statuses=[];requests=[];global_requests=[]
node.create_subscription(PoseStamped,'/se2_global/goal_pose',lambda m:global_requests.append(m),10)
node.create_subscription(TwistStamped,'/first/cmd_vel_stamped',lambda m:commands.append((time.monotonic(),m.twist.linear.x,m.twist.linear.y,m.twist.angular.z)),qos_profile_sensor_data)
node.create_subscription(String,'/first/status',lambda m:statuses.append(json.loads(m.data)),10)
node.create_subscription(Int32,'/first/request_gait',lambda m:requests.append(m.data),1)
f=(out/'adapter.log').open('w')
proc=subprocess.Popen([sys.executable,str(Path(__file__).with_name('adapter.py')),'--output',str(out/'planner')],stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
began=time.monotonic();task_at=None;clear_at=None;cancel_at=None;new_at=None;last=began;x=y=a=0.;last_cloud=0.
try:
    while time.monotonic()-began<25:
        now=time.monotonic();dt=min(now-last,.05);last=now
        elapsed=now-task_at if task_at else -1
        if commands and now-commands[-1][0]<.15:
            _,vx,vy,w=commands[-1];x+=dt*(math.cos(a)*vx-math.sin(a)*vy);y+=dt*(math.sin(a)*vx+math.cos(a)*vy);a+=dt*w
        stamp=node.get_clock().now().to_msg()
        tf=PoseStamped();tf.header.frame_id='se2_scan_map';tf.header.stamp=stamp;tf.pose.orientation.w=1.;pubs['tf'].publish(tf)
        odom=Odometry();odom.header.frame_id='nvblox_odom';odom.child_frame_id='base_link_dog';odom.header.stamp=stamp
        odom.pose.pose.position.z=.55;odom.pose.pose.position.x=x;odom.pose.pose.position.y=y;odom.pose.pose.orientation.z=math.sin(a/2);odom.pose.pose.orientation.w=math.cos(a/2);pubs['odom'].publish(odom);pubs['lidar_odom'].publish(odom)
        pending=12291. if 1.5<elapsed<3. else 0.
        actual=12291. if elapsed>2. and clear_at is None else 12290.
        pubs['robot'].publish(Float64MultiArray(data=[.57,actual,17.,.01,.01,pending]))
        if now-last_cloud>.09:
            cloud=cloud_message(np.array([[px,py,-.55] for px in np.arange(-2,3,.06) for py in np.arange(-1,1,.06)],np.float32),odom.header)
            pubs['cloud'].publish(cloud);pubs['body'].publish(cloud);last_cloud=now
            ns=stamp.sec*1000000000+stamp.nanosec
            for delta in (-10000000,10000000):
                imu=Imu();imu.header.stamp.sec,imu.header.stamp.nanosec=divmod(ns+delta,1000000000);imu.orientation=odom.pose.pose.orientation;pubs['imu'].publish(imu)
        if task_at is None and statuses and now-began>2:
            task_at=now
            saved.publish(String(data=json.dumps(dict(frame_id='se2_scan_map',PosX=1.2,PosY=0.,PosZ=0.,AngleYaw=0.,PointInfo=0,Gait=12291,Speed=1,Manner=0,ObsMode=0,NavMode=1 if auto else 0))))
        if auto and global_requests and task_at and now-task_at>.2 and 'reset_sent' not in locals():
            reset_sent=True;empty=RosPath();empty.header=tf.header;routes.publish(empty)
        if auto and global_requests and task_at and now-task_at>.3 and 'wrong_sent' not in locals():
            wrong_sent=True;wrong=RosPath();wrong.header=tf.header
            p=PoseStamped();p.header=tf.header;p.pose.position.x=1.2;p.pose.position.z=3.;p.pose.orientation.w=1.;wrong.poses=[p];routes.publish(wrong)
        if auto and global_requests and task_at and now-task_at>.6 and 'responded' not in locals():
            responded=True;path=RosPath();path.header=tf.header
            for px in (0.,.4,.8,1.2):
                p=PoseStamped();p.header=path.header;p.pose.position.x=px;p.pose.orientation.w=1.;path.poses.append(p)
            routes.publish(path)
        if task_at and statuses and statuses[-1]['state']=='arrived' and clear_at is None:
            clear_at=now
            path=RosPath();path.header=tf.header
            for px in (x,4.):
                p=PoseStamped();p.header=path.header;p.pose.position.x=px;p.pose.orientation.w=1.;path.poses.append(p)
            routes.publish(path);new_at=now
        if new_at and now-new_at>1.5 and cancel_at is None:
            cancel_at=now;empty=RosPath();empty.header=tf.header;routes.publish(empty)
        if cancel_at and now-cancel_at>1.:break
        for _ in range(5):rclpy.spin_once(node,timeout_sec=.002)
        if proc.poll() is not None:raise RuntimeError((out/'adapter.log').read_text())
finally:
    os.killpg(proc.pid,signal.SIGINT)
    try:proc.wait(timeout=5)
    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
    f.close();node.destroy_node();rclpy.try_shutdown()
    (out/'result.json').write_text(json.dumps(dict(commands=commands,statuses=statuses,gaits=requests)))
assert proc.returncode==0,(out/'adapter.log').read_text()
assert task_at and clear_at and cancel_at,'saved goal failed to arrive or subsequent goal failed'
assert 12291 in requests,'no saved gait request'
assert all(max(map(abs,c[1:]))==0 for c in commands if task_at+.3<c[0]<task_at+3.),'moved before fresh gait confirmation'
assert any(s.get('planner_mode')==('dwa' if auto else 'line') and s['command_valid'] and s.get('task_gait')==12291 for s in statuses)
assert all(abs(c[1])<=(.50001 if auto else .80001) for c in commands if task_at<c[0]<clear_at),'wrong saved speed profile'
assert any(s.get('planner_mode')=='dwa' and s['command_valid'] and s.get('task_gait') is None and s['planner_parameters']['maxSpeedY']==1. for s in statuses),'saved profile leaked into ASA goal'
assert all(max(map(abs,c[1:]))==0 for c in commands if c[0]>cancel_at+.3),'old task moved after cancellation'
print(json.dumps(dict(passed=True,scope='saved gait hold, pending confirmation, source profile, arrival, fresh RViz goal and cancellation; simulated telemetry')))
