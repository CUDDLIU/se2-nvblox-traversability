"""Atomic ASA route + three gait segments, simulated ROS only, no actuator."""
import json,math,os,signal,subprocess,sys,time
from pathlib import Path
assert os.environ.get('ROS_DOMAIN_ID')=='86' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'global_planner'))
from navigation_modes import envelope,mode
import rclpy
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from nav_msgs.msg import Odometry,Path as RosPath
from geometry_msgs.msg import PoseStamped,TwistStamped
from sensor_msgs.msg import PointCloud2,PointField,Imu
import numpy as np
from runtime import cloud_message
from std_msgs.msg import String,Int32,Float64MultiArray,Empty

out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True)
rclpy.init();node=rclpy.create_node('source_route_modes_acceptance')
pubs={k:node.create_publisher(t,topic,qos_profile_sensor_data) for k,t,topic in (
    ('lidar_odom',Odometry,'/nvblox_lio/odom_lidar'),('odom',Odometry,'/nvblox_lio/odom'),('tf',PoseStamped,'/se2_scan/map_transform'),
    ('body',PointCloud2,'/nvblox_lio/body_cloud'),('imu',Imu,'/IMU'),
    ('cloud',PointCloud2,'/LIDAR/POINTS_NX'),('robot',Float64MultiArray,'/first/robot_state'))}
routes=node.create_publisher(String,'/se2_global/navigation_route',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
plain=node.create_publisher(RosPath,'/se2_global/path',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
cancel=node.create_publisher(Empty,'/first/cancel',10)
commands=[];statuses=[];requests=[];actual=0x3002;wanted=actual;request_at=0.
def gait(m):
    global wanted,request_at
    requests.append(m.data)
    if m.data and m.data!=wanted:wanted=m.data;request_at=time.monotonic()
node.create_subscription(Int32,'/first/request_gait',gait,1)
node.create_subscription(TwistStamped,'/first/cmd_vel_stamped',lambda m:commands.append((time.monotonic(),m.twist.linear.x,m.twist.linear.y,m.twist.angular.z)),qos_profile_sensor_data)
node.create_subscription(String,'/first/status',lambda m:statuses.append(json.loads(m.data)),10)
f=(out/'adapter.log').open('w')
proc=subprocess.Popen([sys.executable,str(Path(__file__).with_name('adapter.py')),'--output',str(out/'planner')],stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
began=last=time.monotonic();x=y=a=0.;last_cloud=0.;sent=arrived=cancelled=None;waiting=[];stale_sent=False
try:
    while time.monotonic()-began<24:
        now=time.monotonic();dt=min(now-last,.05);last=now
        if commands and now-commands[-1][0]<.15:
            _,vx,vy,w=commands[-1];x+=dt*(math.cos(a)*vx-math.sin(a)*vy);y+=dt*(math.sin(a)*vx+math.cos(a)*vy);a+=dt*w
        if now-request_at>.6:actual=wanted
        if actual!=wanted:waiting.append(now)
        stamp=node.get_clock().now().to_msg()
        tf=PoseStamped();tf.header.frame_id='se2_scan_map';tf.header.stamp=stamp;tf.pose.orientation.w=1.;pubs['tf'].publish(tf)
        odom=Odometry();odom.header.frame_id='nvblox_odom';odom.child_frame_id='base_link_dog';odom.header.stamp=stamp
        odom.pose.pose.position.z=.55;odom.pose.pose.position.x=x;odom.pose.pose.position.y=y;odom.pose.pose.orientation.z=math.sin(a/2);odom.pose.pose.orientation.w=math.cos(a/2);pubs['odom'].publish(odom);pubs['lidar_odom'].publish(odom)
        pubs['robot'].publish(Float64MultiArray(data=[.55,float(actual),17.,.01,.01,float(wanted if actual!=wanted else 0)]))
        if now-last_cloud>.09:
            cloud=cloud_message(np.array([[px,py,-.55] for px in np.arange(-2,3,.06) for py in np.arange(-1,1,.06)],np.float32),odom.header)
            pubs['cloud'].publish(cloud);pubs['body'].publish(cloud);last_cloud=now
            ns=stamp.sec*1000000000+stamp.nanosec
            for delta in (-10000000,10000000):
                imu=Imu();imu.header.stamp.sec,imu.header.stamp.nanosec=divmod(ns+delta,1000000000);imu.orientation=odom.pose.pose.orientation;pubs['imu'].publish(imu)
        if sent is None and now-began>2 and statuses:
            sent=now;original=envelope([[0,0,0,0],[5,0,0,0]],
                [mode(0,1.4),mode(1.4,3.2,'stair_up'),mode(3.2,5)],(stamp.sec,stamp.nanosec),'se2_scan_map')
            routes.publish(String(data=json.dumps(original)))
        if sent and x>2 and not stale_sent:
            stale_sent=True;routes.publish(String(data=json.dumps(original)))
            # Display-only route cannot cancel or replace the atomic one.
            empty=RosPath();empty.header=tf.header;plain.publish(empty)
        if sent and statuses and statuses[-1]['state']=='arrived' and arrived is None:
            arrived=now
            routes.publish(String(data=json.dumps(envelope([[x,0,0,0],[8,0,0,0]],
                [mode(0,8-x,'stair_up')],(stamp.sec,stamp.nanosec),'se2_scan_map'))))
        if arrived and now-arrived>.25 and cancelled is None:
            cancelled=now;cancel.publish(Empty())
        if cancelled and now-cancelled>.4 and 'late_sent' not in locals():
            late_sent=True
            # A result already computing when cancel arrived must not restart.
            late=dict(original,stamp=[stamp.sec,stamp.nanosec]);routes.publish(String(data=json.dumps(late)))
        if cancelled and now-cancelled>1:break
        for _ in range(5):rclpy.spin_once(node,timeout_sec=.002)
        if proc.poll() is not None:raise RuntimeError((out/'adapter.log').read_text())
finally:
    os.killpg(proc.pid,signal.SIGINT)
    try:proc.wait(timeout=5)
    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
    f.close();node.destroy_node();rclpy.try_shutdown()
    (out/'result.json').write_text(json.dumps(dict(commands=commands,statuses=statuses,gaits=requests)))
assert proc.returncode==0,(out/'adapter.log').read_text()
assert arrived and cancelled,'route failed to arrive/cancel'
sequence=[]
for s in statuses:
    if s['command_valid'] and s.get('asa_mode'):
        terrain=s['asa_mode']['terrain']
        if not sequence or sequence[-1]!=terrain:sequence.append(terrain)
        assert s['planner_parameters']['judge_close'] is False
        assert s['planner_mode']=='dwa'
assert sequence==['flat','stair_up','flat'],sequence
for s in statuses:
    if s.get('requested_gait') and s.get('actual_gait')!=s['requested_gait']:
        assert not s['command_valid'],'motion before gait confirmed'
assert all(max(map(abs,c[1:]))==0 for c in commands if c[0]>cancelled+.2)
result=dict(passed=True,sequence=sequence,scope='atomic route mode switching, stale/display route ignored, gait confirmation and cancel; no physical stair simulation')
(out/'summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
