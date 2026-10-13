#!/usr/bin/env python3
"""Isolated domain only. Synthetic data; no robot socket or actuator process."""
import json,math,os,struct,subprocess,time,tempfile,sys
from pathlib import Path
assert os.environ.get('ROS_DOMAIN_ID')=='86' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
import rclpy
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from geometry_msgs.msg import PoseStamped,TwistStamped
from nav_msgs.msg import Odometry,Path as RosPath
from sensor_msgs.msg import PointCloud2,PointField
from std_msgs.msg import Float64MultiArray,Int32,String
STAIRS='--stairs' in sys.argv
NO_SUPPORT='--no-support' in sys.argv
HEIGHT_OFFSET='--height-offset' in sys.argv
POSTURE='--posture' in sys.argv
ROUTE_PROGRESS='--route-progress' in sys.argv
HERE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(HERE));from run import session_remaps,ros_args,stop_children
SESSION='--session' in sys.argv
remaps=session_remaps('ros_chain') if SESSION else {}
def topic(name):return remaps.get(name,name)
root=HERE/('runtime/isolated_no_support' if NO_SUPPORT else 'runtime/isolated_stairs' if STAIRS else 'runtime/isolated');root.mkdir(parents=True,exist_ok=True)
terrain=root/'flat.terrain'
with terrain.open('w') as f:
    f.write('FIRST_TERRAIN 1 0.05\n')
    for x in range(-100,101):
        for y in range(-100,101):
            if NO_SUPPORT and (x<99 or y<99):continue
            f.write(f'{x} {y} {(-.42 if STAIRS and x>=40 else -.57)-(.15 if HEIGHT_OFFSET else 0.)} 0\n')
rclpy.init();node=rclpy.create_node('first_acceptance')
pubs={
 'pose':node.create_publisher(Odometry,'/nvblox_lio/odom',qos_profile_sensor_data),
 'tf':node.create_publisher(PoseStamped,'/se2_scan/map_transform',qos_profile_sensor_data),
 'robot':node.create_publisher(Float64MultiArray,topic('/first/robot_state'),qos_profile_sensor_data),
 'cloud':node.create_publisher(PointCloud2,'/LIDAR/POINTS_NX',qos_profile_sensor_data),
 'route':node.create_publisher(RosPath,topic('/se2_global/path'),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))}
foreign_route=node.create_publisher(RosPath,'/se2_global/path',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)) if SESSION else None
received=[];statuses=[];timing=[];gaits=[]
node.create_subscription(TwistStamped,topic('/first/cmd_vel_stamped'),lambda m:received.append((time.monotonic(),m.twist.linear.x,m.twist.linear.y,m.twist.angular.z)),qos_profile_sensor_data)
node.create_subscription(String,'/first/status',lambda m:statuses.append(json.loads(m.data)),10)
node.create_subscription(Float64MultiArray,'/first/control_timing',lambda m:timing.append(list(m.data)),100)
node.create_subscription(Int32,topic('/first/request_gait'),lambda m:gaits.append(m.data),1)
log=(root/'node.log').open('w');proc=subprocess.Popen([str(HERE/'build/first_node'),*ros_args(remaps),'-p','library:='+str(HERE/'vendor/paths/first'),'-p','terrain:='+str(terrain)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)

def route(empty=False,goal_x=4.,goal_yaw=0.,publisher=None):
    m=RosPath();m.header.frame_id='se2_scan_map';m.header.stamp=node.get_clock().now().to_msg()
    if not empty:
        for i in range(41):
            p=PoseStamped();p.header=m.header;p.pose.position.x=goal_x*i/40;p.pose.position.z=-.42 if STAIRS and p.pose.position.x>=2 else -.57;p.pose.orientation.w=math.cos(goal_yaw/2);p.pose.orientation.z=math.sin(goal_yaw/2);m.poses.append(p)
    (publisher or pubs['route']).publish(m)

body_x=0.
def feed(seconds,blocked=False,cloud=True,clear=False,gait=0x3002,move_to=None,low_return=None,height=.57,z_override=None,pending=0,map_shift=0.):
    global body_x
    start=time.monotonic();last_scan=0
    while time.monotonic()-start<seconds:
        now=node.get_clock().now();stamp=now.to_msg()
        if move_to is not None:body_x+=max(-.015,min(.015,move_to-body_x))
        body_z=.15 if STAIRS and body_x>=2 else 0.
        if z_override is not None:body_z=z_override
        tf=PoseStamped();tf.header.frame_id='se2_scan_map';tf.header.stamp=stamp;tf.pose.orientation.w=1.;tf.pose.position.x=map_shift;pubs['tf'].publish(tf)
        o=Odometry();o.header.frame_id='nvblox_odom';o.child_frame_id='base_link_dog';o.header.stamp=stamp;o.pose.pose.orientation.w=1.;o.pose.pose.position.x=body_x;o.pose.pose.position.z=body_z;pubs['pose'].publish(o)
        pubs['robot'].publish(Float64MultiArray(data=[height,float(gait),17.,.01,.01,float(pending)]))
        if cloud and time.monotonic()-last_scan>.1:
            m=PointCloud2();m.header.frame_id='base_link_dog';m.header.stamp=stamp;m.height=1;m.point_step=24;m.is_dense=True
            m.fields=[PointField(name=n,offset=k,datatype=t,count=1) for n,k,t in [('x',0,7),('y',4,7),('z',8,7),('timestamp',12,8),('ring',20,4)]]
            points=[]
            for i in range(360):
                a=i*math.pi/180;d=.2 if blocked else 3.
                # For blocked scene use an enclosing wall intersecting every template.
                x=d*math.cos(a);y=d*math.sin(a);z=.425 if blocked else -height
                if clear:
                    # The input is float32: at +/-0.2 m it can fall across
                    # a voxel boundary from the Python double. Aim through
                    # the cell actually hit, not the unquantized test point.
                    bx=(math.floor(struct.unpack("<f",struct.pack("<f",.2*math.cos(a)))[0]/.05)+.5)*.05
                    by=(math.floor(struct.unpack("<f",struct.pack("<f",.2*math.sin(a)))[0]/.05)+.5)*.05
                    x,y,z=bx,by,.425
                if clear:
                    ox=.32028 if i%192<96 else -.32028
                    x=ox+20*(bx-ox);y=20*by;z=-.013+20*(.425+.013)
                points.append(struct.pack('<fffdHxx',x,y,z,now.nanoseconds/1e9,i%192))
            if low_return is not None:
                points.append(struct.pack('<fffdHxx',.425,0.,low_return,now.nanoseconds/1e9,0))
            m.data=b''.join(points);m.width=len(points);m.row_step=m.width*m.point_step;pubs['cloud'].publish(m);last_scan=time.monotonic()
        rclpy.spin_once(node,timeout_sec=.01)
        if proc.poll() is not None:raise RuntimeError('first_node exited: '+(root/'node.log').read_text()[-2000:])
try:
    if SESSION:
        route(publisher=foreign_route)
        deadline=time.monotonic()+20
        while not (received and statuses and pubs['route'].get_subscription_count()) and time.monotonic()<deadline:feed(.2)
        assert received and statuses,'FIRST startup/discovery timed out: '+(root/'node.log').read_text()[-1000:]
        feed(.5)
        assert all(not any(x[1:]) for x in received),'foreign retained route triggered motion'
        assert statuses[-1]['state']=='idle',statuses[-5:]
    if POSTURE:
        feed(1.5,gait=0x1001)
        assert statuses[-1]['cloud_age_s']<.35,'stable standing did not populate obstacle preview'
        assert all(not any(x[1:]) for x in received[-10:]),'standing preview commanded motion'
        route();feed(1.5,gait=0x1001,pending=0x3002,low_return=-.425)
        assert 0x3002 in gaits,'bootstrap waited for a cloud before requesting gait'
        assert all(not any(x[1:]) for x in received[-10:]),'unconfirmed gait commanded motion'
        feed(.2,height=.50,z_override=-.07,low_return=-.29)
        assert all(not any(x[1:]) for x in received[-4:]),'lowering posture commanded motion'
        feed(2,height=.43,z_override=-.14)
        assert any(x[1]>.5 for x in received[-20:]),'transition scan left a ghost obstacle'
        feed(1.5,blocked=True,height=.43,z_override=-.14)
        assert all(not any(x[1:]) for x in received[-10:]),'real obstacle not checked after posture settled'
    elif ROUTE_PROGRESS:
        feed(2);route(goal_x=10.);feed(1)
        feed(7,move_to=4.5);feed(.5)
        assert abs(body_x-4.5)<.001,body_x
        assert any(x[1]>.5 for x in received[-20:]),statuses[-5:]
        feed(1.5,map_shift=.04)
        assert statuses[-1]['state']=='tracking',statuses[-8:]
        assert any(x[1]>.5 for x in received[-20:]),'map correction lost route progress'
        # A posture refresh also invalidates the old command/evidence, without
        # resetting the fixed-map route to its first three metres.
        feed(.5,gait=0x1001,pending=0x3002,map_shift=.04)
        feed(1.5,map_shift=.04)
        assert statuses[-1]['state']=='tracking',statuses[-8:]
        route(True);feed(.5,map_shift=.04)
        assert all(not any(x[1:]) for x in received[-10:]),'cancellation leaked motion'
        route(goal_x=10.);feed(1,map_shift=.04)
        assert statuses[-1]['state']=='route_disconnected','new route did not reset progress'
    elif HEIGHT_OFFSET:
        feed(2);route();feed(3)
        # Old ground is 15 cm lower. A low return still lies within the
        # unchanged 10 cm live-ground threshold; a real higher hit blocks.
        feed(1.5,low_return=-.495)
        assert any(x[1]>.1 for x in received[-20:]),'saved height offset created a false obstacle'
        feed(1.5,low_return=-.445)
        assert all(not any(x[1:]) for x in received[-10:]),'higher measured obstacle was erased'
    elif STAIRS:
        feed(2);route();feed(2)
        assert 0x3003 in gaits,'no advance stair gait request'
        assert any(x[1]>.1 for x in received[-30:]),statuses[-5:]
        # Pending gait must not stop normal approach; entry must wait for telemetry.
        feed(2,move_to=1.5);feed(.5)
        assert all(not any(x[1:]) for x in received[-10:]),'unconfirmed stair entry'
        assert any(s['state']=='waiting_stair_confirmation' for s in statuses[-8:])
        feed(1,gait=0x3003)
        assert any(x[1]>.1 for x in received[-20:]),statuses[-5:]
        feed(2,gait=0x3003,move_to=2.7);feed(.5,gait=0x3003)
        assert gaits[-1]==0x3002,'flat gait not requested after full body clears step'
    else:
        feed(2);route();feed(3);free=received[-40:];assert any(x[1]>.5 for x in free),statuses[-5:]
        assert any(s.get('continuity_active') for s in statuses[-10:]),'valid native commands did not enable continuity'
        route(True);feed(.7);assert all(not any(x[1:]) for x in received[-10:]),'cancel leaked command'
        assert not statuses[-1].get('continuity_active'),'cancel retained continuity'
        route();feed(1);feed(1,cloud=False);assert all(not any(x[1:]) for x in received[-10:]),'stale cloud leaked command'
        feed(1.5,blocked=True);assert all(not any(x[1:]) for x in received[-10:]),'collision leaked command'
        assert not statuses[-1].get('continuity_active'),'blocked command retained continuity'
        # Only a farther physical ray clears earlier occupancy.
        feed(2,clear=True);assert any(x[1]>.5 for x in received[-30:]),statuses[-5:]
        route(goal_x=.11);feed(1)
        assert any(.25<=x[1]<.256 and x[2]==0 and x[3]==0 for x in received[-20:]),statuses[-5:]
        route(goal_x=0.,goal_yaw=-.2);feed(1)
        assert any(x[1]==0 and x[2]==0 and abs(x[3]+.7)<1e-8 for x in received[-20:]),statuses[-5:]
        route(goal_x=0.);feed(.7);assert all(not any(x[1:]) for x in received[-10:]),'arrival leaked command'
    report=dict(passed=True,scene="route_progress" if ROUTE_PROGRESS else "posture_transition" if POSTURE else "saved_height_offset" if HEIGHT_OFFSET else "missing_saved_support" if NO_SUPPORT else "stairs" if STAIRS else "flat",commands=len(received),max_control_period_ms=max(x[1] for x in timing[20:]),mean_control_period_ms=sum(x[1] for x in timing[20:])/len(timing[20:]),max_planning_ms=max(x[2] for x in timing),states=sorted(set(s['state'] for s in statuses)))
    (root/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
finally:
    stop_children({'first':proc});log.close();node.destroy_node();rclpy.shutdown()
