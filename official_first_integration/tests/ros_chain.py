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
MAP_ENTRY='--map-entry' in sys.argv
RECOVERY='--recovery' in sys.argv
TERMINAL_SWITCH='--terminal-switch' in sys.argv
TURN_DEADBAND='--corner-deadband' in sys.argv
CORNER='--corner' in sys.argv or TURN_DEADBAND
STALLED='--stalled' in sys.argv
HEADING='--heading' in sys.argv
RETAIN='--retain-obstacle' in sys.argv
HERE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(HERE));from run import session_remaps,ros_args,stop_children
SESSION='--session' in sys.argv
remaps=session_remaps('ros_chain') if SESSION else {}
def topic(name):return remaps.get(name,name)
root=HERE/('runtime/isolated_heading' if HEADING else 'runtime/isolated_corner' if CORNER else 'runtime/isolated_terminal_switch' if TERMINAL_SWITCH else 'runtime/isolated_map_entry' if MAP_ENTRY else 'runtime/isolated_recovery' if RECOVERY else 'runtime/isolated_no_support' if NO_SUPPORT else 'runtime/isolated_stairs' if STAIRS else 'runtime/isolated');root.mkdir(parents=True,exist_ok=True)
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

def route(empty=False,goal_x=4.,goal_yaw=0.,publisher=None,start_x=0.):
    m=RosPath();m.header.frame_id='se2_scan_map';m.header.stamp=node.get_clock().now().to_msg()
    if not empty:
        for i in range(41):
            p=PoseStamped();p.header=m.header;p.pose.position.x=start_x+(goal_x-start_x)*i/40;p.pose.position.z=-.42 if STAIRS and p.pose.position.x>=2 else -.57;p.pose.orientation.w=math.cos(goal_yaw*i/80);p.pose.orientation.z=math.sin(goal_yaw*i/80);m.poses.append(p)
    if CORNER and not empty:
        m.poses=[]
        for x,y,yaw in [(i*.1,0.,0.) for i in range(25)]+[(2.4,0.,i*math.pi/8) for i in range(1,5)]+[(2.4,i*.1,math.pi/2) for i in range(1,31)]:
            p=PoseStamped();p.header=m.header;p.pose.position.x=x;p.pose.position.y=y;p.pose.position.z=-.57
            p.pose.orientation.w=math.cos(yaw/2);p.pose.orientation.z=math.sin(yaw/2);m.poses.append(p)
    (publisher or pubs['route']).publish(m)

body_x=body_y=body_yaw=0.;body_trace=[]
def feed(seconds,blocked=False,cloud=True,clear=False,gait=0x3002,move_to=None,low_return=None,height=.57,z_override=None,pending=0,map_shift=0.,dead_end=False,rear=True,alternating=False):
    global body_x,body_y,body_yaw
    start=time.monotonic();last_scan=0;last_body=start;scan_number=0
    while time.monotonic()-start<seconds:
        now=node.get_clock().now();stamp=now.to_msg()
        if CORNER:
            clock=time.monotonic();dt=min(.03,clock-last_body);last_body=clock
            if received and clock-received[-1][0]<.12:
                _,vx,vy,w=received[-1];c,s=math.cos(body_yaw),math.sin(body_yaw)
                if TURN_DEADBAND and abs(w)<.15:w=0.
                body_x+=(c*vx-s*vy)*dt;body_y+=(s*vx+c*vy)*dt;body_yaw+=w*dt
            body_trace.append([body_x,body_y,body_yaw])
            # Actual padded body rectangle must never cross the room wall.
            for i in range(113):
                dx=-1.+i*.025-body_x;dy=.7-body_y;c,s=math.cos(body_yaw),math.sin(body_yaw)
                assert not(abs(c*dx+s*dy)<.44536 and abs(-s*dx+c*dy)<.28836),'corner cut through wall'
        if move_to is not None:body_x+=max(-.015,min(.015,move_to-body_x))
        body_z=.15 if STAIRS and body_x>=2 else 0.
        if z_override is not None:body_z=z_override
        tf=PoseStamped();tf.header.frame_id='se2_scan_map';tf.header.stamp=stamp;tf.pose.orientation.w=1.;tf.pose.position.x=map_shift;pubs['tf'].publish(tf)
        o=Odometry();o.header.frame_id='nvblox_odom';o.child_frame_id='base_link_dog';o.header.stamp=stamp;o.pose.pose.orientation.w=math.cos(body_yaw/2);o.pose.pose.orientation.z=math.sin(body_yaw/2);o.pose.pose.position.x=body_x;o.pose.pose.position.y=body_y;o.pose.pose.position.z=body_z;pubs['pose'].publish(o)
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
            if dead_end:
                # Quantized U-shaped corridor: only backing out is feasible.
                for i in range(-40,41):
                    for x,y in ((.475,i*.0125),(i*.025,.325),(i*.025,-.325)):
                        if x>.475:continue
                        points.append(struct.pack('<fffdHxx',x,y,.0,now.nanoseconds/1e9,0))
            if CORNER:
                for i in range(113):
                    dx=-1.+i*.025-body_x;dy=.7-body_y;c,s=math.cos(body_yaw),math.sin(body_yaw)
                    points.append(struct.pack('<fffdHxx',c*dx+s*dy,-s*dx+c*dy,.0,now.nanoseconds/1e9,0))
            if alternating:
                points=[p for p in points if (struct.unpack_from("<H",p,20)[0]>=96)==bool(scan_number%2)]
                scan_number+=1
            if not rear:
                points=[p[:20]+struct.pack('<Hxx',struct.unpack_from('<H',p,20)[0]%96) for p in points]
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
    if RETAIN:
        feed(2);route();feed(.6);feed(1.,blocked=True)
        assert all(not any(x[1:]) for x in received[-8:]),'initial obstacle did not block'
        route(goal_x=3.);feed(1.)
        assert all(not any(x[1:]) for x in received[-8:]),'new route erased uncleared occupied cells'
        feed(.4,gait=0x1001,pending=0x3002);feed(1.)
        assert all(not any(x[1:]) for x in received[-8:]),'posture refresh erased occupied cells'
        route(True);feed(.4)
    elif HEADING:
        feed(2);route();feed(.5)
        body_x=.8;body_y=.25;body_yaw=0.
        feed(.8)
        assert any(x[2]<-.05 for x in received[-12:]),'no lateral correction'
        assert all(abs(x[3])<.11 for x in received[-10:]),'preview bearing still controls yaw'
        body_yaw=-math.pi/3
        feed(.8)
        assert any(x[3]>.5 for x in received[-10:]),'over-rotated body did not turn back'
        assert all(abs(s['route_heading_rad'])<1e-9 for s in statuses[-4:])
        route(True);feed(.5)
        assert all(not any(x[1:]) for x in received[-10:]),'cancellation leaked motion'
    elif STALLED:
        feed(2);route();feed(4.5)
        assert any(s['state']=='no_progress' for s in statuses),'stationary tracking never detected'
        assert any(s['recovery_state']=='retreat' for s in statuses),'stationary tracking never retreated'
        assert any(x[1]<-.05 for x in received[-35:]),statuses[-5:]
        route(True);feed(.5)
        assert all(not any(x[1:]) for x in received[-10:])
    elif CORNER:
        feed(2);route();feed(16)
        (root/'body_trace.json').write_text(json.dumps(body_trace))
        assert math.hypot(body_x-2.4,body_y-3.)<.18,(body_x,body_y,statuses[-5:])
        assert any(s['state']=='arrived' for s in statuses[-10:]),statuses[-5:]
        assert not any(s['state']=='route_disconnected' for s in statuses)
        route(True);feed(.5)
        assert all(not any(x[1:]) for x in received[-10:])
    elif TERMINAL_SWITCH:
        feed(2);route(goal_x=0.,goal_yaw=-.2);feed(1)
        assert statuses[-1]['phase']=='align',statuses[-5:]
        feed(.6,move_to=-.115)
        assert statuses[-1]['phase']=='align','11.5 cm noise toggled back to approach'
        assert all(abs(x[1])<1e-8 for x in received[-8:]),'alignment leaked translation'
        before=len(received);feed(.8,move_to=-.16)
        assert statuses[-1]['phase']=='approach',statuses[-5:]
        assert any(0.<x[1]<.2 and -.7<x[3]<0. for x in received[before:]),'phase change had no blend'
        assert all(x[1]>=.25 and abs(x[3])<1e-8 for x in received[-8:]),'blend failed to finish'
        route(True);feed(.5)
        assert all(not any(x[1:]) for x in received[-10:]),'cancellation delayed stop'
    elif MAP_ENTRY:
        feed(2);route(start_x=.35);feed(1)
        assert statuses[-1]['phase']=='join_map',statuses[-5:]
        assert any(.1<x[1]<=.400001 for x in received[-20:]),statuses[-5:]
        feed(1,move_to=.35);feed(.5)
        assert statuses[-1]['phase']=='follow',statuses[-5:]
        assert any(x[1]>.5 for x in received[-20:]),statuses[-5:]
        route(start_x=2.);feed(.5)
        assert statuses[-1]['state']=='map_entry_too_far',statuses[-5:]
        assert all(not any(x[1:]) for x in received[-10:])
    elif RECOVERY:
        feed(2);route();feed(1);feed(1.6,dead_end=True)
        assert any(x[1]<-.05 for x in received[-30:]),statuses[-10:]
        assert any(s.get('recovery_state')=='retreat' for s in statuses[-10:]),statuses[-10:]
        before=len(statuses);feed(1.2,dead_end=True,alternating=True)
        tail=statuses[before+2:]
        assert len(tail)>=5 and all(s['recovery_state']=='retreat' for s in tail),tail
        assert all(0<=s['rear_age_s']<.35 for s in tail),tail
        feed(.7,dead_end=True,rear=False)
        assert all(not any(x[1:]) for x in received[-8:]),'rear loss leaked motion'
        route(True);feed(.5)
        assert statuses[-1]['state']=='idle' and statuses[-1]['recovery_attempts']==0
    elif POSTURE:
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
        deadline=time.monotonic()+7
        while body_x<4.499 and time.monotonic()<deadline:feed(.1,move_to=4.5)
        feed(.5)
        assert abs(body_x-4.5)<.001,body_x
        assert any(x[1]>.5 for x in received[-20:]),statuses[-5:]
        feed(1.5,map_shift=.04,move_to=4.7)
        assert statuses[-1]['state']=='tracking',statuses[-8:]
        assert any(x[1]>.5 for x in received[-20:]),'map correction lost route progress'
        # A posture refresh also invalidates the old command/evidence, without
        # resetting the fixed-map route to its first three metres.
        feed(.5,gait=0x1001,pending=0x3002,map_shift=.04)
        feed(1.5,map_shift=.04,move_to=4.9)
        assert statuses[-1]['state']=='tracking',statuses[-8:]
        route(True);feed(.5,map_shift=.04)
        assert all(not any(x[1:]) for x in received[-10:]),'cancellation leaked motion'
        route(goal_x=10.);feed(1,map_shift=.04)
        assert statuses[-1]['state']=='map_entry_too_far','new route did not reset progress'
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
        # This fixture holds odometry fixed, so an already-started retreat ends
        # through its 4 s timeout before the clear forward route can resume.
        feed(5,clear=True);assert any(x[1]>.5 for x in received[-30:]),statuses[-5:]
        route(goal_x=.11);feed(1)
        assert any(.25<=x[1]<.256 and x[2]==0 and x[3]==0 for x in received[-20:]),statuses[-5:]
        route(goal_x=0.,goal_yaw=-.2);feed(1)
        assert any(x[1]==0 and x[2]==0 and abs(x[3]+.7)<1e-8 for x in received[-20:]),statuses[-5:]
        route(goal_x=0.);feed(.7);assert all(not any(x[1:]) for x in received[-10:]),'arrival leaked command'
    report=dict(passed=True,scene="retained_obstacle" if RETAIN else "corner_deadband" if TURN_DEADBAND else "heading" if HEADING else "stalled" if STALLED else "corner" if CORNER else "terminal_switch" if TERMINAL_SWITCH else "map_entry" if MAP_ENTRY else "reverse_recovery" if RECOVERY else "route_progress" if ROUTE_PROGRESS else "posture_transition" if POSTURE else "saved_height_offset" if HEIGHT_OFFSET else "missing_saved_support" if NO_SUPPORT else "stairs" if STAIRS else "flat",commands=len(received),max_control_period_ms=max(x[1] for x in timing[20:]),mean_control_period_ms=sum(x[1] for x in timing[20:])/len(timing[20:]),max_planning_ms=max(x[2] for x in timing),states=sorted(set(s['state'] for s in statuses)))
    (root/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
finally:
    stop_children({'first':proc});log.close();node.destroy_node();rclpy.shutdown()
