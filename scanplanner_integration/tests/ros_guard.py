#!/usr/bin/env python3
"""Deterministic component test: motion, obstacle stop, stale odom, cancel.

An isolated ROS domain and synthetic sensor data; no M20 UDP process is started.
The accepted spline is injected to test the fast controller independently of
optimizer convergence (the recorded-bag test covers the optimizer).
"""
import argparse,json,math,os,signal,struct,subprocess,time
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import Odometry,Path as RosPath
from sensor_msgs.msg import PointCloud2,PointField
from geometry_msgs.msg import PoseStamped,Twist
from scan_planner_msgs.msg import Bspline
from std_msgs.msg import Bool,Float64MultiArray
ROOT=Path(__file__).resolve().parents[2]

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);ap.add_argument('--aligned',action='store_true');ap.add_argument('--compact-map',action='store_true');args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=rclpy.create_node('scan_guard_test')
    odom=node.create_publisher(Odometry,'/test/odom',qos_profile_sensor_data)
    cloud=node.create_publisher(PointCloud2,'/test/cloud',qos_profile_sensor_data)
    reference=node.create_publisher(RosPath,'/se2_scan/reference',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    spline=node.create_publisher(Bspline,'/se2_scan/verified_bspline',1)
    alive=node.create_publisher(Bool,'/se2_scan/route_alive',1)
    alignment=node.create_publisher(PoseStamped,'/se2_scan/map_transform',qos_profile_sensor_data)
    frame='se2_scan_map' if args.aligned else 'nvblox_odom';yaw=.7 if args.aligned else 0.
    def point(x,y,z):
        from geometry_msgs.msg import Point
        return Point(x=math.cos(yaw)*x-math.sin(yaw)*y+(1. if args.aligned else 0.),y=math.sin(yaw)*x+math.cos(yaw)*y-(1. if args.aligned else 0.),z=z+(.2 if args.aligned else 0.))
    commands=[];decisions=[];metrics=[];occupied=[]
    def occupancy(m):
        import sensor_msgs_py.point_cloud2 as pc
        occupied[:]=[[float(v) for v in p] for p in pc.read_points(m,field_names=('x','y','z'),skip_nans=True)]
    node.create_subscription(PointCloud2,'/se2_scan/grid_map/occupancy',occupancy,qos_profile_sensor_data)
    node.create_subscription(Twist,'/se2_scan/cmd_vel',lambda m:commands.append((time.monotonic(),m.linear.x,m.linear.y,m.angular.z)),qos_profile_sensor_data)
    node.create_subscription(Bool,'/se2_scan/blocked',lambda m:decisions.append((time.monotonic(),m.data)),1000)
    node.create_subscription(Float64MultiArray,'/se2_scan/timing',lambda m:metrics.append(list(m.data)),1000)
    log=(args.output/'bridge.log').open('w')
    compact=['-p','grid_map.compute_inflation:=false','-p','grid_map.resolution:=0.05','-p','grid_map.max_ray_length:=4.0','-p','grid_map.sliding_map_size_x:=8.0','-p','grid_map.sliding_map_size_y:=8.0','-p','grid_map.sliding_map_size_z:=2.0'] if args.compact_map else []
    proc=subprocess.Popen(['ros2','run','se2_scan_bridge','scan_bridge','--ros-args','--params-file',str(ROOT/'scanplanner_integration/planner.yaml'),'-r','__ns:=/se2_scan','-r','raw_cloud:=/test/cloud','-r','raw_body_pose:=/test/odom','-r','cloud:=/unused','-r','sensor_pose:=/unused_pose','-p','control_lookahead_s:=1.3','-p','world_frame:='+frame,'-p','grid_map.frame_id:='+frame,'-p','use_map_alignment:='+str(args.aligned).lower(),*compact],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    def pump(seconds,obstacle=False,pose=True,trajectory=False,align=True,far_returns=False):
        end=time.monotonic()+seconds;last_cloud=last_spline=0
        while time.monotonic()<end:
            now=time.monotonic();t=node.get_clock().now().nanoseconds/1e9
            alive.publish(Bool(data=True))
            if args.aligned and align:
                a=PoseStamped();a.header.frame_id=frame;a.header.stamp=node.get_clock().now().to_msg();a.pose.position=point(0.,0.,0.);a.pose.orientation.z=math.sin(yaw/2);a.pose.orientation.w=math.cos(yaw/2);alignment.publish(a)
            if pose:
                o=Odometry();o.header.frame_id='nvblox_odom';o.child_frame_id='base_link_dog';o.header.stamp=node.get_clock().now().to_msg();o.pose.pose.orientation.w=1.;odom.publish(o)
            if now-last_cloud>.1:
                last_cloud=now;c=PointCloud2();c.header.frame_id='lidar_link';c.header.stamp=node.get_clock().now().to_msg()
                points=[(x*.1,y*.1,-.57,t) for x in range(-25,26) for y in range(-25,26)]
                if obstacle:points +=[(.52,y*.03,z*.04,t) for y in range(-6,7) for z in range(-1,7)]
                if far_returns:
                    # Real returns past the 4 m local map must clear the old
                    # near obstacle without becoming an occupied horizon ring.
                    points +=[(6.,y*.03*6/.52,z*.04*6/.52,t) for y in range(-6,7) for z in range(-1,7)]
                c.height=1;c.width=len(points);c.point_step=20;c.row_step=20*len(points);c.is_dense=True
                c.fields=[PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(['x','y','z'])]+[PointField(name='timestamp',offset=12,datatype=8,count=1)]
                c.data=b''.join(struct.pack('<fffd',*p) for p in points);cloud.publish(c)
            if trajectory and now-last_spline>.2:
                last_spline=now;b=Bspline();b.order=3;b.traj_id=1;b.start_time=node.get_clock().now().to_msg()
                from geometry_msgs.msg import Point
                b.pos_pts=[point(x,0.,0.) for x in [0.,0.,.1,.3,.5,.5]];b.knots=[i*.5 for i in range(-3,7)];spline.publish(b)
            rclpy.spin_once(node,timeout_sec=.005)
    result={}
    try:
        until=time.monotonic()+15
        while time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.05)
            if cloud.get_subscription_count():break
        else:raise RuntimeError('bridge not ready')
        path=RosPath();path.header.frame_id=frame
        for x in [0.,.5]:
            a=PoseStamped();a.pose.position=point(x,0.,0.);a.pose.orientation.z=math.sin(yaw/2);a.pose.orientation.w=math.cos(yaw/2);path.poses.append(a)
        reference.publish(path);pump(2,trajectory=True)
        moving=[c for c in commands if abs(c[1])>.01]
        assert moving,'controller did not issue nonzero commands on clear synthetic floor'
        assert max(c[1] for c in moving)>.20,'Normal profile did not increase actual startup speed'
        assert all(math.hypot(c[1],c[2])<=1.5+1e-9 and abs(c[2])<=.4 and abs(c[3])<=1.0 for c in commands)
        result['max_clear_forward_m_s']=max(c[1] for c in moving)
        injected=time.monotonic();pump(1,obstacle=True,trajectory=True)
        block=next(t for t,b in decisions if t>=injected and b)
        zero=next(t for t,x,y,z in commands if t>=block and max(abs(x),abs(y),abs(z))<1e-6)
        # The user treats 50 ms as an optimization target, not an acceptance
        # gate. Preserve and report the actual observation, including misses.
        result.update(obstacle_to_block_ms=(block-injected)*1000,
                      obstacle_to_zero_ms=(zero-injected)*1000,
                      target_50ms_met=block-injected<.05)
        assert zero-injected<.10,(zero-injected)
        # No ray through the obstacle: elapsed time alone must not erase it.
        absent=time.monotonic();pump(.6,trajectory=True)
        assert all(max(map(abs,c[1:]))<1e-6 for c in commands if c[0]>absent+.2),'Unobserved obstacle was cleared'
        far=time.monotonic();pump(1.4,trajectory=True,far_returns=True)
        resumed=[c for c in commands if c[0]>far+.9 and c[1]>.01]
        if not resumed:
            origin=point(0.,0.,0.)
            result['uncleared_near_cells']=[p for p in occupied if sum((p[i]-[origin.x,origin.y,origin.z][i])**2 for i in range(3))<1.5**2 and p[2]>origin.z-.3]
        assert resumed,('Far free rays did not clear the ghost obstacle',result.get('uncleared_near_cells'))
        result['far_return_clear_resumed']=True
        reinjected=time.monotonic();pump(.6,obstacle=True,trajectory=True,far_returns=True)
        assert all(max(map(abs,c[1:]))<1e-6 for c in commands if c[0]>reinjected+.2),'Far rays erased a visible near obstacle'
        result['visible_near_obstacle_retained']=True
        stale=time.monotonic();pump(.5,pose=False,trajectory=True)
        recent=[c for c in commands if c[0]>stale+.3]
        assert recent and all(max(abs(v) for v in c[1:])<1e-6 for c in recent)
        if args.aligned:
            pump(1.2,trajectory=True);lost=time.monotonic();pump(2.,trajectory=True,align=False)
            recent=[c for c in commands if c[0]>lost+1.7]
            assert recent and all(max(abs(v) for v in c[1:])<1e-6 for c in recent),'stale map transform did not stop motion'
            result['alignment_expiry_zero']=True
        empty=RosPath();empty.header.frame_id=frame;reference.publish(empty);pump(.3,trajectory=True)
        assert all(max(abs(v) for v in c[1:])<1e-6 for c in commands[-5:])
        result.update(state='passed',max_ingress_decision_ms=max(v[6] for v in metrics),p95_ingress_decision_ms=sorted(v[6] for v in metrics)[int(.95*(len(metrics)-1))],nonzero_commands=len(moving),control_samples=len(commands),frames=len(metrics),checks=['normal startup speed','new obstacle stop','unobserved obstacle retained','far rays clear old obstacle','visible near obstacle retained','stale odometry stop','empty path cancel'])
    finally:
        os.killpg(proc.pid,signal.SIGINT)
        try:proc.wait(timeout=8)
        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        result['last_metrics']=metrics[-3:]
        (args.output/'guard.json').write_text(json.dumps(result,indent=2));log.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    print(json.dumps(result))
if __name__=='__main__':main()
