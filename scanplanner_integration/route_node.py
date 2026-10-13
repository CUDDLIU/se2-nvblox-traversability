#!/usr/bin/env python3
"""SE(2) route windows and static swept-body verification, outside the fast loop."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import math
import os
import signal
from pathlib import Path
import sys
import time
import numpy as np
from core import Route, sample_bspline, project_reference, wrap, adapt_route, polyline_distances
from static_guard import StaticGuard
from corridor import encode_corridor
from live_occupancy import LiveOccupancy
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'global_planner'))
from planner import Planner
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path as RosPath
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String, Float64MultiArray
from scan_planner_msgs.msg import Bspline
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy


def pose_array(p):
    q=p.orientation
    return [p.position.x,p.position.y,p.position.z,
            math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]


class RouteNode(Node):
    def __init__(self,directory,frame=None):
        super().__init__('se2_scan_route')
        self.guard_resolution=float(self.declare_parameter('guard_resolution',.1).value)
        if not .025<=self.guard_resolution<=.1:raise ValueError('Invalid guard resolution')
        self.planner=Planner(directory)
        self.guard=StaticGuard(self.planner)
        self.max_step=self.planner.config.max_step
        self.frame=frame or self.planner.document['frame']
        self.route=None;self.body=None;self.odom_at=0.;self.height=None;self.height_at=0.
        self.reference=None;self.reference_height=None;self.generation=0;self.issued=0.;self.future=None;self.waiting=None
        self.route_future=None
        self.optimizer_state=None;self.optimizer_at=0.;self.last_candidate_id=-1
        self.live=None;self.live_at=0.;self.detour=None;self.detour_at=0.
        self.last_diagnostics=0.
        self.pool=ThreadPoolExecutor(max_workers=1)
        retained=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.ref_pub=self.create_publisher(RosPath,'/se2_scan/reference',retained)
        self.corridor_pub=self.create_publisher(Float64MultiArray,'/se2_scan/reference_corridor',retained)
        self.traj_pub=self.create_publisher(Bspline,'/se2_scan/verified_bspline',1)
        self.turn_pub=self.create_publisher(PoseStamped,'/se2_scan/turn_permit',1)
        self.alive_pub=self.create_publisher(Bool,'/se2_scan/route_alive',1)
        self.last_alive=0.;self.ref_message=None
        self.local_pub=self.create_publisher(RosPath,'/se2_scan/local_path',1)
        self.status_pub=self.create_publisher(String,'/se2_scan/route_status',1)
        self.diagnostics_pub=self.create_publisher(String,'/se2_scan/route_diagnostics',1)
        self.timing_pub=self.create_publisher(Float64MultiArray,'/se2_scan/optimizer_timing',100)
        self.create_subscription(RosPath,'/se2_global/path',self.path,retained)
        self.create_subscription(Odometry,'/nvblox_lio/odom',self.odom,qos_profile_sensor_data)
        self.create_subscription(String,'/nvblox_robot/motion_status/raw',self.motion,10)
        self.create_subscription(Bspline,'/se2_scan/planning/bspline',self.candidate,1)
        self.create_subscription(Float64MultiArray,'/se2_scan/planning/state',self.optimizer,1)
        self.create_subscription(PointCloud2,'/se2_scan/guard/collision_occupancy',self.occupancy,qos_profile_sensor_data)
        self.create_subscription(RosPath,'/se2_scan/planning/detour',self.detour_path,1)
        self.create_subscription(Bool,'/se2_scan/cancel',lambda m:self.clear('cancelled') if m.data else None,1)
        self.create_timer(.01,self.tick)
        self.last_report='';self.last_turn_check=0.

    def status(self,text):
        if text!=self.last_report:
            self.last_report=text
            self.status_pub.publish(String(data=text));self.get_logger().info(text)

    def message(self,poses):
        m=RosPath();m.header.frame_id=self.frame;m.header.stamp=self.get_clock().now().to_msg()
        for p in poses:
            a=PoseStamped();a.header=m.header
            a.pose.position.x=float(p[0]);a.pose.position.y=float(p[1]);a.pose.position.z=float(p[2])
            a.pose.orientation.z=math.sin(p[3]/2);a.pose.orientation.w=math.cos(p[3]/2);m.poses.append(a)
        return m

    def clear(self,reason):
        self.route=None;self.reference=None;self.reference_height=None;self.generation+=1;self.waiting=None
        self.detour=None
        self.ref_pub.publish(self.message([]));self.turn_pub.publish(PoseStamped());self.status(reason)

    def path(self,m):
        self.clear('new_route')
        if not m.poses:return
        if m.header.frame_id!=self.frame:
            self.status('rejected: path frame differs from the saved map');return
        try:poses=Route([pose_array(p.pose) for p in m.poses]).poses
        except ValueError as e:self.status(str(e));return
        generation=self.generation
        self.route_future=(generation,self.pool.submit(adapt_route,poses,self.guard.motions))
        self.status('adapting global path for continuous local tracking')
        self.tick()

    def odom(self,m):
        if m.header.frame_id!=self.frame or m.child_frame_id!='base_link_dog':return
        p=np.asarray(pose_array(m.pose.pose))
        if np.isfinite(p).all():self.body=p;self.odom_at=time.monotonic()

    def motion(self,m):
        try:
            status=json.loads(m.data)['PatrolDevice']['Items']['MotionStatus'];h=float(status['Height'])
            if .2<h<1.0:self.height=h;self.height_at=time.monotonic()
        except (KeyError,TypeError,ValueError):pass

    def candidate(self,m):
        now=time.monotonic()
        if self.reference is None or self.body is None or self.height is None:return
        # SCAN's original message has no route id. Reject pre-window timestamps,
        # and verify every new candidate against this window and saved geometry.
        t=m.start_time.sec+m.start_time.nanosec/1e9
        if t<self.issued-.02:return
        self.waiting=(m,self.generation,self.reference.copy(),self.reference_height,now,self.body.copy(),self.height)

    def optimizer(self,m):
        if len(m.data)!=5 or not np.isfinite(m.data).all():return
        age=self.get_clock().now().nanoseconds/1e9-m.data[1]
        if -.1<=age<.4:self.optimizer_state=list(m.data);self.optimizer_at=time.monotonic()

    def occupancy(self,m):
        if m.header.frame_id!=self.frame:return
        age=self.get_clock().now().nanoseconds/1e9-(m.header.stamp.sec+m.header.stamp.nanosec/1e9)
        if -.1<=age<.4:
            p=read_points_numpy(m,field_names=('x','y','z','resolution'),skip_nans=True)
            self.live=LiveOccupancy(p[:,:3],p[:,3],self.guard_resolution)
            self.live_at=time.monotonic()

    def detour_path(self,m):
        if self.ref_message is None or m.header!=self.ref_message.header or len(m.poses)<2:return
        self.detour=np.asarray([pose_array(p.pose) for p in m.poses]);self.detour_at=time.monotonic()

    def verify(self,item):
        m,generation,reference,height,began,body,body_height=item
        try:
            positions=sample_bspline([[p.x,p.y,p.z] for p in m.pos_pts],m.knots,m.order)
            if np.max(np.linalg.norm(positions-positions[0],axis=1))<.03:
                return item,None,'optimizer stop trajectory: no progress'
            projected=project_reference(positions,reference)
            if np.max(np.abs(positions[:,2]-projected[:,2]))>self.max_step:
                return item,None,'trajectory leaves the reference surface layer'
            # SCAN commands XY/yaw; body Z is a timing/geometry guide, not a
            # commanded flight altitude. Validate support at the saved ground
            # layer, independently of changing measured stance height.
            poses=np.column_stack((positions,projected[:,3]));poses[:,2]=projected[:,2]-height
            if np.linalg.norm(positions[0]-body[:3])>.5:
                return item,None,'trajectory start too far from measured body'
            from_start=body.copy();from_start[2]=project_reference([body[:3]],reference)[0,2]-height
            surface=self.guard.surface_poses(np.vstack((from_start,poses)))
            from_start,poses=surface[0],surface[1:]
            if abs(body[2]-body_height-from_start[2])>self.max_step:
                return item,None,'measured body height differs from the reference surface layer'
            pairs=[(from_start,poses[0])]+list(zip(poses,poses[1:]))
            if not self.guard.motions(pairs).all():
                return item,None,'native spline fails saved-map body/support sweep'
            # Heading differences are handled by the trajectory tracker. They
            # must not replace a valid SCAN spline with a hand-picked maneuver.
            return item,poses,'verified native SCAN spline'
        except (ValueError,RuntimeError) as e:return item,None,str(e)

    def tick(self):
        t=time.monotonic()
        if t-self.last_diagnostics>.2:
            self.last_diagnostics=t
            self.diagnostics_pub.publish(String(data=json.dumps(dict(route_active=self.route is not None,
                odom_age=t-self.odom_at,height_age=t-self.height_at,height=self.height,
                occupancy_age=t-self.live_at,optimizer_age=t-self.optimizer_at,
                native_trajectory_only=True,reference_height=self.reference_height,future_active=self.future is not None,
                waiting=self.waiting is not None,issued=self.issued,optimizer_state=self.optimizer_state))))
        if self.route_future is not None and self.route_future[1].done():
            generation,future=self.route_future;self.route_future=None
            if generation==self.generation:
                try:
                    poses,report=future.result();self.route=Route(poses)
                    self.status('route adapted: '+json.dumps(report))
                except (ValueError,RuntimeError) as exc:self.clear('route adaptation failed: '+str(exc))
        if t-self.last_alive>.05:
            self.last_alive=t
            self.alive_pub.publish(Bool(data=self.route is not None and t-self.odom_at<.25 and t-self.height_at<.5))
        if self.future is not None and self.future.done():
            item,poses,reason=self.future.result();self.future=None
            m,generation,ref,height,began,body,body_height=item
            if generation==self.generation:
                elapsed=(t-began)*1000
                source_stamp=(m.start_time.sec+m.start_time.nanosec/1e9) if m is not None else self.issued
                fresh=t-self.odom_at<.25 and t-self.height_at<.5
                publish=poses is not None and t-began<.3 and fresh
                kind=1. if publish else 0.
                self.timing_pub.publish(Float64MultiArray(data=[source_stamp,elapsed,float(poses is not None),t,
                    float(m.traj_id) if m is not None else -1.,float(publish),kind]))
                if publish:
                    self.traj_pub.publish(m)
                    self.local_pub.publish(self.message(poses))
                    self.status('tracking: '+reason)
                else:self.status('hold: '+(reason if poses is None else 'certificate inputs expired'))
        if self.waiting is not None and self.future is None:
            item=self.waiting;self.waiting=None;self.future=self.pool.submit(self.verify,item)
        if self.route is None or self.body is None:return
        if t-self.odom_at>.25 or self.height is None or t-self.height_at>.5:
            missing='body pose' if t-self.odom_at>.25 else 'MotionStatus.Height'
            self.status('hold: waiting for fresh '+missing)
            self.turn_pub.publish(PoseStamped());return
        floor=self.body.copy();floor[2]-=self.height
        reference,turn=self.route.window(self.body,self.height)
        # SCAN may detour outside a narrow band around the global reference.
        # Retain the local planning horizon bound; geometry validates the path.
        if self.route.distance(floor)>3.0:
            self.clear('replan_required: measured body is outside current route segment');return
        if len(reference)==1 and np.linalg.norm(reference[0,:3]-self.body[:3])<.12 and abs(float(wrap(reference[0,3]-self.body[3])))<.12:
            self.clear('goal_reached');return
        changed=(self.reference is None or len(reference)!=len(self.reference) or
                 np.linalg.norm(reference[-1,:3]-self.reference[-1,:3])>.20 or
                 abs(float(wrap(reference[-1,3]-self.reference[-1,3])))>.10 or
                 np.linalg.norm(reference[0,:3]-self.reference[0,:3])>.20)
        if changed:
            self.generation+=1;self.reference=reference;self.reference_height=self.height
            self.detour=None
            self.ref_message=self.message(reference)
            self.issued=self.ref_message.header.stamp.sec+self.ref_message.header.stamp.nanosec/1e9
            constraint=encode_corridor(self.planner,reference,self.reference_height,self.issued)
            self.corridor_pub.publish(Float64MultiArray(data=constraint))
            self.ref_pub.publish(self.ref_message);self.turn_pub.publish(PoseStamped())
        if turn and t-self.last_turn_check>.1 and self.future is None:
            self.last_turn_check=t
            a=self.body.copy();a[2]-=self.height;b=reference[-1].copy();b[2]-=self.height
            # Pure rotation remains a SE(2) motion, including support and corners.
            ok=bool(self.guard.motions([(a,b)])[0])
            permit=PoseStamped()
            if ok:permit.header=self.ref_message.header;permit.pose=self.ref_message.poses[-1].pose
            self.turn_pub.publish(permit)
            self.status('turn in place verified' if ok else 'hold: turn body sweep rejected')


def main():
    ap=argparse.ArgumentParser();ap.add_argument('map');ap.add_argument('--frame');args,ros=ap.parse_known_args()
    rclpy.init(args=ros);node=RouteNode(args.map,args.frame)
    reload_requested=[False]
    signal.signal(signal.SIGHUP,lambda *_:reload_requested.__setitem__(0,True))
    try:
        while rclpy.ok() and not reload_requested[0]:rclpy.spin_once(node,timeout_sec=.05)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():node.clear('route adapter restarting')
        node.pool.shutdown(wait=True,cancel_futures=True);node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
    if reload_requested[0]:os.execv(sys.executable,[sys.executable,*sys.argv])
if __name__=='__main__':main()
