#!/usr/bin/env python3
"""SE(2) route windows and static swept-body verification, outside the fast loop."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import math
from pathlib import Path
import sys
import time
import numpy as np
from core import Route, sample_bspline, route_yaws, wrap
from static_guard import StaticGuard
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


def pose_array(p):
    q=p.orientation
    return [p.position.x,p.position.y,p.position.z,
            math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]


class RouteNode(Node):
    def __init__(self,directory,frame=None):
        super().__init__('se2_scan_route')
        self.planner=Planner(directory)
        self.guard=StaticGuard(self.planner)
        self.frame=frame or self.planner.document['frame']
        self.route=None;self.body=None;self.odom_at=0.;self.height=None;self.height_at=0.
        self.reference=None;self.generation=0;self.issued=0.;self.future=None;self.waiting=None
        self.pool=ThreadPoolExecutor(max_workers=1)
        retained=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.ref_pub=self.create_publisher(RosPath,'/se2_scan/reference',retained)
        self.traj_pub=self.create_publisher(Bspline,'/se2_scan/verified_bspline',1)
        self.turn_pub=self.create_publisher(PoseStamped,'/se2_scan/turn_permit',1)
        self.alive_pub=self.create_publisher(Bool,'/se2_scan/route_alive',1)
        self.last_alive=0.;self.ref_message=None
        self.local_pub=self.create_publisher(RosPath,'/se2_scan/local_path',1)
        self.status_pub=self.create_publisher(String,'/se2_scan/route_status',1)
        self.timing_pub=self.create_publisher(Float64MultiArray,'/se2_scan/optimizer_timing',100)
        self.create_subscription(RosPath,'/se2_global/path',self.path,retained)
        self.create_subscription(Odometry,'/nvblox_lio/odom',self.odom,qos_profile_sensor_data)
        self.create_subscription(String,'/nvblox_robot/motion_status/raw',self.motion,10)
        self.create_subscription(Bspline,'/se2_scan/planning/bspline',self.candidate,1)
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
        self.route=None;self.reference=None;self.generation+=1;self.waiting=None
        self.ref_pub.publish(self.message([]));self.turn_pub.publish(PoseStamped());self.status(reason)

    def path(self,m):
        self.clear('new_route')
        if not m.poses:return
        if m.header.frame_id!=self.frame:
            self.status('rejected: path frame differs from the saved map');return
        try:self.route=Route([pose_array(p.pose) for p in m.poses])
        except ValueError as e:self.status(str(e));return
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
        if self.reference is None:return
        # SCAN's original message has no route id. Reject pre-window timestamps,
        # and verify every new candidate against this window and saved geometry.
        t=m.start_time.sec+m.start_time.nanosec/1e9
        if t<self.issued-.02:return
        self.waiting=(m,self.generation,self.reference.copy(),self.height,now,self.body.copy())

    def verify(self,item):
        m,generation,reference,height,began,body=item
        try:
            positions=sample_bspline([[p.x,p.y,p.z] for p in m.pos_pts],m.knots,m.order)
            yaws=route_yaws(positions,reference)
            poses=np.column_stack((positions,yaws));poses[:,2]-=height
            # Limit local optimization to the current ordered route neighbourhood;
            # changing floors or skipping a landing cannot pass this test.
            if np.linalg.norm(positions[0]-body[:3])>.5:
                return item,None,'trajectory start too far from measured body'
            from_start=body.copy();from_start[2]-=height
            pairs=[(from_start,poses[0])]+list(zip(poses,poses[1:]))
            accepted=self.guard.motions(pairs)
            if not accepted.all():
                # If smoothing leaves the certified corridor, use a stopped-end
                # cubic along the next checked reference segment. This still
                # passes the SCAN occupancy guard before any velocity is emitted.
                return self.fallback(item,'optimizer left certified support corridor')
            # A short local optimizer may deviate around obstacles, but must not
            # make a long shortcut across rooms or between overlapping floors.
            for p in positions:
                nearest=min(np.linalg.norm(p-a[:3]) for a in reference)
                if nearest>.65:return self.fallback(item,'outside local route neighbourhood')
            return item,poses,'verified'
        except (ValueError,RuntimeError) as e:return item,None,str(e)

    def fallback(self,item,reason):
        m,generation,reference,height,began,body=item
        a=body.copy();a[2]-=height
        for target in reference:
            distance=float(np.linalg.norm(target[:3]-body[:3]))
            if distance<.04:continue
            b=target.copy();b[2]-=height
            if not self.guard.motions([(a,b)])[0]:return item,None,reason
            if distance>.65:return item,None,reason
            # Clamped cubic Bezier on a certified straight SE(2) segment, zero
            # endpoint speed, <=0.35 m/s and <=0.5 m/s^2 by construction.
            duration=max(.3,1.5*distance/.35,math.sqrt(6*distance/.5))
            fallback=Bspline();fallback.order=3;fallback.traj_id=m.traj_id;fallback.start_time=m.start_time
            from geometry_msgs.msg import Point
            for p in (body[:3],body[:3],target[:3],target[:3]):
                fallback.pos_pts.append(Point(x=float(p[0]),y=float(p[1]),z=float(p[2])))
            fallback.knots=[0.]*4+[duration]*4
            result=(fallback,generation,reference,height,began,body)
            return result,np.asarray([a,b]),'verified reference fallback'
        return item,None,reason

    def tick(self):
        t=time.monotonic()
        if t-self.last_alive>.05:
            self.last_alive=t
            self.alive_pub.publish(Bool(data=self.route is not None and t-self.odom_at<.25 and t-self.height_at<.5))
        if self.future is not None and self.future.done():
            item,poses,reason=self.future.result();self.future=None
            m,generation,ref,height,began,body=item
            if generation==self.generation:
                elapsed=(t-began)*1000
                self.timing_pub.publish(Float64MultiArray(data=[m.start_time.sec+m.start_time.nanosec/1e9,elapsed,float(poses is not None),t,float(m.traj_id)]))
                if poses is not None and t-began<.5:
                    self.traj_pub.publish(m);self.local_pub.publish(self.message(poses));self.status('tracking: '+reason)
                else:self.status('hold: '+reason)
        if self.waiting is not None and self.future is None:
            item=self.waiting;self.waiting=None;self.future=self.pool.submit(self.verify,item)
        if self.route is None or self.body is None:return
        if t-self.odom_at>.25 or self.height is None or t-self.height_at>.5:
            self.status('hold: waiting for fresh body pose and measured MotionStatus.Height')
            self.turn_pub.publish(PoseStamped());return
        floor=self.body.copy();floor[2]-=self.height
        reference,turn=self.route.window(self.body,self.height)
        if self.route.distance(floor)>.65:
            self.clear('replan_required: measured body is outside current route segment');return
        if len(reference)==1 and np.linalg.norm(reference[0,:3]-self.body[:3])<.12 and abs(float(wrap(reference[0,3]-self.body[3])))<.12:
            self.clear('goal_reached');return
        changed=(self.reference is None or len(reference)!=len(self.reference) or
                 np.linalg.norm(reference[-1,:3]-self.reference[-1,:3])>.20 or
                 abs(float(wrap(reference[-1,3]-self.reference[-1,3])))>.10 or
                 np.linalg.norm(reference[0,:3]-self.reference[0,:3])>.20)
        if changed:
            self.generation+=1;self.reference=reference;self.issued=self.get_clock().now().nanoseconds/1e9
            self.ref_message=self.message(reference);self.ref_pub.publish(self.ref_message);self.turn_pub.publish(PoseStamped())
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
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.pool.shutdown(wait=True,cancel_futures=True);node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
