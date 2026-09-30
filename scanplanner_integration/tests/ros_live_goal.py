#!/usr/bin/env python3
"""Read-only hardware-in-the-loop route test. Refuse if a UDP sender is armed."""
import argparse,collections,json,time
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from geometry_msgs.msg import PointStamped,TwistStamped
from nav_msgs.msg import Odometry,Path
from std_msgs.msg import String,Bool

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--goal',nargs=3,type=float,default=[.65,-.30,-.54]);ap.add_argument('--observe',type=float,default=15.);args=ap.parse_args()
    rclpy.init();n=rclpy.create_node('se2_live_route_test');states=[];poses=[];heights=[];status=[];cmds=[];routes=[];locals=[];gates=[]
    controls=[]
    retained=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    n.create_subscription(String,'/se2_global/status',lambda m:states.append(json.loads(m.data)),retained)
    n.create_subscription(Odometry,'/se2_scan/aligned_odom',lambda m:poses.append((time.monotonic(),m)),qos_profile_sensor_data)
    n.create_subscription(String,'/nvblox_robot/motion_status/raw',lambda m:heights.append((time.monotonic(),json.loads(m.data))),10)
    n.create_subscription(String,'/se2_scan/m20_status',lambda m:gates.append(json.loads(m.data)),10)
    n.create_subscription(String,'/se2_scan/route_status',lambda m:status.append(m.data),10)
    n.create_subscription(String,'/se2_scan/control_status',lambda m:controls.append((time.monotonic(),m.data)),10)
    n.create_subscription(Path,'/se2_global/path',routes.append,retained)
    n.create_subscription(Path,'/se2_scan/local_path',locals.append,10)
    n.create_subscription(TwistStamped,'/se2_scan/cmd_vel_stamped',lambda m:cmds.append([time.monotonic(),m.twist.linear.x,m.twist.linear.y,m.twist.angular.z]),qos_profile_sensor_data)
    pub=n.create_publisher(PointStamped,'/clicked_point',10);cancel=n.create_publisher(Bool,'/se2_scan/cancel',10)
    def wait(predicate,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end and not predicate():rclpy.spin_once(n,timeout_sec=.01)
        assert predicate(),dict(global_status=states[-1:],route=status[-5:],poses=len(poses),heights=len(heights),gates=gates[-1:])
    try:
        wait(lambda:states and poses and heights and gates and time.monotonic()-poses[-1][0]<.2 and time.monotonic()-heights[-1][0]<.3,90)
        assert gates[-1]['telemetry_only'],'Refusing to inject a goal with hardware execution enabled'
        initial=states[-1]['generation'];goal=PointStamped();goal.header.frame_id='se2_scan_map';goal.point.x,goal.point.y,goal.point.z=args.goal;pub.publish(goal)
        wait(lambda:states[-1]['generation']>initial and states[-1]['state'] in ('success','no_path','invalid_start'),90)
        assert states[-1]['state']=='success',states[-1]
        wait(lambda:locals or any(s.startswith('hold:') for s in status),30)
        observing=time.monotonic();end=observing+args.observe
        while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
        nonzero=[c for c in cmds if max(abs(x) for x in c[1:])>.005]
        result=dict(state='passed' if nonzero and locals else 'blocked',start=states[-1]['start'],global_seconds=states[-1]['result']['seconds'],path_poses=len(routes[-1].poses) if routes else 0,local_paths=len(locals),nonzero_commands=len(nonzero),route_status=status[-10:],telemetry_only=gates[-1]['telemetry_only'],basic=gates[-1]['basic'])
        measured=[c for c in cmds if c[0]>=observing]
        forward=[c[1] for c in measured if c[1]>.005]
        result.update(goal=args.goal,observation_seconds=args.observe,
            active_command_fraction=sum(max(abs(v) for v in c[1:])>.005 for c in measured)/max(1,len(measured)),
            active_forward_m_s=dict(median=sorted(forward)[len(forward)//2],max=max(forward)) if forward else None,
            control_reasons=dict(collections.Counter(v for t,v in controls if t>=observing)),
            sender=gates[-1])
        cancel.publish(Bool(data=True));cut=time.monotonic()+.4;end=time.monotonic()+1
        while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
        tail=[c for c in cmds if c[0]>cut];assert tail and all(max(abs(x) for x in c[1:])<1e-6 for c in tail),'Cancel failed'
        result['cancel_zero']=True;print(json.dumps(result,ensure_ascii=False),flush=True)
        assert result['state']=='passed',result
    finally:
        cancel.publish(Bool(data=True));n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
