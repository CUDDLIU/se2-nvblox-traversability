#!/usr/bin/env python3
"""Supervised, explicitly armed goal trial. Always cancel on exit."""
import argparse,json,time,math,signal
from pathlib import Path as FilePath
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import PointStamped,TwistStamped,PoseStamped
from std_msgs.msg import String,Bool

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--goal',nargs=3,type=float,required=True)
    ap.add_argument('--execute-test',action='store_true');ap.add_argument('--duration',type=float,default=180.)
    ap.add_argument('--path-file',type=FilePath,help='Reuse a captured ASA route to the same goal for a controlled comparison')
    ap.add_argument('--output',type=FilePath,required=True);args=ap.parse_args()
    rclpy.init();n=rclpy.create_node('se2_supervised_goal_trial');rows=[];state={};times={};running=[True]
    signal.signal(signal.SIGTERM,lambda *_:running.__setitem__(0,False))
    def receive(k,m):
        times[k]=time.monotonic()
        if k=='body':
            p=m.pose.pose;state[k]=[p.position.x,p.position.y,p.position.z,2*math.atan2(p.orientation.z,p.orientation.w)]
        elif k=='cmd':state[k]=[m.twist.linear.x,m.twist.linear.y,m.twist.angular.z]
        elif k=='blocked':state[k]=m.data
        else:
            try:state[k]=json.loads(m.data)
            except ValueError:state[k]=m.data
    retained=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for k,topic,typ,q in [('global','/se2_global/status',String,retained),('body','/se2_scan/aligned_odom',Odometry,qos_profile_sensor_data),('sender','/se2_scan/m20_status',String,1),('route','/se2_scan/route_status',String,1),('control','/se2_scan/control_status',String,1),('blocked','/se2_scan/blocked',Bool,1),('cmd','/se2_scan/cmd_vel_stamped',TwistStamped,qos_profile_sensor_data)]:
        n.create_subscription(typ,topic,lambda m,k=k:receive(k,m),q)
    goal=n.create_publisher(PointStamped,'/clicked_point',10);cancel=n.create_publisher(Bool,'/se2_scan/cancel',10)
    route=n.create_publisher(Path,'/se2_global/path',retained) if args.path_file else None
    result={'state':'not_started','goal':args.goal,'execute_test':args.execute_test}
    def spin(seconds):
        end=time.monotonic()+seconds
        while running[0] and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
    try:
        end=time.monotonic()+90
        while running[0] and time.monotonic()<end:
            spin(.1)
            if (all(k in state for k in ('global','body','sender')) and
                time.monotonic()-times['body']<.2 and
                time.monotonic()-times['sender']<.2 and
                state['sender'].get('motion_age',1e9)<.2):break
        assert (all(k in state for k in ('global','body','sender')) and
                time.monotonic()-times['body']<.2 and
                time.monotonic()-times['sender']<.2 and
                state['sender'].get('motion_age',1e9)<.2),'Fresh body/height input unavailable'
        assert state['sender']['telemetry_only'] is (not args.execute_test),'Execution mode does not match explicit test flag'
        if args.execute_test:assert state['sender']['mode_confirmed'] and not state['sender']['failed'],state['sender']
        initial=state['global']['generation'];origin=list(state['body']);result['origin']=origin
        if args.path_file:
            poses=json.loads(args.path_file.read_text())['path']
            assert poses and math.dist(poses[-1][:3],args.goal)<.2,'Captured path has a different goal'
            m=Path();m.header.frame_id='se2_scan_map';m.header.stamp=n.get_clock().now().to_msg()
            for v in poses:
                p=PoseStamped();p.header=m.header;p.pose.position.x,p.pose.position.y,p.pose.position.z=map(float,v[:3])
                p.pose.orientation.z=math.sin(v[3]/2);p.pose.orientation.w=math.cos(v[3]/2);m.poses.append(p)
            route.publish(m);result['reused_asa_path']=str(args.path_file)
        else:
            m=PointStamped();m.header.frame_id='se2_scan_map';m.point.x,m.point.y,m.point.z=args.goal;goal.publish(m)
        start=last_report=time.monotonic();result['state']='timeout';best=math.inf;progress_at=start;planned_at=start if args.path_file else None
        while running[0] and time.monotonic()-start<args.duration:
            spin(.1);now=time.monotonic()
            g=state.get('global',{})
            if g.get('generation',0)>initial:
                if g.get('state') in ('no_path','invalid_start'):result['state']=g['state'];break
                if g.get('state')=='success' and planned_at is None:planned_at=now;progress_at=now
            body=state['body'];distance=math.dist(body[:2],args.goal[:2])
            if distance<best-.05:best=distance;progress_at=now
            if state.get('route')=='goal_reached':
                result['state']='reached' if distance<.2 else 'incorrect_completion';break
            if planned_at is not None and now-progress_at>45:result['state']='stalled';break
            if now-last_report>=5:
                row=dict(elapsed=round(now-start,2),distance=distance,**state);rows.append(row)
                print(json.dumps({k:v for k,v in row.items() if k not in ('global','sender')},ensure_ascii=False),flush=True);last_report=now
        result.update(final=state,elapsed=time.monotonic()-start,displacement=math.dist(origin[:2],state['body'][:2]))
    finally:
        cancel.publish(Bool(data=True));spin(.6);cancel.publish(Bool(data=True))
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(dict(result=result,samples=rows),indent=2,ensure_ascii=False))
        print(json.dumps(result,ensure_ascii=False),flush=True)
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
