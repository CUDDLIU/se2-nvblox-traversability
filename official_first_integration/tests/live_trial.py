#!/usr/bin/env python3
"""Bounded supervised hardware trial of an owned FIRST session; no velocity injection."""
import argparse,json,math,os,signal,time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('session',type=Path)
    ap.add_argument('--goal',type=float,nargs=3,required=True)
    ap.add_argument('--max-displacement',type=float,default=1.)
    ap.add_argument('--seconds',type=float,default=120.)
    ap.add_argument('--expected-start',type=float,nargs=4,default=[-.19,-.18,0.,0.])
    args=ap.parse_args()
    if not 0<args.max_displacement<=10 or not 0<args.seconds<=180:ap.error('invalid trial bounds')
    doc=json.loads((args.session/'session.json').read_text())
    assert doc['execute'] and doc['state']=='execution_waiting_goal',doc
    pid=doc['launcher_pid']
    argv=Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
    assert len(argv)>1 and argv[1]==os.fsencode(Path(__file__).resolve().parents[1]/'run.py'),'session owner changed'
    rclpy.init();node=rclpy.create_node('first_supervised_trial')
    state={};at={};trace=(args.session/'trial.jsonl').open('w')
    def record(name,value):state[name]=value;at[name]=time.monotonic()
    def odom(m):
        p,q=m.pose.pose.position,m.pose.pose.orientation
        if m.header.frame_id=='se2_scan_map':record('body',[p.x,p.y,p.z,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))])
    node.create_subscription(Odometry,'/first/aligned_odom',odom,qos_profile_sensor_data)
    node.create_subscription(String,'/first/status',lambda m:record('planner',json.loads(m.data)),10)
    node.create_subscription(String,'/first/m20_status',lambda m:record('robot',json.loads(m.data)),10)
    clouds={};motion={}
    node.create_subscription(PointCloud2,'/first/occupancy',lambda m:clouds.update(occupied=m),qos_profile_sensor_data)
    node.create_subscription(PointCloud2,'/LIDAR/POINTS_NX',lambda m:clouds.update(raw=m),qos_profile_sensor_data)
    node.create_subscription(String,doc['remaps']['/nvblox_robot/motion_status/raw'],lambda m:motion.update(json.loads(m.data)),10)
    pub=node.create_publisher(PointStamped,'/clicked_point',10)
    began=time.monotonic();sent=None;origin=None;active=None;max_d=0.;reason='timeout';last_log=0.;blocked_since=None
    try:
        while time.monotonic()-began<args.seconds:
            rclpy.spin_once(node,timeout_sec=.02);now=time.monotonic()
            if not Path(f'/proc/{pid}').exists():reason='session_exited';break
            p=state.get('planner',{});robot=state.get('robot',{})
            if now-last_log>=.5:
                item=dict(time_ns=time.time_ns(),**state,displacement_m=max_d)
                trace.write(json.dumps(item)+'\n');trace.flush();print(json.dumps(item),flush=True);last_log=now
            if sent is None:
                if not all(now-at.get(k,-1e9)<1.2 for k in ('body','planner','robot')):continue
                if p.get('cloud_age_s',99)>.3 or p.get('localization_age_s',99)>1.:continue
                if robot.get('failed') or robot.get('basic',{}).get('ControlUsageMode')!=1:continue
                ready=args.session/'global/status.json'
                if not ready.exists() or json.loads(ready.read_text()).get('state')!='ready' or not pub.get_subscription_count():continue
                b=state['body']
                expected=args.expected_start
                if math.hypot(b[0]-expected[0],b[1]-expected[1])>.5 or abs(b[2]-expected[2])>.25 or abs(math.remainder(b[3]-expected[3],2*math.pi))>.3:
                    reason='start_pose_outside_confirmed_region';break
                origin=b.copy();m=PointStamped();m.header.frame_id='se2_scan_map';m.header.stamp=node.get_clock().now().to_msg()
                m.point.x,m.point.y,m.point.z=args.goal;pub.publish(m);sent=now
                print('GOAL_SENT '+json.dumps(args.goal),flush=True)
            else:
                if now-at.get('body',-1e9)>1. or now-at.get('robot',-1e9)>2.:
                    reason='monitor_lost_live_feedback';break
                b=state['body'];max_d=max(max_d,math.hypot(b[0]-origin[0],b[1]-origin[1]))
                if max_d>=args.max_displacement:reason='distance_limit_reached';break
                if robot.get('failed'):reason='robot_error_latched';break
                if p.get('state')=='arrived':reason='arrived';break
                if robot.get('nonzero_packets',0)>0 and active is None:active=now
                if p.get('state')=='blocked':
                    if blocked_since is None:blocked_since=now
                    if now-blocked_since>2:reason='collision_blocked';break
                else:blocked_since=None
                if active and now-active>20 and max_d<.15:reason='no_measured_progress';break
                global_file=args.session/'global/status.json'
                g=json.loads(global_file.read_text()) if global_file.exists() else {}
                if g.get('state') in ('no_path','invalid_start','invalid_query'):reason='global_'+g['state'];break
                if now-sent>70 and active is None:reason='no_executable_command';break
    finally:
        try:os.kill(pid,signal.SIGTERM)
        except ProcessLookupError:pass
        # Preserve the actual accumulated world at the instant of stopping,
        # rather than collecting a new, empty-history map after a restart.
        if clouds:
            import numpy as np
            from sensor_msgs_py.point_cloud2 import read_points
            arrays={};snapshot=dict(body=state.get('body'),motion=motion,robot=state.get('robot'),planner=state.get('planner'))
            for key,m in clouds.items():
                fields=tuple(f for f in ('x','y','z','timestamp','ring','intensity') if any(a.name==f for a in m.fields)) if key=='raw' else ('x','y','z')
                points=read_points(m,field_names=fields,skip_nans=True)
                arrays[key]=np.column_stack([points[f] for f in fields])
                snapshot[key+'_frame']=m.header.frame_id;snapshot[key+'_stamp']=m.header.stamp.sec+m.header.stamp.nanosec/1e9
            np.savez_compressed(args.session/'snapshot.npz',**arrays)
            (args.session/'snapshot.json').write_text(json.dumps(snapshot))
        result=dict(reason=reason,goal=args.goal,origin=origin,body=state.get('body'),max_displacement_m=max_d,
                    goal_sent=sent is not None,nonzero_packets=state.get('robot',{}).get('nonzero_packets',0),
                    stop_requested=True,seconds=time.monotonic()-began)
        (args.session/'trial_result.json').write_text(json.dumps(result,indent=2));print('TRIAL_RESULT '+json.dumps(result),flush=True)
        trace.close();node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
