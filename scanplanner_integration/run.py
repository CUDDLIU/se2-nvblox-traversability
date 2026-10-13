#!/usr/bin/env python3
"""Own only this session's children; default is isolated bag replay, no actuation."""
import argparse
from datetime import datetime
import json
import hashlib
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from core import SPEED_PROFILES
ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bag',type=Path);p.add_argument('--map',type=Path)
    p.add_argument('--live',action='store_true');p.add_argument('--map-aligned',action='store_true')
    p.add_argument('--localize-at-start',action='store_true',help='Match live surfaces near the saved map recording start; does not play the bag')
    p.add_argument('--initial-map-pose',nargs=4,type=float,help='Measured XYZ/yaw search seed for restarting localization; full scan matching is still required')
    p.add_argument('--execute',action='store_true');p.add_argument('--domain',type=int)
    p.add_argument('--speed-profile',choices=tuple(SPEED_PROFILES),default='normal',help='Live speed profile: normal walking or the former slow commissioning limits')
    p.add_argument('--enter-navigation',action='store_true',help='With --execute, request navigation mode once after fresh standing telemetry')
    p.add_argument('--no-rviz',action='store_true');p.add_argument('--sensors-only',action='store_true')
    p.add_argument('--duration',type=float,default=0);p.add_argument('--offset',type=float,default=0)
    p.add_argument('--path',type=Path,help='Replay a saved, verified global query JSON');p.add_argument('--no-global',action='store_true');p.add_argument('--start-paused',action='store_true')
    args=p.parse_args()
    if args.domain is None:args.domain=0 if args.live else 80
    if args.execute and (not args.live or not args.localize_at_start or args.sensors_only):
        p.error('--execute requires --live --localize-at-start and the complete planning chain')
    if args.live and not args.localize_at_start and not args.sensors_only:
        p.error('Live navigation requires measured map localization: use --localize-at-start near the map recording start')
    if args.map_aligned:p.error('--map-aligned no longer bypasses localization; use --localize-at-start')
    if args.enter_navigation and not args.execute:p.error('--enter-navigation requires --execute')
    if args.live and args.path:p.error('Choose a live goal in RViz; recorded-query injection is for offline tests only')
    if not args.live and args.bag is None:
        files=list((ROOT/'bags').glob('*/data/metadata.yaml'))
        if not files:p.error('No bags found')
        args.bag=max(files,key=lambda q:q.stat().st_mtime).parent.parent
    if not args.sensors_only and args.map is None:
        folders=list((args.bag if args.bag else ROOT/'bags').glob('**/global_plans/*/navmesh.json'))
        if not folders:p.error('No built global map found; specify --map')
        args.map=max(folders,key=lambda q:q.stat().st_mtime).parent
    dds_profile=(Path(os.environ.get('SE2_SCAN_LIVE_DDS_PROFILE',str(Path.home()/'.config/fastdds/eno1.xml')))
                 if args.live else ROOT/'terrain_variants/fastdds_large_shm.xml')
    if not dds_profile.is_file():p.error(f'DDS profile not found: {dds_profile}; set SE2_SCAN_LIVE_DDS_PROFILE for live inputs')
    out=ROOT/'scanplanner_integration/runtime'/datetime.now().strftime('%Y%m%d_%H%M%S')
    out.mkdir(parents=True)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain),ROS_LOCALHOST_ONLY='0' if args.live else '1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',OMP_WAIT_POLICY='PASSIVE',FASTRTPS_DEFAULT_PROFILES_FILE=str(dds_profile))
    os.environ.pop('FASTDDS_DEFAULT_PROFILES_FILE',None)
    os.environ['RMW_FASTRTPS_PUBLICATION_MODE']='ASYNCHRONOUS'
    children={};logs=[];running=True;rviz_restart_requested=False;rviz_command=None
    def stop(*_):
        nonlocal running
        running=False
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    def restart_rviz(*_):
        nonlocal rviz_restart_requested
        rviz_restart_requested=True
    signal.signal(signal.SIGUSR1,restart_rviz)
    def spawn(name,cmd,append=False):
        f=(out/(name+'.log')).open('a' if append else 'w');logs.append(f)
        children[name]=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
        return children[name]
    sim='false' if args.live else 'true'
    common=['--ros-args','--params-file',str(ROOT/'scanplanner_integration/planner.yaml'),'-p','use_sim_time:='+sim,'-r','__ns:=/se2_scan']
    localized=args.live and not args.sensors_only
    frame='se2_scan_map' if localized else 'nvblox_odom'
    odom='/se2_scan/aligned_odom' if localized else '/nvblox_lio/odom'
    common+=['-p','world_frame:='+frame,'-p','grid_map.frame_id:='+frame]
    if localized:common+=['-p','use_map_alignment:=true']
    if args.live:
        speed=SPEED_PROFILES[args.speed_profile]
        common+=['-p',f'max_forward_speed:={speed["forward"]}','-p',f'max_lateral_speed:={speed["lateral"]}',
                 '-p',f'max_yaw_speed:={speed["yaw"]}','-p',f'control_lookahead_s:={speed["lookahead"]}',
                 '-p','min_in_place_yaw_speed:=0.5',
                 '-p',f'manager.max_vel:={speed["forward"]}','-p',f'optimization.max_vel:={speed["forward"]}']
    remaps=['--ros-args','-p','use_sim_time:='+sim,'-r','/nvblox_lio/odom:='+odom]
    report=dict(source_sha256={str(q.relative_to(ROOT)):hashlib.sha256(q.read_bytes()).hexdigest() for q in (ROOT/'scanplanner_integration').rglob('*') if q.is_file() and q.suffix in ('.py','.cpp','.yaml','.patch') and 'runtime' not in q.parts and '__pycache__' not in q.parts},state='running',arguments={k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()},outbound_robot_commands=bool(args.execute),children={})
    (out/'session.json').write_text(json.dumps(report,indent=2))
    print('SESSION '+str(out),flush=True)
    try:
        if args.live:
            from live_inputs import ensure_live_inputs
            report['live_inputs']=ensure_live_inputs(ROOT,out,spawn,lambda:running)
            if not running:report['state']='complete';return
            spawn('m20',[sys.executable,str(ROOT/'scanplanner_integration/m20_udp.py'),
                         '--execute' if args.execute else '--telemetry-only','--output',str(out/'telemetry'),
                         *(['--enter-navigation'] if args.enter_navigation else [])])
        if localized:
            spawn('localization',[sys.executable,str(ROOT/'scanplanner_integration/localization_node.py'),str(args.map),'--output',str(out/'localization'),
                  *(['--initial-map-pose',*map(str,args.initial_map_pose)] if args.initial_map_pose else [])])
        spawn('bridge',['ros2','run','se2_scan_bridge','scan_bridge',*common,
                       # Cover the 1.5 m/s profile and its longer braking sweep.
                       # Out-of-map voxels still reject motion; the optimizer
                       # map and shared cloud retain their 4 m range.
                       '-p','grid_map.max_ray_length:=4.0','-p','preprocess_ray_length:=4.0',
                       # The guard checks raw occupancy with the full rectangle.
                       # Its unused cylinder-inflation layer need not be built.
                       '-p','grid_map.compute_inflation:=false',
                       '-p','grid_map.resolution:=0.05',
                       '-p','grid_map.sliding_map_size_x:=8.0','-p','grid_map.sliding_map_size_y:=8.0',
                       '-p','grid_map.sliding_map_size_z:=2.0',
                       '-r','raw_cloud:=/LIDAR/POINTS_NX','-r','raw_body_pose:=/nvblox_lio/odom',
                       '-r','cloud:=/se2_scan/unused_map_cloud','-r','sensor_pose:=/se2_scan/unused_map_pose',
                       '-r','grid_map/occupancy:=guard/occupancy','-r','grid_map/occupancy_inflate:=guard/occupancy_inflate',
                       '-r','grid_map/sliding_map_bbox:=guard/sliding_map_bbox'])
        spawn('metrics',[sys.executable,str(ROOT/'scanplanner_integration/metrics.py'),str(out)])
        if not args.sensors_only:
            spawn('optimizer',['ros2','run','scan_planner','scan_planner_node',*common,
                  '-r','initial_path:=reference','-r','cloud:=world_cloud','-r','sensor_pose:=body_pose',
                  '-p','grid_map.sliding_map_frame_id:=se2_scan_optimizer_map'])
            spawn('route',[sys.executable,str(ROOT/'scanplanner_integration/route_node.py'),str(args.map),
                          '--frame',frame,*remaps,'-p','guard_resolution:=0.05'])
            if not args.no_global:
                spawn('global',[sys.executable,str(ROOT/'global_planner/ros_node.py'),str(args.map),
                                '--robot-start','--runtime',str(out/'global'),'--frame',frame,*remaps])
        if not args.no_rviz:
            config=ROOT/'scanplanner_integration/scan.rviz'
            if localized:
                import yaml
                document=yaml.safe_load(config.read_text());vm=document['Visualization Manager'];vm['Global Options']['Fixed Frame']=frame
                for display in vm['Displays']:
                    if display.get('Class')=='rviz_default_plugins/Odometry':display['Topic']['Value']=odom;display['Name']='M20 实时地图定位'
                vm['Views']['Current'].update(Distance=9.,Pitch=.8,Yaw=.2,**{'Focal Point':dict(X=0.,Y=0.,Z=0.)})
                config=out/'live.rviz';config.write_text(yaml.safe_dump(document,allow_unicode=True,sort_keys=False))
            rviz_command=['rviz2','-d',str(config),'--ros-args','-p','use_sim_time:='+sim]
            spawn('rviz',rviz_command)
        # Allow subscriptions/geometry to initialize before reading recorded frames.
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import PointCloud2
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=rclpy.create_node('se2_scan_session')
        global_ready=[args.no_global or args.sensors_only]
        auto_pause=dict(owned=False,pending=False,generation=None,state='ready')
        if not global_ready[0]:
            from std_msgs.msg import String
            from rclpy.qos import QoSProfile,DurabilityPolicy
            from rosbag2_interfaces.srv import Pause,Resume,IsPaused
            pause=node.create_client(Pause,'/rosbag2_player/pause')
            resume=node.create_client(Resume,'/rosbag2_player/resume')
            paused=node.create_client(IsPaused,'/rosbag2_player/is_paused')
            def global_status(message):
                global_ready[0]=True
                doc=json.loads(message.data);state=doc.get('state');auto_pause['state']=state
                if args.live:return
                if state=='planning' and not auto_pause['owned'] and not auto_pause['pending'] and paused.service_is_ready():
                    auto_pause['pending']=True
                    def checked(future):
                        auto_pause['pending']=False
                        if future.exception() or future.result().paused or auto_pause['state']!='planning':return
                        def did_pause(f):
                            if not f.exception():
                                auto_pause['owned']=True
                                if auto_pause['state']!='planning':
                                    resume.call_async(Resume.Request());auto_pause['owned']=False
                        pause.call_async(Pause.Request()).add_done_callback(did_pause)
                    paused.call_async(IsPaused.Request()).add_done_callback(checked)
                elif state in ('success','no_path','invalid_query','ready') and auto_pause['owned']:
                    resume.call_async(Resume.Request());auto_pause['owned']=False
            node.create_subscription(String,'/se2_global/status',global_status,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        until=time.monotonic()+60
        while running and time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.1)
            bridge=any(s.node_name=='scan_bridge' for s in node.get_subscriptions_info_by_topic('/LIDAR/POINTS_NX'))
            route=args.sensors_only or any(s.node_name=='se2_scan_route' for s in node.get_subscriptions_info_by_topic('/se2_global/path'))
            if bridge and route and global_ready[0]:break
        else:
            if not running:report['state']='complete';return
            raise RuntimeError('Nodes did not become ready')
        path_pub=None
        if args.path:
            import math
            from nav_msgs.msg import Path as RosPath
            from geometry_msgs.msg import PoseStamped
            from rclpy.qos import QoSProfile,DurabilityPolicy
            path_pub=node.create_publisher(RosPath,'/se2_global/path',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            path=RosPath();path.header.frame_id='nvblox_odom'
            for v in json.loads(args.path.read_text())['poses']:
                pose=PoseStamped();pose.header=path.header
                pose.pose.position.x=float(v[0]);pose.pose.position.y=float(v[1]);pose.pose.position.z=float(v[2])
                pose.pose.orientation.z=math.sin(v[3]/2);pose.pose.orientation.w=math.cos(v[3]/2);path.poses.append(pose)
            path_pub.publish(path)
        if not args.live:
            topics=['/nvblox_lio/odom','/LIDAR/POINTS_NX','/nvblox_robot/motion_status/raw']
            cmd=['ros2','bag','play',str(args.bag/'data'),'--clock','50','--disable-keyboard-controls','--read-ahead-queue-size','3000','--start-offset',str(args.offset)]
            if args.start_paused:cmd+=['--start-paused']
            cmd+=['--topics',*topics]
            spawn('player',cmd)
        began=time.monotonic();eof=None
        report['children']={k:v.pid for k,v in children.items()};(out/'session.json').write_text(json.dumps(report,indent=2))
        while running:
            rclpy.spin_once(node,timeout_sec=.1)
            if args.duration and time.monotonic()-began>=args.duration:break
            if rviz_restart_requested:
                rviz_restart_requested=False
                if rviz_command is not None:
                    proc=children['rviz']
                    if proc.poll() is None:
                        os.killpg(proc.pid,signal.SIGINT)
                        try:proc.wait(timeout=8)
                        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=3)
                    spawn('rviz',rviz_command,append=True)
                    report['children']['rviz']=children['rviz'].pid
                    report['rviz_restarts']=report.get('rviz_restarts',0)+1
                    (out/'session.json').write_text(json.dumps(report,indent=2))
                    print('RViz window restarted; localization and planning processes retained.',flush=True)
            for name,proc in children.items():
                if proc.poll() is not None:
                    if name=='player' and proc.returncode==0:
                        eof=eof or time.monotonic()
                        if time.monotonic()-eof>1.:running=False
                        continue
                    raise RuntimeError(f'{name} exited ({proc.returncode}); see {out/name}.log')
        node.destroy_node();rclpy.shutdown();report['state']='complete'
    except Exception as exc:
        report.update(state='failed',error=str(exc));raise
    finally:
        # Stop sources/optimizer before the bridge's last zero command; close UDP last.
        for name in [k for k in children if k not in ('bridge','metrics','m20','lio')]+['bridge','metrics','m20','lio']:
            proc=children.get(name)
            if proc is None or proc.poll() is not None:continue
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=3)
        for f in logs:f.close()
        (out/'session.json').write_text(json.dumps(report,indent=2))
        print('RESULT '+str(out),flush=True)
if __name__=='__main__':main()
