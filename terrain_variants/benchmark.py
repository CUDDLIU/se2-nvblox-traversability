"""Sequential isolated-domain experiments, preserving raw bag and real timing."""
import argparse
import hashlib
from datetime import datetime
import json
import os
import re
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

import numpy as np
import yaml


def percentiles(values):
    a=np.asarray([v for v in values if v is not None],dtype=float)
    return dict(n=len(a),p50=float(np.percentile(a,50)),p95=float(np.percentile(a,95)),
                p99=float(np.percentile(a,99)),maximum=float(a.max())) if len(a) else dict(n=0)


def effective_weights(log_text):
    """First mapper's actual integrators, not the ROS parameter's requested value."""
    result={};sensor=None
    for line in log_text.splitlines():
        name=line.strip()
        if name in ('camera_tsdf_integrator','lidar_tsdf_integrator'):
            sensor=name.split('_')[0]
        elif sensor and 'weighting_function_type::' in name:
            result.setdefault(sensor,name.split('::',1)[1].strip())
            sensor=None
        if len(result)==2:break
    applied=re.findall(r'SE2_EFFECTIVE_LIDAR_WEIGHTING=(\w+)',log_text)
    if applied:result['lidar']=applied[-1]
    return result


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('mode',choices=('depth','fusion','lidar'))
    parser.add_argument('--bag',default='20260928_144814_b2e209')
    parser.add_argument('--label',default='baseline')
    parser.add_argument('--implementation',default='baseline')
    parser.add_argument('--tile-cells',type=int,default=16)
    parser.add_argument('--resolution',type=float,default=.1)
    parser.add_argument('--normal-filter',action='store_true')
    parser.add_argument('--riser-filter',action='store_true')
    parser.add_argument('--adaptive-clearance',action='store_true')
    parser.add_argument('--start-offset',type=float,default=0.)
    parser.add_argument('--read-ahead',type=int,default=5000)
    parser.add_argument('--trace-mesh',action='store_true')
    parser.add_argument('--trace-roi',type=float,nargs=6,metavar=('XMIN','XMAX','YMIN','YMAX','ZMIN','ZMAX'))
    parser.add_argument('--trace-times',type=int,nargs='+')
    parser.add_argument('--duration',type=float,default=0)
    parser.add_argument('--rate',type=float,default=1)
    parser.add_argument('--voxel',type=float,default=.05)
    parser.add_argument('--truncation',type=float,default=4.)
    parser.add_argument('--min-mesh-weight',type=float,default=.0001)
    parser.add_argument('--weighting',choices=('constant','constant_dropoff','inverse_square',
        'inverse_square_dropoff','inverse_square_tsdf_distance_penalty','linear_with_max'),
        help='Camera TSDF weighting in this installed nvblox; LiDAR keeps its own default')
    parser.add_argument('--lidar-weighting',choices=('inverse_square','inverse_square_tsdf_distance_penalty'),
        help='Experimental verified LiDAR-only integrator setting; installed callback retained')
    parser.add_argument('--mesh-mbps',type=float,default=30.)
    parser.add_argument('--mesh-radius',type=float,default=5.)
    parser.add_argument('--depth-rate',type=float,default=15.,help='Integration ceiling; use >15 Hz to avoid jitter throttling 15 Hz camera frames')
    parser.add_argument('--clear-radius',type=float,default=10.)
    parser.add_argument('--max-step',type=float,default=.03)
    parser.add_argument('--slope',type=float,default=15)
    parser.add_argument('--mesh-only',action='store_true',help='Disable unused ESDF/debug/decay work')
    parser.add_argument('--range',type=float,default=6.)
    parser.add_argument('--timings',action='store_true')
    parser.add_argument('--fast-lidar',action='store_true',help='Use locally built bulk pointcloud converter')
    parser.add_argument('--self-filter',action='store_true',help='Exclude known M20 body/leg envelope from LiDAR TSDF input')
    parser.add_argument('--deskew',action='store_true',help='Compensate each LiDAR point using recorded 200 Hz odometry')
    parser.add_argument('--restore-scan-time',action='store_true',help='Experimental inversion of verified two-period timestamp compression, nvblox input only')
    parser.add_argument('--multi-lidar',action='store_true',help='Experimental physical front/rear ray origins for this verified M20 bag')
    parser.add_argument('--lidar-native-axes',action='store_true',help='Use physical LiDAR spindle axes and measured 0.4 degree / 96-ring projection')
    parser.add_argument('--lidar-optical-origin',action='store_true',help='Use AIRY axial optical-plane offset inside native-axis nvblox projection')
    parser.add_argument('--large-shm',action='store_true',help='Use isolated 64 MiB shared-memory transport for large sensor messages')
    parser.add_argument('--capture-mesh',action='store_true',help='Save sparse diagnostic Mesh snapshots; adds measurement overhead')
    parser.add_argument('--rviz',action='store_true')
    args=parser.parse_args()
    if args.multi_lidar and (not args.fast_lidar or not args.deskew or args.mode=='depth'):
        parser.error('--multi-lidar requires --fast-lidar --deskew and LiDAR input')
    if args.lidar_native_axes and not args.multi_lidar:
        parser.error('--lidar-native-axes requires --multi-lidar')
    if args.lidar_optical_origin and not args.lidar_native_axes:
        parser.error('--lidar-optical-origin requires --lidar-native-axes')
    if args.restore_scan_time and not args.deskew:
        parser.error('--restore-scan-time requires --deskew')
    if args.weighting and args.mode=='lidar':
        parser.error('--weighting affects only camera TSDF in this installed version; use fusion or depth')
    if args.lidar_weighting and (args.mode=='depth' or args.multi_lidar):
        parser.error('--lidar-weighting requires LiDAR input and cannot be combined with --multi-lidar')
    here=Path(__file__).resolve().parent;root=here.parent
    bag=root/'bags'/args.bag
    if bag.resolve().parent!=(root/'bags').resolve():raise ValueError('invalid bag')
    for unit in ('nvblox-d435i-shadow','se2-terrain-tuner'):
        if subprocess.run(['systemctl','--user','is-active','--quiet',unit]).returncode==0:
            raise RuntimeError('请先关闭实时深度建图与调参管理器：'+unit)
    # Domain isolation is established before loading ROS.
    os.environ.update(ROS_DOMAIN_ID='74',ROS_LOCALHOST_ONLY='1',OPENBLAS_NUM_THREADS='1',
                      OMP_NUM_THREADS='3',OMP_WAIT_POLICY='PASSIVE',TERRAIN_IMPLEMENTATION=args.implementation)
    os.environ.pop('FASTRTPS_DEFAULT_PROFILES_FILE',None);os.environ.pop('FASTDDS_DEFAULT_PROFILES_FILE',None)
    if args.large_shm:os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']=str(here/'fastdds_large_shm.xml')
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry
    from nvblox_msgs.msg import Mesh
    from rosgraph_msgs.msg import Clock
    from rosbag2_interfaces.srv import Resume
    sys.path.insert(0,str(here/'baseline'))
    from mesh_wire import mesh_summary
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('terrain_variant_benchmark')
    for _ in range(20):rclpy.spin_once(node,timeout_sec=.1)
    others=[name for name in node.get_node_names() if name!=node.get_name()]
    if others:raise RuntimeError('实验 domain 74 已被占用：'+str(others))
    tag=datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+args.mode+'_'+uuid.uuid4().hex[:5]
    out=bag/'experiments'/tag;out.mkdir(parents=True)
    print('EXPERIMENT '+str(out),flush=True)
    metadata=yaml.safe_load((bag/'data/metadata.yaml').read_text())['rosbag2_bagfile_information']
    bag_start=metadata['starting_time']['nanoseconds_since_epoch']/1e9
    sys.path.insert(0,str(root/'bag_tools'))
    from rebuild_config import write_configs
    from replay_transforms import prepare
    prepare(bag,out)
    terrain=root.parent/'se2_terrain_check'
    mapper_path,checker_path=write_configs(root,terrain,bag,out,dict(max_step=args.max_step,max_slope_deg=args.slope))
    checker_doc=yaml.safe_load(checker_path.read_text())
    checker_doc['se2_terrain_check']['ros__parameters']['tile_cells']=args.tile_cells
    checker_doc['se2_terrain_check']['ros__parameters']['resolution']=args.resolution
    if args.normal_filter:checker_doc['se2_terrain_check']['ros__parameters']['surface_normal_filter']=True
    if args.riser_filter:checker_doc['se2_terrain_check']['ros__parameters']['stair_riser_filter']=True
    if args.adaptive_clearance:checker_doc['se2_terrain_check']['ros__parameters']['adaptive_clearance']=True
    checker_path.write_text(yaml.safe_dump(checker_doc,sort_keys=False))
    doc=yaml.safe_load(mapper_path.read_text());p=doc['nvblox_node']['ros__parameters']
    p.update(use_depth=args.mode!='lidar',use_lidar=args.mode!='depth',
             use_lidar_motion_compensation=False,voxel_size=args.voxel,
             integrate_depth_rate_hz=args.depth_rate,
             publish_layer_rate_hz=15.,integrate_lidar_rate_hz=15.,
             print_queue_drops_to_console=True,print_delays_to_console=True,
             layer_streamer_bandwidth_limit_mbps=args.mesh_mbps,
             layer_visualization_exclusion_radius_m=args.mesh_radius,
             map_clearing_radius_m=args.clear_radius,
             print_timings_to_console=args.timings)
    p['static_mapper'].update(projective_integrator_max_integration_distance_m=args.range,
        lidar_projective_integrator_max_integration_distance_m=args.range,
        projective_integrator_truncation_distance_vox=args.truncation,
        mesh_integrator_min_weight=args.min_mesh_weight)
    if args.weighting:
        p['static_mapper']['projective_integrator_weighting_mode']=args.weighting
    if args.mesh_only:
        p.update(update_esdf_rate_hz=0.,publish_debug_vis_rate_hz=0.,decay_tsdf_rate_hz=0.,
                 publish_esdf_distance_slice=False)
    if args.lidar_native_axes:
        p.update(lidar_width=900,lidar_height=96,use_non_equal_vertical_fov_lidar_params=True,
                 min_angle_below_zero_elevation_rad=.05,max_angle_above_zero_elevation_rad=1.5707963267948966)
    mapper_path.write_text(yaml.safe_dump(doc,sort_keys=False))
    manifest=dict(arguments=vars(args),bag=args.bag,bag_start=bag_start,state='starting',
        measurement='wall-clock input receipt to result; separate geometry, replacements and fully completed inputs',
        fusion='native shared TSDF baseline',lidar_motion_compensation=args.deskew,control_outputs='none')
    manifest['source_sha256']={str(p.relative_to(here)):hashlib.sha256(p.read_bytes()).hexdigest()
        for directory in (here/args.implementation,here/'fast_lidar') for p in directory.iterdir()
        if p.suffix in ('.py','.cpp','.cu','.h','.hpp','.sh','.so')}
    for p in (here/'benchmark.py',here/'checker_node.py'):
        manifest['source_sha256'][str(p.relative_to(here))]=hashlib.sha256(p.read_bytes()).hexdigest()
    if args.large_shm:
        p=here/'fastdds_large_shm.xml';manifest['source_sha256'][p.name]=hashlib.sha256(p.read_bytes()).hexdigest()
    if args.deskew:
        for p in (here/'deskew').iterdir():
            if p.is_file() and p.suffix in ('.cpp','.hpp','.txt'):
                manifest['source_sha256'][str(p.relative_to(here))]=hashlib.sha256(p.read_bytes()).hexdigest()
        binary=here/'deskew/build/terrain_lidar_deskew'
        manifest['source_sha256'][str(binary.relative_to(here))]=hashlib.sha256(binary.read_bytes()).hexdigest()
    if args.multi_lidar:
        for p in (here/'multi_lidar').iterdir():
            if p.is_file() and p.suffix in ('.cpp','.hpp','.sh','.so','.sha256'):
                manifest['source_sha256'][str(p.relative_to(here))]=hashlib.sha256(p.read_bytes()).hexdigest()
        manifest['lidar_origins']='ring 0..95: (+.32028,0,-.013); 96..191: (-.32028,0,-.013); axes body-aligned'
        if args.lidar_native_axes:manifest['lidar_origins']+='; transformed to sensor spindle axes'
        if args.lidar_optical_origin:
            manifest['lidar_origins']='ring 0..95: (+.36560,0,-.013); 96..191: (-.36560,0,-.013); axial optical planes; radial origin approximated'
    if args.lidar_weighting:
        for p in (here/'weighting').iterdir():
            if p.is_file() and p.suffix in ('.cpp','.sh','.so','.sha256'):
                manifest['source_sha256'][str(p.relative_to(here))]=hashlib.sha256(p.read_bytes()).hexdigest()
    (out/'experiment.json').write_text(json.dumps(manifest,indent=2))
    file=(out/'metrics.jsonl').open('w');children={};logs=[];playing=None;running=True
    state=dict(clock=None,mesh=[],replacements=[],status=[],odom=[],sensor_counts={},resources=[])
    def emit(kind,data):
        row=dict(kind=kind,monotonic=time.monotonic(),clock=state['clock'],data=data)
        file.write(json.dumps(row,separators=(',',':'))+'\n')
    trace=None
    if args.trace_mesh:
        from trace_mesh_changes import MeshTrace
        trace=MeshTrace(out,bag_start,args.trace_roi,args.trace_times)
    def mesh(raw):
        d=mesh_summary(raw);now=time.monotonic()
        if d['vertices']>0:state['mesh'].append((now,d['stamp']))
        emit('mesh',d)
        if trace is not None:trace.update(raw,state['clock'],state['odom'][-1] if state['odom'] else None)
    def local(msg):
        d=json.loads(msg.data)
        if d.get('event')=='replace':
            state['replacements'].append(dict(t=time.monotonic(),sequence=d.get('input_sequence'),
                oldest=d.get('latency_oldest_ms'),newest=d.get('latency_newest_ms'),
                callback=d.get('callback_ms'),complete=d.get('complete_input'),
                nonempty=any(s.get('cells') for s in d.get('updated_slabs',[]))))
        emit('local',d)
    def status(msg):
        d=json.loads(msg.data);state['status'].append(d);emit('status',d)
    def odom(msg):
        v=msg.pose.pose.position;q=msg.pose.pose.orientation
        d=dict(stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,position=[v.x,v.y,v.z],rotation=[q.x,q.y,q.z,q.w])
        state['odom'].append(d);emit('odom',d)
    retained=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions=[node.create_subscription(Mesh,'/nvblox_node/mesh',mesh,10,raw=True),
        node.create_subscription(String,'/se2_navmesh/local_result',local,100),
        node.create_subscription(String,'/se2_terrain/status',status,retained),
        node.create_subscription(Odometry,'/nvblox_lio/odom_lidar',odom,qos_profile_sensor_data),
        node.create_subscription(Clock,'/clock',lambda m:state.update(clock=m.clock.sec+m.clock.nanosec/1e9),qos_profile_sensor_data)]
    def spawn(name,command,env=None):
        log=(out/(name+'.log')).open('w');logs.append(log)
        children[name]=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,env=env)
    def stop(*_):
        nonlocal running
        running=False
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    result='FAIL';error=None
    try:
        spawn('timed_tf',['/usr/bin/python3',str(root/'bag_tools/replay_transforms.py'),'run',str(out)])
        model=json.loads((out/'timed_transforms.json').read_text())['model']
        if model:
            spawn('robot_model',['/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher',
                '--ros-args','-r','__node:=se2_bag_measured_model','--params-file',str(out/'recorded_robot.yaml'),
                '-r','joint_states:=/bag_rebuild/joint_states_timed',
                '-r','robot_description:=/bag_rebuild/robot_description'])
        if args.start_offset:
            # Seeking skips latched messages at bag start. Restore only recorded
            # static edges; never fabricate transforms for a diagnostic seek.
            import sqlite3
            from tf2_msgs.msg import TFMessage
            from tf2_ros import StaticTransformBroadcaster
            from rclpy.serialization import deserialize_message
            edges={}
            with sqlite3.connect((bag/'data'/metadata['relative_file_paths'][0]).resolve().as_uri()+'?mode=ro',uri=True) as db:
                for (raw,) in db.execute("SELECT data FROM messages WHERE topic_id IN (SELECT id FROM topics WHERE name='/tf_static')"):
                    for tf in deserialize_message(raw,TFMessage).transforms:
                        if not (model and tf.child_frame_id.startswith('nvblox_robot/')):
                            edges[tf.child_frame_id]=tf
            if not edges:raise RuntimeError('No recorded static transforms for seek')
            static_broadcaster=StaticTransformBroadcaster(node)
            static_broadcaster.sendTransform(list(edges.values()))
            (out/'seek_static_frames.json').write_text(json.dumps(sorted(edges)))
        mapper_env=os.environ.copy()
        for flag in ('SE2_LIDAR_NATIVE_AXES','SE2_LIDAR_OPTICAL_ORIGIN',
                     'SE2_LIDAR_TSDF_WEIGHTING','SE2_LIDAR_SELF_FILTER'):
            mapper_env.pop(flag,None)
        if args.fast_lidar:
            library=here/'fast_lidar/libfast_lidar.so'
            if not library.is_file():raise RuntimeError('Build fast_lidar/build.sh on the Jetson first')
            mapper_env['LD_PRELOAD']=str(library)
        if args.multi_lidar:
            library=here/'multi_lidar/libmulti_lidar.so'
            expected=(here/'multi_lidar/installed_library.sha256').read_text().split()[0]
            actual=hashlib.sha256(Path('/opt/ros/humble/lib/libnvblox_ros_lib.so').read_bytes()).hexdigest()
            if expected!=actual:raise RuntimeError('nvblox ABI changed; rebuild multi_lidar locally')
            mapper_env['LD_PRELOAD']=str(library)+':'+mapper_env['LD_PRELOAD']
            if args.lidar_native_axes:mapper_env['SE2_LIDAR_NATIVE_AXES']='1'
            if args.lidar_optical_origin:mapper_env['SE2_LIDAR_OPTICAL_ORIGIN']='1'
        if args.lidar_weighting:
            library=here/'weighting/liblidar_weight.so'
            expected=(here/'weighting/installed_library.sha256').read_text().split()[0]
            actual=hashlib.sha256(Path('/opt/ros/humble/lib/libnvblox_ros_lib.so').read_bytes()).hexdigest()
            if expected!=actual:raise RuntimeError('nvblox ABI changed; rebuild weighting locally')
            if not library.is_file():raise RuntimeError('Build weighting/build.sh on Jetson first')
            mapper_env['LD_PRELOAD']=str(library)+(':'+mapper_env['LD_PRELOAD'] if mapper_env.get('LD_PRELOAD') else '')
            mapper_env['SE2_LIDAR_TSDF_WEIGHTING']=args.lidar_weighting
        if args.self_filter:
            if not args.fast_lidar:raise ValueError('--self-filter requires --fast-lidar')
            mapper_env['SE2_LIDAR_SELF_FILTER']='1'
        if args.deskew:
            if args.mode=='depth':raise ValueError('--deskew requires LiDAR input')
            spawn('deskew',[str(here/'deskew/build/terrain_lidar_deskew'),'--ros-args','-p','use_sim_time:=true',
                           '-p','restore_scan_time:='+str(args.restore_scan_time).lower()])
        lidar_topic='/terrain_variants/lidar_deskewed' if args.deskew else '/LIDAR/POINTS_NX'
        spawn('mapper',['/opt/ros/humble/lib/nvblox_ros/nvblox_node','--ros-args','--params-file',str(mapper_path),
            '-r','camera_0/depth/image:=/camera/d435i/depth/image_rect_raw',
            '-r','camera_0/depth/camera_info:=/camera/d435i/depth/camera_info',
            '-r','pointcloud:='+lidar_topic],env=mapper_env)
        spawn('terrain',[str(root.parent/'se2-terrain-venv/bin/python'),str(here/'checker_node.py'),
            '--ros-args','--params-file',str(checker_path)])
        if args.capture_mesh:
            spawn('capture',[str(root.parent/'se2-terrain-venv/bin/python'),str(here/'capture_mesh.py'),str(out/'mesh_snapshots')])
        if args.rviz:
            spawn('height',['/usr/bin/python3',str(root/'scripts/mesh_height.py'),'--ros-args','-p','use_sim_time:=true'])
            spawn('rviz',['rviz2','-d',str(terrain/'paper.rviz'),'--ros-args','-p','use_sim_time:=true'])
        topics=['/tf','/tf_static','/nvblox_lio/odom','/nvblox_lio/odom_lidar']
        if args.mode!='depth':topics+=['/LIDAR/POINTS_NX']
        if args.mode!='lidar':topics+=['/camera/d435i/depth/image_rect_raw','/camera/d435i/depth/camera_info']
        spawn('player',['ros2','bag','play',str(bag/'data'),'--clock','50','--rate',str(args.rate),
            '--start-paused','--disable-keyboard-controls','--read-ahead-queue-size',str(args.read_ahead),'--start-offset',str(args.start_offset),
            '--qos-profile-overrides-path',str(root/'bag_tools/runtime/play_qos.yaml'),'--topics',*topics,'--remap','/tf:=/bag_reference/tf'])
        resume=node.create_client(Resume,'/rosbag2_player/resume')
        deadline=time.monotonic()+40
        required={'/nvblox_node/mesh':'se2_navmesh_input'}
        if model:required['/bag_rebuild/joint_states_timed']='se2_bag_measured_model'
        if args.mode!='depth':required[lidar_topic]='nvblox_node'
        if args.deskew:required['/LIDAR/POINTS_NX']='terrain_lidar_deskew'
        if args.mode!='lidar':required['/camera/d435i/depth/image_rect_raw']='nvblox_node'
        while running and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            if (out/'timed_transforms.ready').exists() and resume.service_is_ready() and all(any(s.node_name==name for s in node.get_subscriptions_info_by_topic(t)) for t,name in required.items()):break
        else:raise RuntimeError('startup subscribers timeout')
        future=resume.call_async(Resume.Request());rclpy.spin_until_future_complete(node,future,timeout_sec=5)
        if not future.done() or future.exception():raise RuntimeError('resume failed')
        playing=time.monotonic();last_report=playing;last_resource=0
        while running:
            rclpy.spin_once(node,timeout_sec=.02);now=time.monotonic()
            if now-last_resource>=1:
                last_resource=now
                mem={k:int(v.split()[0]) for k,v in (line.split(':',1) for line in Path('/proc/meminfo').read_text().splitlines())}
                rss={}
                for name,process in children.items():
                    try:
                        status_text=Path(f'/proc/{process.pid}/status').read_text()
                        rss[name]=int(re.search(r'^VmRSS:\s+(\d+)',status_text,re.M)[1])
                    except (OSError,TypeError):pass
                resource=dict(available_kib=mem['MemAvailable'],swap_used_kib=mem['SwapTotal']-mem['SwapFree'],rss_kib=rss)
                state['resources'].append(resource);emit('resource',resource)
                if mem['MemAvailable']<700*1024:raise RuntimeError('Available memory below 700 MiB; stopped before system exhaustion')
            for name,process in children.items():
                if name!='player' and process.poll() is not None:raise RuntimeError(f'{name} exited: {process.returncode}')
            if state['status']:
                s=state['status'][-1]
                if s.get('compute_error') or s.get('input_error'):raise RuntimeError(str(s))
            if now-last_report>15:
                last_report=now;file.flush()
                print(json.dumps(dict(elapsed=round(now-playing,1),bag_elapsed=round((state['clock'] or bag_start)-bag_start,1),
                    mesh_geometry=len(state['mesh']),replacements=len(state['replacements']),
                    pending=state['status'][-1].get('pending_slabs') if state['status'] else None)),flush=True)
            if children['player'].poll() is not None:
                if children['player'].returncode:raise RuntimeError('player exited with error')
                result='PASS';break
            if args.duration and now-playing>=args.duration:result='PASS';break
            if now-playing>metadata['duration']['nanoseconds']/1e9/args.rate+180:raise RuntimeError('playback timeout')
    except Exception as exc:
        error=repr(exc);print('ERROR '+error,flush=True)
    finally:
        end=time.monotonic()
        for name in ('player','rviz','height','capture','terrain','mapper','deskew','robot_model','timed_tf'):
            process=children.get(name)
            if process is not None and process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=15)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        if trace is not None:trace.close()
        for log in logs:log.close()
        file.close()
        elapsed=max(.001,end-(playing or end));warm=(playing or end)+5
        mesh_stamps={stamp for t,stamp in state['mesh'] if t>=warm}
        rows=[d for d in state['replacements'] if d['t']>=warm and d['nonempty']]
        denominator=max(.001,elapsed-5)
        summary=dict(result=result,error=error,duration_wall_s=elapsed,bag_elapsed_s=(state['clock'] or bag_start)-bag_start,
            distinct_geometry_hz=len(mesh_stamps)/denominator,nonempty_replacement_hz=len(rows)/denominator,
            distinct_result_sequence_hz=len({d['sequence'] for d in rows})/denominator,
            complete_input_result_hz=len({d['sequence'] for d in rows if d['complete']})/denominator,
            latency_oldest_ms=percentiles([d['oldest'] for d in rows]),latency_newest_ms=percentiles([d['newest'] for d in rows]),
            callback_ms=percentiles([d['callback'] for d in rows]),
            pending_slabs=percentiles([d.get('pending_slabs') for d in state['status']]),
            last_status=state['status'][-1] if state['status'] else None,
            pose_z_range=[min(d['position'][2] for d in state['odom']),max(d['position'][2] for d in state['odom'])] if state['odom'] else None)
        summary['resources']=dict(samples=len(state['resources']),
            available_mib=percentiles([d['available_kib']/1024 for d in state['resources']]),
            minimum_available_mib=min((d['available_kib']/1024 for d in state['resources']),default=None),
            peak_rss_mib={name:max((d['rss_kib'].get(name,0)/1024 for d in state['resources']),default=0) for name in children},
            peak_swap_used_mib=max((d['swap_used_kib']/1024 for d in state['resources']),default=0))
        summary.update(run_completed=result=='PASS',performance_target_met=(
            summary['distinct_geometry_hz']>10 and summary['distinct_result_sequence_hz']>10 and
            summary['latency_oldest_ms'].get('p95',float('inf'))<50),
            connectivity_verified=False,stair_connectivity_verified=False)
        mapper_log=(out/'mapper.log').read_text(errors='replace') if (out/'mapper.log').exists() else ''
        summary['effective_tsdf_weighting']=effective_weights(mapper_log)
        summary['weighting_parameter_scope']='ROS weighting parameter: camera TSDF and color only'
        if args.weighting:
            wanted={'constant':'kConstantWeight','constant_dropoff':'kConstantDropoffWeight',
                'inverse_square':'kInverseSquareWeight','inverse_square_dropoff':'kInverseSquareDropoffWeight',
                'inverse_square_tsdf_distance_penalty':'kInverseSquareTsdfDistancePenalty',
                'linear_with_max':'kLinearWithMax'}[args.weighting]
            verified=summary['effective_tsdf_weighting'].get('camera')==wanted
            summary['camera_weighting_verified']=verified
            if not verified:
                result='FAIL';error='Camera weighting was not verified in mapper runtime'
                summary.update(result=result,error=error,run_completed=False,performance_target_met=False)
        if args.lidar_weighting:
            summary['weighting_parameter_scope']+='; independent experimental LiDAR adapter enabled'
            wanted={'inverse_square':'kInverseSquareWeight',
                    'inverse_square_tsdf_distance_penalty':'kInverseSquareTsdfDistancePenalty'}[args.lidar_weighting]
            verified=summary['effective_tsdf_weighting'].get('lidar')==wanted and 'SE2_EFFECTIVE_LIDAR_WEIGHTING=' in mapper_log
            summary['lidar_weighting_verified']=verified
            if not verified:
                result='FAIL';error='LiDAR weighting was not verified in mapper runtime'
                summary.update(result=result,error=error,run_completed=False,performance_target_met=False)
        ingestion={}
        for timer in ('ros/pointcloud_callback','ros/lidar','ros/lidar/integration',
                      'ros/lidar/front_integrated','ros/lidar/rear_integrated',
                      'ros/depth_image_callback','ros/depth','ros/depth/integrate'):
            matches=re.findall(r'^'+re.escape(timer)+r'\s+(\d+)\s+[\d.]+\s+\(',mapper_log,re.MULTILINE)
            if matches:ingestion[timer]=int(matches[-1])
        ingestion['raw_lidar_messages_in_full_bag']=next((t['message_count'] for t in metadata['topics_with_message_count']
            if t['topic_metadata']['name']=='/LIDAR/POINTS_NX'),0)
        ingestion['raw_depth_messages_in_full_bag']=next((t['message_count'] for t in metadata['topics_with_message_count']
            if t['topic_metadata']['name']=='/camera/d435i/depth/image_rect_raw'),0)
        if args.deskew and (out/'deskew.log').exists():
            final=re.findall(r'DESKEW_FINAL (.*)',(out/'deskew.log').read_text(errors='replace'))
            if final:ingestion['deskew_final']={k:float(v) for k,v in re.findall(r'(\w+)=([\d.]+)',final[-1])}
        summary['sensor_ingestion']=ingestion
        summary['latency_scope']=next((s['metrics']['output_scope'] for s in reversed(state['status'])
            if s.get('metrics',{}).get('output_scope')),'all affected slabs' if state['status'] else None)
        (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
        manifest.update(state='finished',result=result,error=error,
                        effective_tsdf_weighting=summary['effective_tsdf_weighting'])
        (out/'experiment.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
        (here/'latest_experiment.json').write_text(json.dumps(dict(path=str(out),summary=summary),ensure_ascii=False,indent=2))
        print(json.dumps(summary,ensure_ascii=False),flush=True)
        node.destroy_node();rclpy.shutdown()
    return 0 if result=='PASS' else 1


if __name__=='__main__':sys.exit(main())
