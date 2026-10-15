#!/usr/bin/env python3
"""Independent FIRST session; default observes/plans without sending velocity."""
import argparse, hashlib, json, os, signal, subprocess, sys, time
import uuid
from datetime import datetime
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/'scanplanner_integration'))

def session_transport_profile(source,destination):
    """Preserve deployed network settings, size local SHM for full NX scans."""
    import xml.etree.ElementTree as ET
    tree=ET.parse(source);root=tree.getroot()
    ns='http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles';ET.register_namespace('',ns)
    for descriptor in root.findall('.//{'+ns+'}transport_descriptor'):
        if descriptor.findtext('{'+ns+'}type')!='SHM':continue
        element=descriptor.find('{'+ns+'}segment_size')
        if element is None:element=ET.SubElement(descriptor,'{'+ns+'}segment_size')
        element.text=str(max(int(element.text or 0),32*1024*1024))
    tree.write(destination,encoding='utf-8',xml_declaration=True)
    return str(destination)

def session_remaps(session):
    # A retained route from another planner/session must never start this robot.
    return {topic:f'/first/session_{session}/{name}' for topic,name in (
        ('/se2_global/path','route'),('/first/cmd_vel_stamped','command'),
        ('/se2_global/navigation_route','navigation_route'),
        ('/se2_global/cancel','global_cancel'),
        ('/first/request_gait','gait'),('/first/saved_goal','saved_goal'),('/first/cancel','cancel'),('/se2_global/goal_pose','goal_pose'),('/first/robot_state','robot_state'),
        ('/nvblox_robot/motion_status/raw','motion_status'))}

def ros_args(remaps):
    return ['--ros-args',*[arg for src,dst in remaps.items() for arg in ('-r',src+':='+dst)]]

def robot_command(execute,enter_navigation,output,remaps,official=False,source=False):
    return [sys.executable,str(HERE/'robot.py'),
            '--execute' if execute else '--telemetry-only','--output',str(output),
            *(['--source-planner'] if source else []),*(['--enter-navigation'] if enter_navigation else []),*(['--official-limits'] if official else []),*ros_args(remaps)]

def stop_children(children):
    # Stop the sender first, while it can still transmit zeros and restore an
    # owned usage mode. A slow RViz must not delay stopping the robot.
    errors=[]
    names=(['robot'] if 'robot' in children else [])+[k for k in reversed(children) if k!='robot']
    for name in names:
        proc=children[name]
        for sig,timeout in ((signal.SIGINT,2.),(signal.SIGTERM,1.),(signal.SIGKILL,1.)):
            if proc.poll() is not None:break
            try:
                os.killpg(proc.pid,sig);proc.wait(timeout=timeout)
            except ProcessLookupError:break
            except subprocess.TimeoutExpired:continue
            except OSError as exc:
                errors.append(f'{name}: {exc}');break
        if proc.poll() is None:errors.append(f'{name} did not exit')
    return errors

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--local-backend',choices=('source','official','legacy'),default='source',help='Reconstructed source (default); official selects the reference binary backend; legacy selects the earlier custom planner')
    p.add_argument('--map',type=Path,default=ROOT/'scanplanner_integration/maps/20261012_confirmed_scene')
    p.add_argument('--execute',action='store_true');p.add_argument('--enter-navigation',action='store_true')
    p.add_argument('--initial-map-pose',type=float,nargs=4)
    p.add_argument('--recording-start',action='store_true',help='Restrict initialization to the recording start; default searches the whole recorded map')
    p.add_argument('--trace-near-body',action='store_true',help='Trace all near-body return outcomes; default records newly occupied near-body cells only')
    p.add_argument('--no-rviz',action='store_true');p.add_argument('--duration',type=float,default=0)
    args=p.parse_args()
    if args.enter_navigation and not args.execute:p.error('--enter-navigation requires --execute')
    if args.recording_start and args.initial_map_pose:p.error('choose --recording-start or --initial-map-pose')
    if not (args.map/'navmesh.json').is_file():p.error('Fixed map missing: '+str(args.map))
    binary=HERE/'build/first_node'
    if args.local_backend=='source':
        from source.core import verify_assets
        official_assets=verify_assets()
        if not (HERE/'build/source/libnos_source.so').is_file():p.error('Run official_first_integration/source/build.sh first')
    elif args.local_backend=='official':
        from native.runtime import verify_assets
        official_assets=verify_assets()
    elif not binary.is_file():p.error('Run official_first_integration/build.sh first')
    from export_terrain import export
    doc=json.loads((args.map/'navmesh.json').read_text());mesh=args.map/doc['source_mesh'];mesh_hash=hashlib.sha256(mesh.read_bytes()).hexdigest()
    terrain=HERE/'maps'/f'{mesh_hash[:16]}.terrain'
    if not terrain.is_file() or not Path(str(terrain)+'.triangles').is_file():export(args.map,terrain)
    os.environ.update(ROS_DOMAIN_ID='0',ROS_LOCALHOST_ONLY='0',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',FASTRTPS_DEFAULT_PROFILES_FILE=str(Path.home()/'.config/fastdds/eno1.xml'),RMW_FASTRTPS_PUBLICATION_MODE='ASYNCHRONOUS')
    session=datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8]
    out=HERE/'runtime'/session;out.mkdir(parents=True)
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']=session_transport_profile(
        Path.home()/'.config/fastdds/eno1.xml',out/'fastdds.xml')
    pose_cache=HERE/'runtime/last_map_pose.json'
    seed_source='explicit' if args.initial_map_pose else 'recording_start' if args.recording_start else 'whole_map'
    # The last session's pose is diagnostic only; default initialization must
    # independently find the robot in the recorded map after manual relocation.
    remaps=session_remaps(session)
    children={};logs=[];running=True
    def stop(*_):
        nonlocal running
        running=False
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    def spawn(name,cmd):
        f=(out/(name+'.log')).open('w');logs.append(f)
        children[name]=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
        return children[name]
    report=dict(state='starting',execute=args.execute,launcher_pid=os.getpid(),map=str(args.map),mesh_sha256=mesh_hash,terrain_sha256=hashlib.sha256(terrain.read_bytes()).hexdigest(),remaps=remaps,children={})
    report.update(local_backend=args.local_backend,official_assets=official_assets if args.local_backend in ('official','source') else None)
    report.update(localization_seed_source=seed_source,initial_map_pose=args.initial_map_pose)
    def save():
        report['children']={k:v.pid for k,v in children.items()};(out/'session.json').write_text(json.dumps(report,indent=2))
    print('FIRST SESSION '+str(out),flush=True)
    try:
        from live_inputs import ensure_live_inputs
        report['inputs']=ensure_live_inputs(ROOT,out,spawn,lambda:running,require_registered=args.local_backend=="source")
        if not running:return
        spawn('robot',robot_command(args.execute,args.enter_navigation,out/'telemetry',remaps,official=args.local_backend=='official',source=args.local_backend=='source'))
        spawn('localization',[sys.executable,str(HERE/'localization.py'),str(args.map),'--output',str(out/'localization'),'--pose-cache',str(pose_cache),*(['--initial-map-pose',*map(str,args.initial_map_pose)] if args.initial_map_pose else ['--recording-start'] if args.recording_start else [])])
        from robot_visual import description
        model_params=out/'robot_model.params.json'
        model_params.write_text(json.dumps({'first_robot_state_publisher':{'ros__parameters':{'robot_description':description(),'publish_frequency':30.}}}))
        spawn('robot_description',['ros2','run','robot_state_publisher','robot_state_publisher','--ros-args','-r','__node:=first_robot_state_publisher','--params-file',str(model_params),'-r','robot_description:=/first/robot_description','-r','joint_states:=/first/joint_states'])
        spawn('robot_visual',[sys.executable,str(HERE/'robot_visual.py'),*ros_args(remaps)])
        spawn('monitor',[sys.executable,str(HERE/'monitor.py'),str(out/'metrics')])
        if args.local_backend=='source':
            spawn('first',[sys.executable,str(HERE/'source/adapter.py'),'--output',str(out/'source'),*ros_args(remaps)])
        elif args.local_backend=='official':
            spawn('first',[sys.executable,str(HERE/'native/adapter.py'),'--output',str(out/'native'),*ros_args(remaps)])
        else:
            spawn('first',[str(binary),*ros_args(remaps),'-p','library:='+str(HERE/'vendor/paths/first'),'-p','terrain:='+str(terrain),'-p','hit_trace:='+str(out/'near_body_hits.jsonl'),'-p','hit_trace_new_only:='+('false' if args.trace_near_body else 'true')])
        spawn('global',[sys.executable,str(ROOT/'global_planner/ros_node.py'),str(args.map),'--robot-start','--runtime',str(out/'global'),'--frame','se2_scan_map',*ros_args({**remaps,'/nvblox_lio/odom':'/first/aligned_odom'})])
        if not args.no_rviz:spawn('rviz',['rviz2','-d',str(HERE/'first.rviz'),*ros_args(remaps)])
        report['state']='execution_waiting_goal' if args.execute else 'shadow';save();began=time.monotonic()
        while running and (not args.duration or time.monotonic()-began<args.duration):
            for name,proc in children.items():
                if proc.poll() is not None:raise RuntimeError(f'{name} exited {proc.returncode}; see {out/name}.log')
            time.sleep(.1)
        report['state']='complete'
    except Exception as exc:
        report.update(state='failed',error=str(exc));raise
    finally:
        report['shutdown_errors']=stop_children(children)
        save()
        for f in logs:f.close()
if __name__=='__main__':main()
