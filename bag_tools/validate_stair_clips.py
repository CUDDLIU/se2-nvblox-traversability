"""End-to-end production replay acceptance of independently extracted stair bags."""
import json,os,sys,time,subprocess,re
from pathlib import Path
import yaml
from bag_manager import BagManager

root=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(root/'scripts'))
from mesh_wire import mesh_summary
source=root/'bags/20260929_125643_0ae23c'
keys=[Path(p).name for p in json.loads((source/'diagnostics/stair_clips/clips.json').read_text())['clips']]
manager=BagManager();report={'clips':[]};node=None
try:
    os.environ.update(manager.env(True))
    import rclpy
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from nvblox_msgs.msg import Mesh
    from rosgraph_msgs.msg import Clock
    rclpy.init()
    state={'clock':None,'mesh':[]}
    qos=QoSProfile(depth=20,reliability=ReliabilityPolicy.BEST_EFFORT)
    def mesh(raw):
        d=mesh_summary(raw)
        if d['vertices']>0:state['mesh'].append((state['clock'],time.monotonic()))
    for key in keys:
        state.update(clock=None,mesh=[])
        path=manager.bags/key;meta=yaml.safe_load((path/'data/metadata.yaml').read_text())['rosbag2_bagfile_information']
        start=meta['starting_time']['nanoseconds_since_epoch']/1e9;duration=meta['duration']['nanoseconds']/1e9
        began=time.monotonic();manager.play(key,rebuild=True,sensor_mode='fusion',settings={'max_step':.2,'max_slope_deg':40.})
        node=rclpy.create_node('stair_clip_acceptance')
        node.create_subscription(Mesh,'/nvblox_node/mesh',mesh,qos,raw=True)
        node.create_subscription(Clock,'/clock',lambda m:state.update(clock=m.clock.sec+m.clock.nanosec/1e9),qos)
        run=dict(bag=key,session=str(manager.session),start_to_play_s=time.monotonic()-began)
        print('REPLAY '+json.dumps(run),flush=True);last=0
        while time.monotonic()-began<duration+90:
            rclpy.spin_once(node,timeout_sec=.05);status=manager.tick()
            if time.monotonic()-last>15:
                last=time.monotonic();print(json.dumps(dict(bag=key,elapsed=None if state['clock'] is None else round(state['clock']-start,1),geometry_messages=len(state['mesh']),mode=status['mode'])),flush=True)
            if status['mode']=='idle':raise RuntimeError(status['message'])
            if status['mode']=='finished':break
        else:raise RuntimeError('Replay did not finish')
        assert manager.children['play'].returncode==0
        assert state['clock'] is not None and state['clock']-start>=duration-1
        assert len(state['mesh'])>100
        stamps=[c for c,_ in state['mesh'] if c is not None]
        assert max(stamps)-start>duration-2
        assert all(manager.children[n].poll() is None for n in ['nvblox','terrain','height','rviz','deskew','timed_tf','replay_model'])
        subprocess.run(['gnome-screenshot','-f',str(manager.session/'clip_final_rviz.png')],timeout=15,check=True)
        run.update(clock_elapsed_s=state['clock']-start,duration_s=duration,geometry_messages=len(stamps),
                   max_mesh_gap_s=max((b-a for a,b in zip(stamps,stamps[1:])),default=None))
        session=manager.session;node.destroy_node();node=None;manager.stop_play()
        log=(session/'deskew.log').read_text(errors='replace')
        final=re.findall(r'DESKEW_FINAL (.*)',log)
        run['deskew_final']={k:float(v) for k,v in re.findall(r'(\w+)=([\d.]+)',final[-1])} if final else {}
        wanted=next(t['message_count'] for t in meta['topics_with_message_count'] if t['topic_metadata']['name']=='/LIDAR/POINTS_NX')
        run['expected_lidar']=wanted
        assert run['deskew_final'].get('received')==wanted
        assert run['deskew_final'].get('frames')==wanted
        assert run['deskew_final'].get('dropped')==0
        run['result']='PASS';(path/'clip_replay_validation.json').write_text(json.dumps(run,indent=2));report['clips'].append(run)
    report['result']='PASS'
finally:
    manager.stop_play()
    try:
        manager.restore_live();report['live_restored']=True
    finally:
        manager.close()
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        (source/'diagnostics/stair_clips/replay_validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
print(json.dumps(report),flush=True)
