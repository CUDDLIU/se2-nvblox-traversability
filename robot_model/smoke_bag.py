"""Explicit isolated telemetry -> bag -> replay test. No motion commands."""
import json
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
TOPICS=['/nvblox_robot/motion_status/'+s for s in ('raw','joints','wheels')]+[
    '/nvblox_robot/joint_states','/nvblox_robot/robot_description','/tf','/tf_static']


def stop(proc):
    if proc.poll() is None:
        os.killpg(proc.pid,signal.SIGINT)
        try:proc.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)


def play(out):
    import rclpy
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from tf2_msgs.msg import TFMessage
    from std_msgs.msg import String
    rclpy.init();node=rclpy.create_node('nvblox_model_bag_smoke')
    tf=[];descriptions=[];raw=[]
    node.create_subscription(TFMessage,'/tf',lambda m:tf.extend(m.transforms),100)
    node.create_subscription(String,TOPICS[0],lambda m:raw.append(m.data),100)
    node.create_subscription(String,TOPICS[4],lambda m:descriptions.append(m.data),
        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    with (out/'play.log').open('w') as log:
        proc=subprocess.Popen(['ros2','bag','play',str(out/'data'),'--clock','--rate','2',
             '--qos-profile-overrides-path',str(out/'qos.yaml')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            deadline=time.monotonic()+22
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.1)
                if proc.poll() is not None:break
            for _ in range(20):rclpy.spin_once(node,timeout_sec=.05)
            assert proc.poll()==0
            assert len(raw)>30 and len(tf)>400 and descriptions,(len(raw),len(tf),len(descriptions))
            assert all(t.child_frame_id.startswith('nvblox_robot/') for t in tf)
            names=node.get_node_names();assert not any('bridge' in n or 'recorder' in n for n in names),names
            result=dict(result='PASS',replayed_raw=len(raw),replayed_tf=len(tf),description_received=True,
                        no_robot_receiver_in_replay=True,nodes=names)
            (out/'replay_result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
        finally:stop(proc);node.destroy_node();rclpy.shutdown()


def record():
    out=ROOT/'robot_model/runtime'/('bag_smoke_'+time.strftime('%Y%m%d_%H%M%S'))
    out.mkdir(parents=True)
    (out/'qos.yaml').write_text('/nvblox_robot/robot_description:\n  reliability: reliable\n  durability: transient_local\n  history: keep_last\n  depth: 1\n/tf_static:\n  reliability: reliable\n  durability: transient_local\n  history: keep_last\n  depth: 100\n')
    env=dict(os.environ,ROS_DOMAIN_ID='75',ROS_LOCALHOST_ONLY='1')
    env.pop('FASTRTPS_DEFAULT_PROFILES_FILE',None);env.pop('FASTDDS_DEFAULT_PROFILES_FILE',None)
    procs=[]
    with (out/'record_test.log').open('w') as log:
        def start(args):
            p=subprocess.Popen(args,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            procs.append(p);return p
        try:
            bag=start(['ros2','bag','record','-o',str(out/'data'),'--qos-profile-overrides-path',str(out/'qos.yaml'),*TOPICS])
            capture=start(['/usr/bin/python3',str(ROOT/'bag_tools/motion_capture.py'),str(out/'telemetry')])
            time.sleep(2)
            model=start(['ros2','launch',str(ROOT/'robot_model/nvblox_robot.launch.py')])
            time.sleep(13)
            assert all(p.poll() is None for p in procs),(out/'record_test.log').read_text()
        finally:
            for p in reversed(procs):stop(p)
    con=sqlite3.connect(str(next((out/'data').glob('*.db3'))))
    counts=dict(con.execute('select topics.name,count(messages.id) from topics join messages on topics.id=messages.topic_id group by topics.name'))
    assert set(counts)==set(TOPICS),counts
    assert all(n>40 for t,n in counts.items() if t not in ('/tf_static',TOPICS[4])),counts
    health=json.loads((out/'telemetry/health.json').read_text());assert health['frames']>40 and not health['running']
    result=dict(result='PASS',counts=counts,sidecar_frames=health['frames'])
    (out/'record_result.json').write_text(json.dumps(result,indent=2));print(out);print(json.dumps(result),flush=True)
    subprocess.run(['/usr/bin/python3',__file__,'--play',str(out)],env=dict(env,ROS_DOMAIN_ID='76'),check=True,timeout=35)


if __name__=='__main__':
    if len(sys.argv)>1 and sys.argv[1]=='--play':play(Path(sys.argv[2]))
    else:record()
