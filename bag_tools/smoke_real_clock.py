"""Isolated real rosbag2 clock + recorded model + RViz tail-replay acceptance."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET
import yaml
import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

ROOT=Path(__file__).resolve().parent.parent
OUT=ROOT/'bag_tools/validation/20260929_recording_tf'
env=dict(os.environ,ROS_DOMAIN_ID='79',ROS_LOCALHOST_ONLY='1',DISPLAY=':0',
         XAUTHORITY='/run/user/1000/gdm/Xauthority',
         FASTRTPS_DEFAULT_PROFILES_FILE=str(ROOT/'terrain_variants/fastdds_large_shm.xml'))
env.pop('FASTDDS_DEFAULT_PROFILES_FILE',None);os.environ.update(env)
processes=[]


def start(name,args):
    log=(OUT/(name+'.log')).open('w')
    p=subprocess.Popen(args,env=env,stdout=log,stderr=log,start_new_session=True)
    processes.append((p,log));return p


def main():
    config=yaml.safe_load((ROOT.parent/'se2_terrain_check/paper.rviz').read_text())
    vm=config['Visualization Manager']
    vm['Displays']=[dict(Class='rviz_default_plugins/Grid',Name='Grid',Enabled=True),
        dict(Class='rviz_default_plugins/RobotModel',Name='nvblox · 实测关节模型',Enabled=True,
        **{'Description Source':'Topic','Description Topic':{'Value':'/bag_rebuild/robot_description',
            'Depth':1,'Reliability Policy':'Reliable','Durability Policy':'Transient Local'},
           'Visual Enabled':True,'Collision Enabled':False})]
    vm['Global Options']['Fixed Frame']='nvblox_odom'
    vm['Views']['Current'].update({'Distance':3.,'Focal Point':{'X':-1.8,'Y':-3.4,'Z':0.}})
    (OUT/'model_acceptance.rviz').write_text(yaml.safe_dump(config,allow_unicode=True))
    start('real_timed_tf',['/usr/bin/python3',str(ROOT/'bag_tools/replay_transforms.py'),'run',str(OUT)])
    start('real_model',['/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher','--ros-args',
        '-r','__node:=se2_bag_measured_model','--params-file',str(OUT/'recorded_robot.yaml'),
        '-r','joint_states:=/bag_rebuild/joint_states_timed','-r','robot_description:=/bag_rebuild/robot_description'])
    start('real_rviz',['rviz2','-d',str(OUT/'model_acceptance.rviz'),'--ros-args','-p','use_sim_time:=true'])
    rclpy.init();node=rclpy.create_node('real_clock_check');buffer=Buffer();listener=TransformListener(buffer,node)
    until=time.monotonic()+4
    while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
    player=start('real_player',['ros2','bag','play',str(ROOT/'bags/20260929_114239_0109c6/data'),
        '--clock','30','--start-offset','440','--disable-keyboard-controls',
        '--topics','/nvblox_lio/odom','/tf_static'])
    description=yaml.safe_load((OUT/'recorded_robot.yaml').read_text())['/**']['ros__parameters']['robot_description']
    links=[j.find('child').get('link') for j in ET.fromstring(description).findall('joint') if j.get('type')!='fixed']
    start_time=time.monotonic();checks=[];shot=False
    while time.monotonic()-start_time<29:
        rclpy.spin_once(node,timeout_sec=.03)
        elapsed=time.monotonic()-start_time
        if elapsed>8:
            missing=[link for link in links if not buffer.can_transform('nvblox_odom',link,Time())]
            checks.append(dict(elapsed=elapsed,missing=missing))
            if not shot:
                subprocess.run(['gnome-screenshot','-f',str(OUT/'real_clock_rviz.png')],timeout=10);shot=True
    result=dict(result='PASS' if checks and all(not c['missing'] for c in checks) else 'FAIL',
                checked_frames=len(links),checks=len(checks),failed=[c for c in checks if c['missing']],
                player_returncode=player.poll(),source_bag='20260929_114239_0109c6',start_offset=440)
    (OUT/'real_clock_result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
    node.destroy_node();rclpy.shutdown()
    if result['result']!='PASS':raise RuntimeError('Real-clock TF lookup failed')


if __name__=='__main__':
    try:main()
    finally:
        for p,log in reversed(processes):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try:p.wait(timeout=8)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
            log.close()
