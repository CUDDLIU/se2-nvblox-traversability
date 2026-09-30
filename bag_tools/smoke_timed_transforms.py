"""Explicit isolated replay-TF integration check, no hardware/control outputs."""
import json
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformListener
import yaml

ROOT=Path(__file__).resolve().parent.parent
OUT=ROOT/'bag_tools/validation/20260929_recording_tf'
env=dict(os.environ,ROS_DOMAIN_ID='79',ROS_LOCALHOST_ONLY='1',
         FASTRTPS_DEFAULT_PROFILES_FILE=str(ROOT/'terrain_variants/fastdds_large_shm.xml'))
env.pop('FASTDDS_DEFAULT_PROFILES_FILE',None)
os.environ.update(env)
processes=[]


def start(name,args):
    log=(OUT/(name+'.log')).open('w')
    process=subprocess.Popen(args,env=env,stdout=log,stderr=log,start_new_session=True)
    processes.append((process,log));return process


def main():
    start('test_timed_tf',['/usr/bin/python3',str(ROOT/'bag_tools/replay_transforms.py'),'run',str(OUT)])
    start('test_model',['/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher','--ros-args',
        '-r','__node:=se2_bag_measured_model','--params-file',str(OUT/'recorded_robot.yaml'),
        '-r','joint_states:=/bag_rebuild/joint_states_timed','-r','robot_description:=/test/description'])
    rclpy.init();node=rclpy.create_node('timed_tf_check')
    buffer=Buffer(cache_time=Duration(seconds=20));listener=TransformListener(buffer,node)
    publisher=node.create_publisher(Clock,'/clock',QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
    deadline=time.monotonic()+4
    while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
    description=yaml.safe_load((OUT/'recorded_robot.yaml').read_text())['/**']['ros__parameters']['robot_description']
    links=[j.find('child').get('link') for j in ET.fromstring(description).findall('joint') if j.get('type')!='fixed']
    meta=yaml.safe_load((ROOT/'bags/20260929_114239_0109c6/data/metadata.yaml').read_text())['rosbag2_bagfile_information']
    begin=meta['starting_time']['nanoseconds_since_epoch'];results=[]
    for elapsed in [2.,100.,143.,219.,240.,450.,460.8]:
        now=begin+int(elapsed*1e9);publisher.publish(Clock(clock=Time(nanoseconds=now).to_msg()))
        until=time.monotonic()+1.5
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
        missing=[];ages=[]
        for link in links:
            try:
                tf=buffer.lookup_transform('nvblox_odom',link,Time())
                stamp=tf.header.stamp.sec*10**9+tf.header.stamp.nanosec
                ages.append((now-stamp)*1e-9)
            except Exception as exc:missing.append(dict(link=link,error=str(exc)))
        results.append(dict(elapsed_s=elapsed,links=len(links),missing=missing,max_measurement_age_s=max(ages,default=None)))
    result=dict(result='PASS' if all(not r['missing'] for r in results) else 'FAIL',checks=results,
                note='Missing source intervals are preserved; latest measured joint pose is not retimestamped.')
    (OUT/'model_tf_result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
    node.destroy_node();rclpy.shutdown()
    if result['result']!='PASS':raise RuntimeError('Model TF lookups failed')


if __name__=='__main__':
    try:main()
    finally:
        for process,log in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=8)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
            log.close()
