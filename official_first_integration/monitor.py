#!/usr/bin/env python3
import json,signal,sys,time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String,Float64MultiArray
from nav_msgs.msg import Odometry
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True)
rclpy.init();node=rclpy.create_node('first_monitor');report={};periods=[];compute=[]
def status(m):
    d=json.loads(m.data);report.update(d)
    with (out/'planner.jsonl').open('a') as f:f.write(json.dumps(dict(time_ns=time.time_ns(),**d))+'\n')
def timing(m):
    if len(m.data)<5:return
    periods.append(m.data[1]);compute.append(m.data[2]);report.update(control_samples=len(periods),control_period_max_ms=max(periods),control_mean_hz=1000/(sum(periods)/len(periods)),planning_max_ms=max(compute))
def odom(m):
    p=m.pose.pose.position;q=m.pose.pose.orientation;report['body']=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
node.create_subscription(String,'/first/status',status,10)
node.create_subscription(Float64MultiArray,'/first/control_timing',timing,100)
node.create_subscription(Odometry,'/first/aligned_odom',odom,qos_profile_sensor_data)
def save():
    tmp=out/'summary.tmp';tmp.write_text(json.dumps(report,indent=2));tmp.replace(out/'summary.json')
node.create_timer(1.,save)
try:rclpy.spin(node)
except KeyboardInterrupt:pass
finally:save();node.destroy_node();rclpy.try_shutdown()
