#!/usr/bin/env python3
"""Read-only capture of initial rigid-map candidates; publishes no messages."""
import argparse,json,sys,time
from collections import deque
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from alignment import MapAligner,deskew
from localization_node import Localization
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from rclpy.qos import qos_profile_sensor_data
ap=argparse.ArgumentParser();ap.add_argument('--map',required=True);ap.add_argument('--seed',type=float,nargs=4,required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
rclpy.init();n=rclpy.create_node('read_only_alignment_candidates');poses=deque(maxlen=1000);clouds=deque(maxlen=5)
def odom(m):
 p=m.pose.pose.position;q=m.pose.pose.orientation
 poses.append([m.header.stamp.sec+m.header.stamp.nanosec/1e9,p.x,p.y,p.z,q.x,q.y,q.z,q.w])
n.create_subscription(Odometry,'/nvblox_lio/odom',odom,qos_profile_sensor_data)
n.create_subscription(PointCloud2,'/LIDAR/POINTS_NX',clouds.append,qos_profile_sensor_data)
end=time.monotonic()+6
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
n.destroy_node();rclpy.shutdown()
assert len(clouds)==5 and len(poses)>100,'missing sensor input'
poses=np.array(poses);source=deskew(np.concatenate([Localization.unpack(None,m) for m in clouds]),poses);body=poses[-1,1:4]
a.output.mkdir(parents=True,exist_ok=True);np.savez_compressed(a.output/'alignment_input.npz',source=source,body=body,poses=poses)
aligner=MapAligner(a.map);aligner.anchor=np.array(a.seed[:3]);aligner.seed_yaw=a.seed[3]
quality=aligner.quality;rows=[]
def observe(points,body,t,r):
 q=quality(points,body,t,r);rows.append(dict(position=(body@r.T+t).tolist(),rpy_deg=Rotation.from_matrix(r).as_euler('xyz',degrees=True).tolist(),quality=q));return q
aligner.quality=observe
try:aligner.fit_rigid(source,body);reason='match accepted'
except ValueError as e:reason=str(e)
rows.sort(key=lambda r:r['quality']['clipped_mean_m']);report=dict(reason=reason,seed=a.seed,candidates=rows)
(a.output/'candidates.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
