#!/usr/bin/env python3
"""Read-only FIRST snapshot for reproducing collision rejection offline."""
import argparse,json,math,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from nav_msgs.msg import Odometry,Path as RosPath
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points

def main():
    ap=argparse.ArgumentParser();ap.add_argument('session',type=Path);args=ap.parse_args()
    doc=json.loads((args.session/'session.json').read_text());data={};clouds={};at={}
    rclpy.init();n=rclpy.create_node('first_scene_capture')
    def pose(p):
        v,q=p.position,p.orientation
        return [v.x,v.y,v.z,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
    def cb(key,m):
        at[key]=time.monotonic()
        if isinstance(m,PointCloud2):
            fields=tuple(f for f in ('x','y','z','timestamp','ring','intensity') if any(a.name==f for a in m.fields)) if key=='raw' else ('x','y','z')
            p=read_points(m,field_names=fields,skip_nans=True);clouds[key]=np.column_stack([p[f] for f in fields])
            data[key+'_frame']=m.header.frame_id
            data[key+'_stamp']=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        elif isinstance(m,Odometry):data[key]=pose(m.pose.pose)
        elif isinstance(m,RosPath):data[key]=[pose(p.pose) for p in m.poses]
        else:data[key]=json.loads(m.data)
    retained=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for key,topic,kind,qos in (
        ('occupied','/first/occupancy',PointCloud2,qos_profile_sensor_data),
        ('raw','/LIDAR/POINTS_NX',PointCloud2,qos_profile_sensor_data),
        ('body','/first/aligned_odom',Odometry,qos_profile_sensor_data),
        ('path',doc['remaps']['/se2_global/path'],RosPath,retained),
        ('robot','/first/m20_status',String,10),
        ('motion',doc['remaps']['/nvblox_robot/motion_status/raw'],String,10)):
        n.create_subscription(kind,topic,lambda m,k=key:cb(k,m),qos)
    end=time.monotonic()+45;ready=None
    try:
        while time.monotonic()<end:
            rclpy.spin_once(n,timeout_sec=.01);now=time.monotonic()
            if all(now-at.get(k,-1e9)<1.2 for k in ('occupied','raw','body','robot','motion')):
                if ready is None:ready=now
                if now-ready>2:break
            else:ready=None
        assert ready is not None,'incomplete snapshot: '+str(list(data))
        (args.session/'snapshot.json').write_text(json.dumps(data))
        np.savez_compressed(args.session/'snapshot.npz',**clouds)
        print(json.dumps(dict(body=data['body'],clouds={k:v.shape for k,v in clouds.items()})))
    finally:n.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
