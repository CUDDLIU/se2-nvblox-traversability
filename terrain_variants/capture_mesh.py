"""Sparse full Mesh snapshots for offline stair geometry diagnosis (domain 74)."""
import argparse
import json
import os
from pathlib import Path
import sys
os.environ.update(ROS_DOMAIN_ID='74',ROS_LOCALHOST_ONLY='1')
# Inherit the runner's verified large-SHM profile; do not silently change transport.
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from nvblox_msgs.msg import Mesh
from rclpy.qos import qos_profile_sensor_data
sys.path.insert(0,str(Path(__file__).parent/'baseline'))
from mesh_wire import decode_mesh

def main():
    parser=argparse.ArgumentParser();parser.add_argument('output',type=Path);args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=rclpy.create_node('terrain_mesh_capture')
    blocks={};pose=None;start=None;next_snapshot=0
    times=[20,60,90,110,125,135,150,170,195,230,250]
    def odom(msg):
        nonlocal pose,start
        p=msg.pose.pose.position;q=msg.pose.pose.orientation
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        pose=dict(stamp=stamp,position=[p.x,p.y,p.z],rotation=[q.x,q.y,q.z,q.w])
        if start is None:start=stamp
    def mesh(raw):
        nonlocal next_snapshot
        msg=decode_mesh(raw)
        if msg.clear:blocks.clear()
        for key,b in zip(msg.block_indices,msg.blocks):
            key=(key.x,key.y,key.z)
            if len(b.triangles_array):blocks[key]=(b.vertices_array,b.triangles_array)
            else:blocks.pop(key,None)
        if pose is None or next_snapshot>=len(times) or pose['stamp']-start<times[next_snapshot]:return
        tag=f"mesh_{times[next_snapshot]:03d}";next_snapshot+=1
        arrays={};keys=list(blocks)
        for i,key in enumerate(keys):arrays['v'+str(i)],arrays['t'+str(i)]=blocks[key]
        arrays['keys']=np.asarray(keys,dtype=np.int32)
        np.savez(args.output/(tag+'.npz'),**arrays)
        (args.output/(tag+'.json')).write_text(json.dumps(dict(pose=pose,frame=msg.header.frame_id,
            block_size=msg.block_size_m,blocks=len(keys),bag_elapsed=pose['stamp']-start),indent=2))
        print(tag,len(keys),pose['position'],flush=True)
    node.create_subscription(Odometry,'/nvblox_lio/odom_lidar',odom,qos_profile_sensor_data)
    node.create_subscription(Mesh,'/nvblox_node/mesh',mesh,10,raw=True)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
