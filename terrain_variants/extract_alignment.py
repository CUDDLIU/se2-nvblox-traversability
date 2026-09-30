"""Read matched sensor samples and deskew with the recorded 200 Hz poses.

Read-only SQLite queries. Writes evidence inside the owning bag; does not
change extrinsics or claim a calibration from a single flat surface.
"""
import argparse
import json
from pathlib import Path
import sqlite3
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2,Image,CameraInfo
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import yaml

def matrix(translation,rotation):
    t=np.eye(4);t[:3,:3]=Rotation.from_quat(rotation).as_matrix();t[:3,3]=translation;return t

def main():
    p=argparse.ArgumentParser();p.add_argument('bag',type=Path);p.add_argument('--times',type=float,nargs='+',default=[25,60,95,125,140,175,205]);args=p.parse_args()
    data=args.bag/'data';metadata=yaml.safe_load((data/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    begin=metadata['starting_time']['nanoseconds_since_epoch'];dbs=[]
    for f in metadata['relative_file_paths']:
        c=sqlite3.connect('file:'+str(data/f)+'?mode=ro',uri=True)
        lo=c.execute('SELECT timestamp FROM messages ORDER BY timestamp LIMIT 1').fetchone()[0]
        hi=c.execute('SELECT timestamp FROM messages ORDER BY timestamp DESC LIMIT 1').fetchone()[0]
        dbs.append((c,lo,hi,dict(c.execute('SELECT name,id FROM topics'))))
    def fetch(topic,ns,radius=.4):
        lo,hi=ns-int(radius*1e9),ns+int(radius*1e9);rows=[]
        for c,a,b,topics in dbs:
            if b<lo or a>hi or topic not in topics:continue
            rows+=c.execute('SELECT timestamp,data FROM messages WHERE timestamp BETWEEN ? AND ? AND topic_id=? ORDER BY timestamp',(lo,hi,topics[topic])).fetchall()
        return rows
    def nearest(topic,ns,cls):
        rows=fetch(topic,ns)
        if not rows:raise ValueError('Missing '+topic)
        return deserialize_message(min(rows,key=lambda r:abs(r[0]-ns))[1],cls)
    def stamp(msg):return msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
    edges={}
    for _,raw in fetch('/tf_static',begin+500000000,1.):
        for t in deserialize_message(raw,TFMessage).transforms:
            v=t.transform.translation;q=t.transform.rotation
            edges[t.child_frame_id]=(t.header.frame_id,matrix([v.x,v.y,v.z],[q.x,q.y,q.z,q.w]))
    def static_to_base(frame):
        out=np.eye(4);visited=set()
        while frame!='base_link_dog':
            if frame in visited or frame not in edges:raise ValueError('No static base transform for '+frame)
            visited.add(frame);frame,t=edges[frame];out=t@out
        return out
    out=args.bag/'diagnostics/sensor_alignment';out.mkdir(parents=True,exist_ok=True)
    for elapsed in args.times:
        ns=begin+int(elapsed*1e9)
        depth=nearest('/camera/d435i/depth/image_rect_raw',ns,Image)
        info=nearest('/camera/d435i/depth/camera_info',ns,CameraInfo)
        cloud=nearest('/LIDAR/POINTS_NX',ns,PointCloud2)
        pose_rows=[]
        for _,raw in fetch('/nvblox_lio/odom',ns,.6):
            o=deserialize_message(raw,Odometry);v=o.pose.pose.position;q=o.pose.pose.orientation
            pose_rows.append((stamp(o),[v.x,v.y,v.z],[q.x,q.y,q.z,q.w]))
        unique={p[0]:p for p in pose_rows};pose_rows=[unique[t] for t in sorted(unique)]
        ts=np.asarray([p[0] for p in pose_rows]);xyz=np.asarray([p[1] for p in pose_rows]);rot=Rotation.from_quat([p[2] for p in pose_rows]);slerp=Slerp(ts,rot)
        def transform_at(t):
            t=np.asarray(t);r=slerp(t).as_matrix();v=np.column_stack([np.interp(t,ts,xyz[:,j]) for j in range(3)])
            return r,v
        fields={f.name:f for f in cloud.fields}
        dtype=np.dtype(dict(names=['x','y','z','ring','timestamp'],formats=['<f4','<f4','<f4','<u2','<f8'],
            offsets=[fields[n].offset for n in ['x','y','z','ring','timestamp']],itemsize=cloud.point_step))
        points=np.ndarray((cloud.height,cloud.width),dtype=dtype,buffer=cloud.data,strides=(cloud.row_step,cloud.point_step)).ravel()
        lidar=np.column_stack([points[n] for n in ['x','y','z']]);times=points['timestamp'].copy();rings=points['ring'].copy()
        valid=np.isfinite(lidar).all(axis=1)&np.isfinite(times)&(times>=ts[0])&(times<=ts[-1])&(np.linalg.norm(lidar,axis=1)<6)&(np.linalg.norm(lidar,axis=1)>.4)
        lidar,times,rings=lidar[valid],times[valid],rings[valid]
        tbase=static_to_base(cloud.header.frame_id);base=lidar@tbase[:3,:3].T+tbase[:3,3]
        rr,vv=transform_at(times);lidar_world=np.einsum('nij,nj->ni',rr,base)+vv
        if depth.encoding!='16UC1':raise ValueError('Unsupported depth encoding '+depth.encoding)
        dt='>u2' if depth.is_bigendian else '<u2'
        dep=np.ndarray((depth.height,depth.width),dtype=dt,buffer=depth.data,strides=(depth.step,2))[::3,::3].astype(float)*.001
        y,x=np.mgrid[0:depth.height:3,0:depth.width:3];k=np.asarray(info.k).reshape(3,3)
        cam=np.stack(((x-k[0,2])*dep/k[0,0],(y-k[1,2])*dep/k[1,1],dep),axis=-1)
        cam=cam[(dep>.4)&(dep<4.)]
        camera_base=static_to_base(depth.header.frame_id);r,v=transform_at(np.array([stamp(depth)]))
        base=cam@camera_base[:3,:3].T+camera_base[:3,3]
        depth_world=base@r[0].T+v[0]
        tag=f'sensors_{elapsed:06.1f}'
        np.savez(out/(tag+'.npz'),lidar_world=lidar_world,depth_world=depth_world,
            depth_camera=cam,lidar_body=lidar,lidar_times=times,lidar_rings=rings,
            pose_times=ts,pose_positions=xyz,pose_quaternions=rot.as_quat(),
            T_base_camera=camera_base,R_world_base=r[0],t_world_base=v[0])
        meta=dict(elapsed=elapsed,depth_stamp=stamp(depth),lidar_stamp=stamp(cloud),
            lidar_frame=cloud.header.frame_id,depth_frame=depth.header.frame_id,lidar_points=len(lidar),depth_points=len(cam),
            T_base_camera=camera_base.tolist(),ring_range=[int(rings.min()),int(rings.max())],
            point_time_range=[float(times.min()),float(times.max())],static_frames=list(edges))
        (out/(tag+'.json')).write_text(json.dumps(meta,indent=2));print(tag,meta['lidar_points'],meta['depth_points'],flush=True)
    for c,*_ in dbs:c.close()

if __name__=='__main__':main()
