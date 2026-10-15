#!/usr/bin/env python3
"""Match live LiDAR to the saved mesh; isolated corrected body pose for SCAN."""
import argparse
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import time
import numpy as np
from scipy.spatial.transform import Rotation
from alignment import MapAligner,deskew,rotation,correction_at
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class Localization(Node):
    def __init__(self,directory,output,initial_map_pose=None):
        super().__init__('se2_scan_localization')
        self.aligner=MapAligner(directory);self.output=Path(output);self.output.mkdir(parents=True,exist_ok=True)
        if initial_map_pose is not None:
            seed=np.asarray(initial_map_pose,dtype=float)
            if seed.shape!=(4,) or not np.isfinite(seed).all():raise ValueError('invalid map search seed')
            self.aligner.anchor=seed[:3].copy();self.aligner.seed_yaw=float(seed[3])
        self.cache=self.output.parent.parent/('localization_'+self.aligner.mesh_sha256[:12]+'.json');self.cache_loaded=False;self.publisher_key=None
        self.poses=deque(maxlen=1000);self.clouds=deque(maxlen=5);self.last_scan=0.;self.transform=None;self.stable=0
        self.good_at=0.;self.last_try=0.;self.last_ros=0.;self.last_position=None
        self.pool=ThreadPoolExecutor(max_workers=1);self.future=None;self.generation=0
        self.relocalizations=0
        self.pub=self.create_publisher(PoseStamped,'/se2_scan/map_transform',qos_profile_sensor_data)
        self.status_pub=self.create_publisher(String,'/se2_scan/localization_status',1)
        self.create_subscription(Odometry,'/nvblox_lio/odom',self.odom,qos_profile_sensor_data)
        self.create_subscription(PointCloud2,'/LIDAR/POINTS_NX',self.raw,qos_profile_sensor_data)
        self.create_timer(.05,self.tick);self.state('localizing','Matching measured surfaces near the confirmed map start')

    def state(self,state,reason,**kwargs):
        d=dict(state=state,reason=reason,time_ns=time.time_ns(),frame='se2_scan_map',source_frame='nvblox_odom',
               mesh_sha256=self.aligner.mesh_sha256,**kwargs)
        self.status_pub.publish(String(data=json.dumps(d)))
        tmp=self.output/'status.tmp';tmp.write_text(json.dumps(d,indent=2));tmp.replace(self.output/'status.json')
        with (self.output/'status.jsonl').open('a') as f:f.write(json.dumps(d)+'\n')
        if state!='tracking':self.get_logger().info(reason)

    def odom(self,m):
        if m.header.frame_id!='nvblox_odom' or m.child_frame_id!='base_link_dog':return
        p=m.pose.pose.position;q=m.pose.pose.orientation;stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        a=np.array([stamp,p.x,p.y,p.z,q.x,q.y,q.z,q.w])
        if not np.isfinite(a).all() or np.linalg.norm(a[4:])<.5:return
        if self.last_ros and (stamp<self.last_ros-.05 or (self.last_position is not None and np.linalg.norm(a[1:4]-self.last_position)>.3)):
            self.transform=None;self.good_at=0.;self.poses.clear();self.clouds.clear();self.stable=0;self.generation+=1;self.last_ros=0.
            self.aligner.recovery_seed=None
            self.pub.publish(PoseStamped())
            self.cache.unlink(missing_ok=True)
            self.state('lost','Odometry discontinuity; waiting for a new map match')
        if stamp<=self.last_ros:return
        self.last_ros=stamp;self.last_position=a[1:4];self.poses.append(a)

    def raw(self,m):self.clouds.append(m)

    def unpack(self,m):
        if m.header.frame_id not in ('lidar_link','base_link_dog') or m.is_bigendian:raise ValueError('unverified raw cloud frame/layout')
        f={f.name:f for f in m.fields}
        if any(k not in f for k in ['x','y','z','timestamp']):raise ValueError('missing raw point acquisition timestamps')
        dtype=np.dtype({'names':['x','y','z','timestamp'],'formats':['<f4','<f4','<f4','<f8'],
            'offsets':[f[k].offset for k in ['x','y','z','timestamp']],'itemsize':m.point_step})
        a=np.ndarray((m.height,m.width),dtype=dtype,buffer=m.data,strides=(m.row_step,m.point_step)).reshape(-1)[::6]
        raw=np.column_stack([a[k] for k in ['x','y','z','timestamp']]);radius=np.linalg.norm(raw[:,:3],axis=1)
        limit=getattr(getattr(self,'aligner',None),'input_range',4.)
        raw=raw[(radius>.7)&(radius<limit)]
        return raw

    def fit(self,messages,poses,previous,generation):
        began=time.monotonic()
        source=deskew(np.concatenate([self.unpack(m) for m in messages]),poses);body=poses[-1,1:4]
        m=messages[-1]
        t,y,quality=self.aligner.fit_rigid(source,body,previous);quality['compute_ms']=(time.monotonic()-began)*1000
        quality.update(input_kind='raw_deskew')
        return generation,(t,y,quality),m.header.stamp.sec+m.header.stamp.nanosec/1e9

    def tick(self):
        now=time.monotonic()
        publishers=self.get_publishers_info_by_topic('/nvblox_lio/odom')
        keys=[bytes(p.endpoint_gid).hex() for p in publishers]
        if len(keys)!=1:return
        if self.publisher_key is not None and self.publisher_key!=keys[0]:
            self.transform=None;self.good_at=0.;self.stable=0;self.generation+=1;self.cache_loaded=False
            self.aligner.recovery_seed=None
            self.pub.publish(PoseStamped());self.poses.clear();self.clouds.clear()
        self.publisher_key=keys[0]
        if not self.cache_loaded:
            self.cache_loaded=True
            try:
                d=json.loads(self.cache.read_text())
                if d['mesh_sha256']==self.aligner.mesh_sha256 and d['publisher_gid']==self.publisher_key:
                    self.transform=(np.asarray(d['translation'],dtype=float),Rotation.from_quat(d['quaternion_xyzw']).as_matrix())
                    self.aligner.recovery_seed=self.transform
                    self.state('localizing','Revalidating the saved transform for this same running odometry publisher')
            except (OSError,ValueError,KeyError,TypeError):pass
        if self.future is not None and self.future.done():
            try:
                generation,(t,y,quality),source_stamp=self.future.result()
                age=self.get_clock().now().nanoseconds/1e9-source_stamp
                if generation==self.generation:
                    if self.transform is not None:
                        ot,oy=self.transform
                        body=self.poses[-1][1:4]
                        self.stable=self.stable+1 if np.linalg.norm(correction_at(body,ot,oy,t,y))<.005 and Rotation.from_matrix(y@oy.T).magnitude()<.002 else 0
                    self.transform=(t,y)
                    if 0<=age<1.2 and (self.good_at>0 or self.stable>=5):
                        self.good_at=now
                        m=PoseStamped();m.header.frame_id='se2_scan_map';m.header.stamp=self.get_clock().now().to_msg()
                        m.pose.position.x,m.pose.position.y,m.pose.position.z=map(float,t)
                        quaternion=Rotation.from_matrix(y).as_quat()
                        m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w=map(float,quaternion);self.pub.publish(m)
                        cache=dict(mesh_sha256=self.aligner.mesh_sha256,publisher_gid=self.publisher_key,
                            translation=t.tolist(),quaternion_xyzw=quaternion.tolist(),yaw=float(Rotation.from_matrix(y).as_euler('xyz')[2]),quality=quality,time_ns=time.time_ns())
                        tmp=self.cache.with_suffix('.tmp');tmp.write_text(json.dumps(cache,indent=2));tmp.replace(self.cache)
                        self.state('tracking','Live surfaces matched to saved mesh',translation=t.tolist(),quaternion_xyzw=quaternion.tolist(),roll_pitch_yaw_deg=Rotation.from_matrix(y).as_euler('xyz',degrees=True).tolist(),quality=quality,source_age_s=age)
                    else:self.state('localizing','Waiting for five stable fresh map matches before publishing pose',stable_matches=self.stable,source_age_s=age,quality=quality)
            except (ValueError,RuntimeError) as exc:
                self.stable=0
                if not self.good_at:self.transform=None
                self.state('lost' if now-self.good_at>1.5 else 'degraded',str(exc))
                if self.good_at and now-self.good_at>1.5 and self.poses and self.transform is not None:
                    # A rejected previous correction must not lock ICP forever
                    # to the same failed basin. Stop publishing, invalidate the
                    # route, and repeat the existing strict initial match near
                    # the last map pose propagated by measured odometry.
                    t,r=self.transform;self.aligner.anchor=r@self.poses[-1][1:4]+t
                    self.aligner.recovery_seed=(t.copy(),r.copy())
                    self.aligner.seed_yaw=float(np.arctan2(r[1,0],r[0,0]))
                    self.transform=None;self.good_at=0.;self.generation+=1;self.relocalizations+=1
                    self.cache.unlink(missing_ok=True);self.pub.publish(PoseStamped())
                    self.state('localizing','Reacquiring near odometry-propagated last map pose; route invalidated',
                               anchor=self.aligner.anchor.tolist(),relocalizations=self.relocalizations)
            self.future=None
        if self.future is not None or len(self.clouds)<5 or len(self.poses)<100 or now-self.last_try<.5:return
        stamp=self.clouds[-1].header.stamp.sec+self.clouds[-1].header.stamp.nanosec/1e9
        if stamp<=self.last_scan:return
        self.last_scan=stamp;self.last_try=now
        poses=np.asarray(self.poses);self.future=self.pool.submit(self.fit,list(self.clouds),poses,self.transform,self.generation)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('map');ap.add_argument('--output',required=True)
    ap.add_argument('--initial-map-pose',nargs=4,type=float);args=ap.parse_args()
    rclpy.init();node=Localization(args.map,args.output,args.initial_map_pose)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.pool.shutdown(wait=True,cancel_futures=True);node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
