"""Project connection: registered scan + pose + IMU -> recovered AOS cost cells.

ASA selects this input for stair segments. Original height-map calculations are
unchanged. Input availability checks are transport checks, not body acceptance.
"""
import threading,time
from collections import deque
import numpy as np
from height_frontend import HeightFrontend

def key(m):return m.header.stamp.sec*1000000000+m.header.stamp.nanosec

def quaternion(q):return (q.x,q.y,q.z,q.w)

class StairPerception:
    def __init__(self):
        self.lock=threading.Lock();self.odoms={};self.clouds={};self.imus=deque(maxlen=300)
        self.frontend=None;self.enabled=False;self.after=0;self.last=0;self.frames=0
        self.diagnostics=dict(stair_perception='disabled',height_frames=0)
    def enable(self,enabled,after=None):
        self.enabled=bool(enabled);self.after=time.time_ns() if after is None else int(after);self.last=0
        if self.frontend:self.frontend.close();self.frontend=None
        if enabled:
            self.frontend=HeightFrontend();self.frontend.policy.set_terrain(2);self.frontend.policy.set_stair_oa(0)
        self.diagnostics=dict(stair_perception='waiting_synchronized_scan' if enabled else 'disabled',height_frames=self.frames)
    def odometry(self,m):
        p=m.pose.pose.position;q=quaternion(m.pose.pose.orientation);pose=(p.x,p.y,p.z,*q)
        if m.header.frame_id!='nvblox_odom' or m.child_frame_id!='base_link_dog' or not np.isfinite(pose).all() or np.linalg.norm(q)<.5:return
        with self.lock:
            self.odoms[key(m)]=pose
            while len(self.odoms)>100:self.odoms.pop(next(iter(self.odoms)))
    def cloud(self,m):
        if m.header.frame_id!='base_link_dog':return
        with self.lock:
            self.clouds[key(m)]=m
            while len(self.clouds)>8:self.clouds.pop(next(iter(self.clouds)))
    def imu(self,m):
        q=quaternion(m.orientation);t=key(m)
        if not np.isfinite(q).all() or np.linalg.norm(q)<.5 or getattr(m,"orientation_covariance",[0])[0]==-1:return
        with self.lock:
            if self.imus and t<=self.imus[-1][0]:return
            self.imus.append((t,q))
    def update(self,decode,now=None):
        if not self.enabled:return None
        now=time.time() if now is None else now
        with self.lock:
            keys=[k for k in self.clouds.keys()&self.odoms.keys() if k>self.last and k>=self.after and -.05<=now-k*1e-9<.35]
            samples=list(self.imus)
            # Wait for the upper IMU bracket rather than treating missing IMU as
            # proof of level ground on an executing stair segment.
            matched=[k for k in keys if len(samples)>1 and samples[0][0]<=k<=samples[-1][0]]
            if not matched:
                self.diagnostics['stair_perception']='waiting_imu_bracket' if keys else 'waiting_synchronized_scan'
                return None
            k=max(matched);m=self.clouds[k];pose=self.odoms[k]
        try:
            for t,q in samples:self.frontend.add_imu(t*1e-9,q)
            points=decode(m)
            if not len(points) or not np.isfinite(points).all(axis=1).any():
                self.diagnostics['stair_perception']='empty_registered_scan';self.last=k;return None
            began=time.monotonic();ground=self.frontend.process(points,k*1e-9,pose)
            if not self.frontend.gravity_matched:
                self.diagnostics['stair_perception']='waiting_imu_bracket';self.last=k;return None
            result=self.frontend.height_map();self.last=k;self.frames+=1
            self.diagnostics=dict(stair_perception='ready',height_frames=self.frames,height_stamp=k*1e-9,
                height_input_points=len(points),height_input_near2m=int(np.sum(np.sum(points*points,axis=1)<4.)),
                gravity_matched=True,height_obstacle_cells=len(result['cloud_nav']),height_accumulated_points=len(ground),
                height_range=self.frontend.policy.height_range,height_stair_oa=False,height_pitch=self.frontend.pitch,
                height_process_ms=(time.monotonic()-began)*1000)
            return m.header,result['cloud_nav'],self.frontend.rotation(),self.diagnostics['height_process_ms']
        except (ValueError,RuntimeError,StopIteration,TypeError) as exc:
            self.last=k;self.diagnostics['stair_perception']='invalid_stair_input';self.diagnostics['height_error']=str(exc)
            return None
    def close(self):
        if self.frontend:self.frontend.close();self.frontend=None
