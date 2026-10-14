"""AOS accumulation/IMU/rays -> recovered height and obstacle kernels.

Input is synchronized CLOUD_REGISTERED_BODY and its full SE(3) LIO pose.
This API consumes IMU orientation (xyzw), not ASA route heights. Output nav
points are one point per obstacle cell and must not enter a raw-return counter.
No ROS, DDS, vendor executable, or robot command publisher is loaded here.
"""
import ctypes as C
from pathlib import Path
import numpy as np
try:
    from .heightmap import HeightPolicy,build_height_map,obstacle_mask
except ImportError:
    from heightmap import HeightPolicy,build_height_map,obstacle_mask

FLOAT=C.POINTER(C.c_float)
def ptr(a):return a.ctypes.data_as(FLOAT)
def library():
    lib=C.CDLL(str(Path(__file__).resolve().parents[1]/'build/source/libnos_source.so'))
    specs={
        'aos_error':([],C.c_char_p),'aos_create':([],C.c_void_p),
        'aos_destroy':([C.c_void_p],None),
        'aos_options':([C.c_void_p,C.c_int,C.c_int,C.c_int],None),
        'aos_imu':([C.c_void_p,C.c_double,FLOAT],None),
        'aos_process':([C.c_void_p,FLOAT,C.c_int,C.c_double,FLOAT],C.c_int),
        'aos_cloud':([C.c_void_p,FLOAT,C.c_int,C.c_int],C.c_int),
        'aos_matched':([C.c_void_p],C.c_int),'aos_pitch':([C.c_void_p],C.c_float),
        'aos_rotation':([C.c_void_p,FLOAT],None),
        'aos_clear':([FLOAT,C.c_int,FLOAT,C.c_int,C.c_int,FLOAT],C.c_int),
        'aos_ray_query':([FLOAT,C.c_int,FLOAT,FLOAT,C.POINTER(C.c_int)],C.c_int),
        'aos_gravity':([FLOAT,FLOAT],None),
    }
    for name,(args,result) in specs.items():
        f=getattr(lib,name);f.argtypes=args;f.restype=result
    return lib

def orientation(q):
    q=np.ascontiguousarray(q,np.float32)
    if q.shape!=(4,) or not np.isfinite(q).all() or np.dot(q,q)<=0:
        raise ValueError('IMU/odometry orientation must be finite nonzero xyzw')
    return q

class HeightFrontend:
    def __init__(self,*,accumulate=True,casting=True,parallel=True):
        self.lib=library();self.handle=self.lib.aos_create()
        if not self.handle:raise RuntimeError(self.lib.aos_error().decode())
        self.lib.aos_options(self.handle,accumulate,casting,parallel)
        self.policy=HeightPolicy();self.last_stamp=None
        self.gravity_matched=False;self.pitch=0.;self.last_imu=None
    def close(self):
        if self.handle:self.lib.aos_destroy(self.handle);self.handle=None
    def __enter__(self):return self
    def __exit__(self,*_):self.close()
    def add_imu(self,stamp,xyzw):
        if not self.handle:raise RuntimeError('height frontend is closed')
        if not np.isfinite(stamp):raise ValueError('nonfinite IMU time')
        q=orientation(xyzw)
        # Timestamp validation is transport hygiene; native assumes ordered data.
        if self.last_imu is not None and stamp<=self.last_imu:return False
        self.lib.aos_imu(self.handle,stamp,ptr(q));self.last_imu=stamp;return True
    def process(self,points,stamp,pose=(0.,0.,0.,0.,0.,0.,1.)):
        if not self.handle:raise RuntimeError('height frontend is closed')
        if not np.isfinite(stamp):raise ValueError('nonfinite cloud time')
        if self.last_stamp is not None and stamp<=self.last_stamp:raise ValueError('cloud time must increase; reset frontend after a clock jump')
        p=np.ascontiguousarray(points,np.float32).reshape(-1,3)
        pose=np.ascontiguousarray(pose,np.float32)
        if pose.shape!=(7,) or not np.isfinite(pose).all():raise ValueError('pose must be xyz+xyzw')
        orientation(pose[3:])
        if not self.lib.aos_process(self.handle,ptr(p),len(p),stamp,ptr(pose)):
            raise RuntimeError(self.lib.aos_error().decode())
        self.last_stamp=stamp;self.gravity_matched=bool(self.lib.aos_matched(self.handle))
        self.pitch=self.lib.aos_pitch(self.handle)
        return self.cloud()
    def rotation(self):
        if not self.handle:raise RuntimeError('height frontend is closed')
        out=np.empty(4,np.float32);self.lib.aos_rotation(self.handle,ptr(out));return out
    def cloud(self,body=False):
        if not self.handle:raise RuntimeError('height frontend is closed')
        n=self.lib.aos_cloud(self.handle,None,0,body);out=np.empty((n,3),np.float32)
        if self.lib.aos_cloud(self.handle,ptr(out),n,body)<0:raise RuntimeError(self.lib.aos_error().decode())
        return out
    def height_map(self):
        height,samples,counts,raw=build_height_map(self.cloud(),pitch=self.pitch)
        mask=obstacle_mask(height,samples,counts,self.policy)
        cells=np.argwhere(mask)
        # Original computeCostMap emits float-FMA grid coordinates and Z=0.
        nav=np.zeros((len(cells),3),np.float32)
        nav[:,:2]=(cells.astype(np.float64)*float(np.float32(.1))-4.).astype(np.float32)
        return dict(height=height,samples=samples,counts=counts,raw=raw,mask=mask,
                    cloud_nav=nav,gravity_matched=self.gravity_matched,stamp=self.last_stamp)
