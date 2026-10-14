"""NOS handler source numerical API (no DDS/ROS or motion transport).

This is the HANDLE_STEER avoidance component, separate from NAV_POINTS' localPlanner.
Inputs to select() are the handler's already-filtered obstacle cloud in its local
frame. Captured configuration disables cloud timestamp transformation. Main ASA
navigation must not silently route through this second planner.
"""
import ctypes as C
from dataclasses import dataclass
import numpy as np
from height_frontend import library,ptr
D=C.POINTER(C.c_double);F=C.POINTER(C.c_float)
def dp(a):return a.ctypes.data_as(D)
def vector(values,size):
    a=np.ascontiguousarray(values,dtype=np.float64)
    if a.shape!=(size,) or not np.isfinite(a).all():raise ValueError(f'expected {size} finite values')
    return a

def kernels():
    lib=library()
    lib.handler_boundary.argtypes=[C.c_float]*3+[F]
    lib.handler_step.argtypes=[D,D,C.c_double]
    lib.handler_simpson.argtypes=[D,D,C.c_double,D]
    lib.handler_judge.argtypes=[D,C.c_int,F,C.c_int,D,F,D];lib.handler_judge.restype=C.c_int
    lib.handler_limits.argtypes=[C.c_float,C.c_double,D,D,C.c_double,C.c_double,D,D,D]
    lib.handler_select.argtypes=[F,C.c_int,D,D,D,D,D,D,D,C.POINTER(C.c_int)];lib.handler_select.restype=C.c_int
    lib.handler_cloud.argtypes=[F,C.c_int,F];lib.handler_cloud.restype=C.c_int
    return lib

@dataclass(frozen=True)
class Selection:
    valid:bool
    velocity:np.ndarray
    clearance:float
    cost:float
    accepted:int

class Handler:
    """Captured-config kernels and timer decision state; caller owns scheduling/I/O."""
    def __init__(self):
        self.lib=kernels()
        self.cfg=np.array([.4,.4,.4,1.5,.15,6,0,.8,.6,.8,.2,.2,.2,.2,.02,.02,.02])
        self.initial_maxima=np.array([2.,1.,2.]);self.maxima=self.initial_maxima.copy()
        self.slow1=np.array([.6,.4,.6]);self.slow2=np.array([.4,.2,.4])
        self.bounds=np.column_stack([self.maxima,-self.maxima]).ravel()
        self.desired=np.zeros(3);self.enabled=False;self.idle=False;self.avoiding=False
        self.stair_oa=False;self.previous_stair_oa=False
        self.previous_steer_missing=False;self.steer_missing=False
    @property
    def state(self):return 0 if not self.enabled else 1 if self.idle else 3 if self.avoiding else 2
    def process_steer(self,normalized):
        self.desired=np.clip(vector(normalized,3),-1.,1.)*self.bounds[::2]
    def steer_watchdog(self,received_count):
        self.previous_steer_missing=self.steer_missing
        self.steer_missing=received_count==0
    def set_parameters(self,maxima,stair_oa):
        self.maxima=np.abs(vector(maxima,3));self.stair_oa=bool(stair_oa)
        # Native SetOoaParam updates hard limits, not InitParams' slope coefficients.
    def update_limits(self,clearance,pitch_degrees):
        if not np.isfinite([clearance,pitch_degrees]).all():raise ValueError('nonfinite limits input')
        out=np.empty(6)
        self.lib.handler_limits(clearance,pitch_degrees,dp(self.initial_maxima),dp(-self.initial_maxima),1.,.25,dp(self.slow1),dp(self.slow2),dp(out))
        if np.float32(clearance)>=1.:
            out=np.column_stack([self.maxima,-self.maxima]).ravel()
            c=np.cos((pitch_degrees/180.)*3.1415926);out[:2]*=c*c
        self.bounds=out
    def filter_cloud(self,points):
        points=np.ascontiguousarray(points,np.float32).reshape(-1,3);out=np.empty_like(points)
        n=self.lib.handler_cloud(ptr(points),len(points),ptr(out))
        if n<0:raise RuntimeError('handler point-cloud filter failed')
        return out[:n]
    def select(self,points,measured):
        points=np.ascontiguousarray(points,np.float32).reshape(-1,3)
        if not np.isfinite(points).all():raise ValueError('nonfinite obstacle cloud')
        measured=vector(measured,3)
        # Public configuration must not make the native C ABI allocate unbounded paths.
        if self.cfg.shape!=(17,) or not np.isfinite(self.cfg).all() or not 1<=self.cfg[5]<=100 or self.cfg[5]!=int(self.cfg[5]) or self.cfg[4]<=0 or np.any(self.cfg[:4]<=0) or np.any(self.cfg[7:10]<0) or np.any(self.cfg[10:13]<=0) or np.any(self.cfg[13:]<0):
            raise ValueError('invalid handler configuration')
        if not np.isfinite(self.bounds).all() or np.any(self.bounds[::2]<self.bounds[1::2]):raise ValueError('invalid velocity bounds')
        if np.prod(np.ceil((self.bounds[::2]-self.bounds[1::2])/self.cfg[10:13])+2)>1000000:raise ValueError('candidate lattice too large')
        v=np.empty(3);d=C.c_double();cost=C.c_double();n=C.c_int()
        valid=self.lib.handler_select(ptr(points),len(points),dp(self.desired),dp(measured),dp(self.bounds),dp(self.cfg),dp(v),C.byref(d),C.byref(cost),C.byref(n))
        return Selection(bool(valid),v,d.value,cost.value,n.value)
    def tick(self,points,measured,pitch_degrees=0.,*,ready=False,stair_oa_call=None):
        """One RunTimer decision. None means native closed/no output.

        ready is CheckDataReady's four initialized inputs AND frequency check.
        stair_oa_call is an injected height-map service, never created here.
        """
        if not self.enabled:return None
        if not ready:
            self.desired[:]=0.;return np.zeros(3)
        if not np.any(self.desired):
            self.idle=True;self.desired[:]=0.;return np.zeros(3)
        self.idle=False
        if not self.previous_steer_missing and self.steer_missing:
            self.desired[:]=0.;return np.zeros(3)
        result=self.select(points,measured);self.update_limits(result.clearance,pitch_degrees)
        if not result.valid:
            self.desired[:]=0.;out=np.zeros(3)
        else:
            # Native casts each candidate and each clipped bound through float32.
            out=result.velocity.astype(np.float32).astype(np.float64)
            for i in range(3):
                if out[i]>self.bounds[2*i]:out[i]=np.float32(self.bounds[2*i])
                if out[i]<self.bounds[2*i+1]:out[i]=np.float32(self.bounds[2*i+1])
        self.avoiding=result.clearance<=.3
        if self.previous_stair_oa!=self.stair_oa and stair_oa_call is not None:
            stair_oa_call(self.stair_oa)
            # Official code records requested value even when Call returns false.
            self.previous_stair_oa=self.stair_oa
        return out
