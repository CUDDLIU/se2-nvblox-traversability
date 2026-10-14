"""ctypes binding to our compiled source; no original NOS executable or DDS SDK."""
import ctypes as C
from pathlib import Path
import numpy as np
try:
    from .task_profile import DEFAULT,FIELDS,validate_parameters
except ImportError:
    from task_profile import DEFAULT,FIELDS,validate_parameters
BASE=Path(__file__).resolve().parents[1]
def verify_assets():
    import hashlib,json
    manifest=json.loads((BASE/'evidence/manifest.json').read_text())['files']
    identities={}
    for name,item in manifest.items():
        if not name.startswith('paths/first/'):continue
        with (BASE/'vendor'/name).open('rb') as f:
            digest=hashlib.sha256()
            for chunk in iter(lambda:f.read(1024*1024),b''):digest.update(chunk)
        if digest.hexdigest()!=item['sha256']:raise RuntimeError('trajectory data identity mismatch: '+name)
        identities[name]=item['sha256']
    if not identities:raise RuntimeError('missing trajectory identity manifest')
    return identities
def finite_pose(values):
    if len(values)!=3 or not np.isfinite(values).all():raise ValueError('pose must contain three finite numbers')
    return (C.c_double*3)(*values)
class Output(C.Structure):
    _fields_=[(n,C.c_double) for n in ('x','y','w','score')]+[(n,C.c_int) for n in ('id','status','hard','retries')]
class Core:
    def __init__(self):
        self.identities=verify_assets()
        self.lib=C.CDLL(str(BASE/'build/source/libnos_source.so'));d=C.POINTER(C.c_double);f=C.POINTER(C.c_float)
        specs={'nos_create':([C.c_char_p],C.c_void_p),'nos_destroy':([C.c_void_p],None),'nos_goal':([C.c_void_p],None),'nos_cancel':([C.c_void_p],None),'nos_error':([],C.c_char_p),'nos_cloud':([C.c_void_p,f,C.c_int,C.c_int],C.c_int),'nos_filtered':([C.c_void_p,f,C.c_int],C.c_int),'nos_plan':([C.c_void_p,d,d,C.c_int,C.c_double,C.c_double,C.c_double,C.POINTER(Output)],C.c_int)}
        for name,(args,result) in specs.items():fn=getattr(self.lib,name);fn.argtypes=args;fn.restype=result
        self.lib.nos_cloud_reference.argtypes=self.lib.nos_cloud.argtypes;self.lib.nos_cloud_reference.restype=C.c_int
        self.lib.nos_cloud_stats.argtypes=[C.c_void_p,d];self.lib.nos_cloud_stats.restype=None
        self.lib.nos_params.argtypes=[C.c_void_p,d,C.c_int];self.lib.nos_params.restype=C.c_int
        self.lib.nos_obstacles.argtypes=[C.c_void_p,f,C.c_int];self.lib.nos_obstacles.restype=C.c_int
        self.parameters=DEFAULT.copy()
        self.lib.nos_line.argtypes=[C.c_void_p,d,C.c_int,C.POINTER(Output)];self.lib.nos_line.restype=C.c_int
        self.handle=self.lib.nos_create(str(BASE/'vendor/paths/first').encode())
        if not self.handle:raise RuntimeError(self.lib.nos_error().decode())
    def close(self):
        if self.handle:self.lib.nos_destroy(self.handle);self.handle=None
    def configure(self,parameters):
        p=validate_parameters(parameters);a=(C.c_double*len(FIELDS))(*(float(p[k]) for k in FIELDS))
        if not self.lib.nos_params(self.handle,a,len(FIELDS)):raise ValueError('parameter transaction rejected')
        self.parameters=p
    def goal(self):self.lib.nos_goal(self.handle)
    def cancel(self):self.lib.nos_cancel(self.handle)
    def cloud(self,xyz,passability=False,reference=False):
        a=np.ascontiguousarray(xyz,dtype=np.float32).reshape(-1,3)
        fn=self.lib.nos_cloud_reference if reference else self.lib.nos_cloud
        if not fn(self.handle,a.ctypes.data_as(C.POINTER(C.c_float)),len(a),passability):raise RuntimeError(self.lib.nos_error().decode())
        n=self.lib.nos_filtered(self.handle,None,0);out=np.empty((n,3),np.float32)
        self.lib.nos_filtered(self.handle,out.ctypes.data_as(C.POINTER(C.c_float)),n);return out
    def cloud_timing(self):
        a=(C.c_double*10)();self.lib.nos_cloud_stats(self.handle,a)
        return dict(zip(('input_points','cropped_points','voxel_points','filtered_points','obstacle_cells',
                         'crop_ms','voxel_ms','radius_ms','grid_ms','total_ms'),a))
    def plan(self,local,goal,terminal=True,limits=None):
        if limits is None:limits=tuple(self.parameters[k] for k in ('maxSpeedX','maxSpeedY','maxTheta'))
        local=finite_pose(local);goal=finite_pose(goal);out=Output()
        if len(limits)!=3 or not np.isfinite(limits).all() or min(limits)<0:raise ValueError('invalid velocity limits')
        if not self.lib.nos_plan(self.handle,local,goal,terminal,*limits,C.byref(out)):raise RuntimeError(self.lib.nos_error().decode())
        return out
    def obstacles(self,xyz):
        """Already classified AOS cost cells; project semantic input interface."""
        a=np.ascontiguousarray(xyz,dtype=np.float32).reshape(-1,3)
        if not np.isfinite(a).all():raise ValueError('nonfinite obstacle cells')
        if not self.lib.nos_obstacles(self.handle,a.ctypes.data_as(C.POINTER(C.c_float)),len(a)):
            raise RuntimeError(self.lib.nos_error().decode())
        return a
    def line(self,goal,backward=False):
        goal=finite_pose(goal);out=Output()
        if not self.lib.nos_line(self.handle,goal,backward,C.byref(out)):raise RuntimeError(self.lib.nos_error().decode())
        return out
