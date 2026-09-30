"""One native call for selected roots, preserving the full contextual graph."""
import ctypes
from pathlib import Path

import numpy as np

_lib=ctypes.CDLL(str(Path(__file__).with_name('raster.so')))
_lib.classify_selected.argtypes=([ctypes.c_size_t]+[ctypes.c_void_p]*5+
    [ctypes.c_double]*4+[ctypes.c_uint64,ctypes.c_void_p,ctypes.c_size_t]+
    [ctypes.c_void_p]*6+[ctypes.c_size_t]+[ctypes.c_void_p]*3)
_lib.classify_selected.restype=ctypes.c_int
LABELS=('walkable','slope','headroom','unknown_boundary',
        'ledge_or_nonwalkable_neighbor','ambiguous_layer',
        'footprint_or_boundary_distance','restricted','safe')


class Classifier:
    def __init__(self, config, masks):
        self.config=config
        unique={}
        for yaw,mask in enumerate(masks):
            key=tuple(mask[0])
            if key not in unique:unique[key]=[mask,0]
            unique[key][1] |= 1<<yaw
        parents=[];directions=[];checks=[];offsets=[0];check_offsets=[0];bits=[]
        for (_,p,d,c),bit in unique.values():
            parents.extend(p);directions.extend(d);checks.extend(c)
            offsets.append(len(parents));check_offsets.append(len(checks));bits.append(bit)
        self.buffers=[np.ascontiguousarray(a,dtype=np.int64) for a in
                      (parents,directions,checks,offsets,check_offsets)]
        self.buffers.append(np.ascontiguousarray(bits,dtype=np.uint64))
        self.mask_count=len(bits)

    def __call__(self, spans, roots=None):
        n=len(spans)
        if not n:return np.zeros(0,dtype=np.uint64),np.zeros(0),[]
        xy=np.asarray([(s.ix,s.iy) for s in spans],dtype=np.int64)
        z=np.asarray([s.z for s in spans],dtype=np.float64)
        roof=np.asarray([s.ceiling for s in spans],dtype=np.float64)
        walk=np.asarray([s.walkable for s in spans],dtype=np.uint8)
        slope=np.asarray([s.slope_ok for s in spans],dtype=np.uint8)
        return self.arrays(xy,z,roof,walk,slope,roots)

    def from_records(self,records,roots):
        xy=np.column_stack((records['ix'],records['iy'])).astype(np.int64)
        z=records['hi'].astype(np.float64)*self.config.vertical_resolution
        roof=records['ceiling'].astype(np.float64)*self.config.vertical_resolution
        roof[records['ceiling']==np.iinfo(np.int64).max]=np.inf
        walk=np.ascontiguousarray(records['walkable'],dtype=np.uint8)
        slope=np.ascontiguousarray(records['slope_ok'],dtype=np.uint8)
        return self.arrays(xy,z,roof,walk,slope,roots)

    def arrays(self,xy,z,roof,walk,slope,roots):
        n=len(z)
        if not n:return np.zeros(0,dtype=np.uint64),np.zeros(0),[]
        roots=(np.arange(n,dtype=np.int64) if roots is None else
               np.ascontiguousarray(roots,dtype=np.int64))
        if roots.ndim!=1 or (len(roots) and (roots.min()<0 or roots.max()>=n)):
            raise ValueError('Invalid classification roots')
        geometry=np.zeros(n,dtype=np.uint64);distances=np.zeros(n);reasons=np.zeros(n,dtype=np.int32)
        c=self.config
        code=_lib.classify_selected(n,xy.ctypes.data,z.ctypes.data,roof.ctypes.data,
            walk.ctypes.data,slope.ctypes.data,c.max_step,c.required_height,c.resolution,
            min(c.length/2+c.side_margin,c.width/2+c.side_margin),(1<<c.yaw_bins)-1,
            roots.ctypes.data,len(roots),*[a.ctypes.data for a in self.buffers],self.mask_count,
            geometry.ctypes.data,distances.ctypes.data,reasons.ctypes.data)
        if code:raise ValueError(f'Native classification failed ({code})')
        return geometry,distances,[LABELS[i] for i in reasons.tolist()]
