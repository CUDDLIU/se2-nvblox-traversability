"""One native call for selected roots, preserving the full contextual graph."""
import ctypes
from dataclasses import replace
from terrain import footprint_masks
from pathlib import Path

import numpy as np

_lib=ctypes.CDLL(str(Path(__file__).with_name('raster.so')))
_lib.classify_selected.argtypes=([ctypes.c_size_t]+[ctypes.c_void_p]*5+
    [ctypes.c_double]*4+[ctypes.c_uint64,ctypes.c_void_p,ctypes.c_size_t]+
    [ctypes.c_void_p]*6+[ctypes.c_size_t]+[ctypes.c_void_p]*3)
_lib.classify_selected.restype=ctypes.c_int
_lib.promote_stair_risers.argtypes=([ctypes.c_size_t]+[ctypes.c_void_p]*5+[ctypes.c_double]*4)
_lib.promote_stair_risers.restype=ctypes.c_int
_lib.support_from_patch.argtypes=_lib.promote_stair_risers.argtypes
_lib.support_from_patch.restype=ctypes.c_int
_lib.mesh_store_body_states.argtypes=([ctypes.c_void_p,ctypes.c_size_t]+[ctypes.c_void_p]*4+
    [ctypes.c_size_t]+[ctypes.c_double]*7+[ctypes.c_int]+[ctypes.c_void_p]*4)
_lib.mesh_store_body_states.restype=ctypes.c_int
LABELS=('walkable','slope','headroom','unknown_boundary',
        'ledge_or_nonwalkable_neighbor','ambiguous_layer',
        'footprint_or_boundary_distance','restricted','safe')


class Classifier:
    def __init__(self, config, masks, physical_proof=True):
        self.config=config
        self.mesh=None
        self.states=None
        self.comfortable=None
        self.proved=None
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
        if physical_proof and config.side_margin>0:
            physical_config=replace(config,side_margin=0.)
            self.physical=Classifier(physical_config,footprint_masks(physical_config),physical_proof=False)
        else:self.physical=None

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
        if c.surface_normal_filter:
            walk=walk.copy();slope=slope.copy()
            accepted=_lib.support_from_patch(n,xy.ctypes.data,z.ctypes.data,roof.ctypes.data,
                walk.ctypes.data,slope.ctypes.data,c.max_step,c.required_height,c.resolution,c.max_slope_deg)
            if accepted<0:raise ValueError('Surface normal filter failed')
        self.promoted_risers=0
        if c.stair_riser_filter:
            walk=walk.copy();slope=slope.copy()
            self.promoted_risers=_lib.promote_stair_risers(n,xy.ctypes.data,z.ctypes.data,roof.ctypes.data,
                walk.ctypes.data,slope.ctypes.data,c.max_step,c.required_height,c.resolution,c.max_slope_deg)
            if self.promoted_risers<0:raise ValueError('Stair riser filter failed')
        code=_lib.classify_selected(n,xy.ctypes.data,z.ctypes.data,roof.ctypes.data,
            walk.ctypes.data,slope.ctypes.data,c.max_step,c.required_height,c.resolution,
            min(c.length/2+c.side_margin,c.width/2+c.side_margin),(1<<c.yaw_bins)-1,
            roots.ctypes.data,len(roots),*[a.ctypes.data for a in self.buffers],self.mask_count,
            geometry.ctypes.data,distances.ctypes.data,reasons.ctypes.data)
        if code:raise ValueError(f'Native classification failed ({code})')
        if self.mesh is not None:
            self.comfortable=geometry.copy()
            proved=geometry.copy()
            if self.physical is not None:
                proof=self.physical
                code=_lib.classify_selected(n,xy.ctypes.data,z.ctypes.data,roof.ctypes.data,
                    walk.ctypes.data,slope.ctypes.data,c.max_step,c.required_height,c.resolution,
                    min(c.length,c.width)/2,(1<<c.yaw_bins)-1,roots.ctypes.data,len(roots),
                    *[a.ctypes.data for a in proof.buffers],proof.mask_count,
                    proved.ctypes.data,distances.ctypes.data,reasons.ctypes.data)
                if code:raise ValueError('Physical swept footprint proof failed')
                proved |= self.comfortable
            # Complete swept-yaw channels certified by the voxel footprint
            # classifier, before sub-cell metric witnesses refine unresolved bits.
            self.proved=proved.copy()
            self.states=np.full((n,c.yaw_bins,6),np.nan,dtype=np.float64)
            code=_lib.mesh_store_body_states(self.mesh._ptr,n,xy.ctypes.data,z.ctypes.data,walk.ctypes.data,
                roots.ctypes.data,len(roots),c.vertical_resolution,c.resolution,c.length,c.width,c.required_height,c.max_step,
                c.max_slope_deg,c.yaw_bins,proved.ctypes.data,distances.ctypes.data,
                geometry.ctypes.data,self.states.ctypes.data)
            if code:raise ValueError('Physical body state classification failed')
            reasons[(geometry!=0)&(self.comfortable!=(1<<c.yaw_bins)-1)]=7
        return geometry,distances,[LABELS[i] for i in reasons.tolist()]
