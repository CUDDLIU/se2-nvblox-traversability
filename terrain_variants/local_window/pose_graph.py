"""Graph nodes retain metric pose witnesses; edges validate swept bodies.

A feasible yaw bit no longer asserts its whole bin is feasible, and a feasible
cell no longer asserts its center is feasible. Never infer edges from bits.
"""
import ctypes
import math
import numpy as np
from native_mesh import NativeMesh,_lib

_lib.mesh_store_geometry.argtypes=[ctypes.c_void_p]*4+[ctypes.POINTER(ctypes.c_size_t)]
_lib.mesh_store_geometry.restype=ctypes.c_int
_lib.mesh_store_motions.argtypes=[ctypes.c_void_p,ctypes.c_size_t]+[ctypes.c_void_p]*2+[ctypes.c_double]*7+[ctypes.c_void_p]
_lib.mesh_store_motions.restype=ctypes.c_int


def capture(mesh,low,high):
    low,high=mesh.bounds(low,high);ptr=ctypes.c_void_p();count=ctypes.c_size_t()
    if _lib.mesh_store_geometry(mesh._ptr,low.ctypes.data,high.ctypes.data,ctypes.byref(ptr),ctypes.byref(count)):
        raise RuntimeError('Cannot capture graph Mesh')
    try:
        if not count.value:return np.empty((0,3,3))
        return np.ctypeslib.as_array((ctypes.c_double*(count.value*9)).from_address(ptr.value)).reshape(-1,3,3).copy()
    finally:_lib.bvh_free_indices(ptr)


def validate(mesh,motions,groups,c):
    if not motions:return np.zeros(0,dtype=bool)
    data=np.ascontiguousarray(motions,dtype=np.float64).reshape(-1,12)
    groups=np.ascontiguousarray(groups,dtype=np.int64);accepted=np.zeros(len(data),dtype=np.uint8)
    code=_lib.mesh_store_motions(mesh._ptr,len(data),data.ctypes.data,groups.ctypes.data,
        c.vertical_resolution,c.resolution,c.length,c.width,c.required_height,c.max_step,c.max_slope_deg,accepted.ctypes.data)
    if code:raise RuntimeError('Swept body validation failed')
    return accepted.astype(bool)


def build(results,c,triangles,bounds):
    # Bound export to the current observed local window; old retained colored
    # cells are history, not graph permission on the current floor.
    cells={};bins=c.yaw_bins;nodes=[];edges=[];by_xy={}
    for result in results.values():
        if result.pose_states is None:continue
        for i,span in enumerate(result.spans):
            p=np.array([(span.ix+.5)*c.resolution,(span.iy+.5)*c.resolution,span.z])
            if np.any(p<bounds[0]) or np.any(p>=bounds[1]) or not result.masks[i]:continue
            key=(span.ix,span.iy,span.iz);states={}
            for yaw,state in enumerate(result.pose_states[i]):
                if not np.isfinite(state).all():continue
                identifier=f'cell/{span.ix}/{span.iy}/{span.iz}/y{yaw}'
                states[yaw]=(identifier,state)
                nodes.append(dict(id=identifier,position=state[:3].tolist(),yaw=yaw,yaw_rad=float(state[3]),
                    yaw_interval=state[4:6].tolist(),kind='feasible_pose',polygons=[]))
            cells[key]=(states,int(result.comfortable_masks[i]));by_xy.setdefault(key[:2],[]).append(key)
    motions=[];pending=[];groups=[];seen=set()
    def edge(a,b,kind):
        key=tuple(sorted((a[0],b[0])))
        if key in seen:return None
        seen.add(key)
        return dict(source=key[0],target=key[1],kind=kind,polygons=[],
                    length=float(np.linalg.norm(a[1][:3]-b[1][:3])),
                    angle=abs(math.remainder(float(b[1][3]-a[1][3]),2*math.pi)))
    for group,(key,(states,comfortable)) in enumerate(sorted(cells.items())):
        for yaw,a in sorted(states.items()):
            candidates=[]
            other=states.get((yaw+1)%bins)
            if other is not None:candidates.append((other,'rotation',bool(comfortable&(1<<yaw) and comfortable&(1<<((yaw+1)%bins)))))
            for xy in ((key[0]+1,key[1]),(key[0],key[1]+1),(key[0]+1,key[1]+1),(key[0]+1,key[1]-1)):
                for target in by_xy.get(xy,[]):
                    if abs(target[2]-key[2])*c.vertical_resolution>c.max_step+1e-8:continue
                    bs,bc=cells[target]
                    if yaw in bs:candidates.append((bs[yaw],'translation',bool(comfortable&bc&(1<<yaw)) and (xy[0]==key[0] or xy[1]==key[1])))
            for b,kind,proved in candidates:
                item=edge(a,b,kind)
                if item is None:continue
                if proved:edges.append(item)
                else:motions.append(np.r_[a[1],b[1]]);pending.append(item);groups.append(group)
    mesh=NativeMesh();mesh.update_blocks({(0,0,0):(triangles.reshape(-1,3),np.arange(triangles.size//3).reshape(-1,3))},c)
    for item,ok in zip(pending,validate(mesh,motions,groups,c)):
        if ok:edges.append(item)
    return dict(nodes=nodes,edges=edges,directed=False,yaw_bins=bins,
                translation_model='metric_pose_swept_body',scope='current_local_window',
                bounds=np.asarray(bounds).tolist(),refined_edges_checked=len(pending))
