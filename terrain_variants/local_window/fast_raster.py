import ctypes
from pathlib import Path
from collections import defaultdict
import math
import numpy as np
import shapely

_lib = ctypes.CDLL(str(Path(__file__).with_name('raster.so')))
_lib.raster.argtypes = [ctypes.c_void_p,ctypes.c_size_t,ctypes.c_void_p,ctypes.c_size_t,
                       ctypes.c_double,ctypes.c_double,ctypes.POINTER(ctypes.c_void_p),
                       ctypes.POINTER(ctypes.c_size_t)]
_lib.raster.restype = ctypes.c_int
_lib.free_records.argtypes = [ctypes.c_void_p]
_lib.rectangles_clear.argtypes = [ctypes.c_void_p,ctypes.c_int,ctypes.c_int,
                                ctypes.c_void_p,ctypes.c_void_p,ctypes.c_void_p,
                                ctypes.c_size_t,ctypes.c_void_p]
_lib.rectangles_clear.restype = None
_lib.evaluate_mask.argtypes = ([ctypes.c_size_t]+[ctypes.c_void_p]*8+
                              [ctypes.c_size_t,ctypes.c_void_p,ctypes.c_size_t,
                               ctypes.c_double,ctypes.c_uint64]+[ctypes.c_void_p]*3)
_lib.evaluate_mask.restype = None
_dtype = np.dtype([('ix','i4'),('iy','i4'),('support','i4'),('reserved','i4'),
                  ('lo','f8'),('hi','f8'),('xyz','f8',(8,3))])


def evaluate_masks_native(neighbors,missing,z,ceiling,known,covered,masks,height):
    neighbors=np.ascontiguousarray(neighbors,dtype=np.int64)
    missing=np.ascontiguousarray(missing,dtype=np.uint8)
    z=np.ascontiguousarray(z,dtype=np.float64)
    ceiling=np.ascontiguousarray(ceiling,dtype=np.float64)
    known=np.ascontiguousarray(known,dtype=np.float64)
    covered=np.ascontiguousarray(covered,dtype=np.uint8)
    n=len(z)-1
    geometric=np.zeros(n,dtype=np.uint64)
    verified=np.zeros(n,dtype=np.uint64)
    unknown=np.zeros(n,dtype=np.uint64)
    for (_,parents,directions,checks),bit in masks:
        parents=np.ascontiguousarray(parents,dtype=np.int64)
        directions=np.ascontiguousarray(directions,dtype=np.int64)
        checks=np.ascontiguousarray(checks,dtype=np.int64).reshape(-1,3)
        _lib.evaluate_mask(n,neighbors.ctypes.data,missing.ctypes.data,z.ctypes.data,
            ceiling.ctypes.data,known.ctypes.data,covered.ctypes.data,parents.ctypes.data,
            directions.ctypes.data,len(parents),checks.ctypes.data,len(checks),height,int(bit),
            geometric.ctypes.data,verified.ctypes.data,unknown.ctypes.data)
    return geometric,verified,unknown


def rectangles_clear(depth, lo, hi, threshold):
    depth = np.ascontiguousarray(depth,dtype=np.float32)
    lo = np.ascontiguousarray(lo,dtype=np.int64).reshape(-1,2)
    hi = np.ascontiguousarray(hi,dtype=np.int64).reshape(-1,2)
    threshold = np.ascontiguousarray(threshold,dtype=np.float64).reshape(-1)
    if depth.ndim != 2 or len(lo)!=len(hi) or len(lo)!=len(threshold):
        raise ValueError('Invalid rectangle arrays')
    result = np.zeros(len(lo),dtype=np.uint8)
    _lib.rectangles_clear(depth.ctypes.data,*depth.shape,lo.ctypes.data,hi.ctypes.data,
                          threshold.ctypes.data,len(lo),result.ctypes.data)
    return result.astype(bool)


def rasterize(vertices, triangles, config, triangle_owners=None):
    vertices = np.ascontiguousarray(vertices,dtype=np.float64).reshape(-1,3)
    triangles = np.ascontiguousarray(triangles,dtype=np.int64).reshape(-1,3)
    if not np.isfinite(vertices).all():
        raise ValueError('Nonfinite vertex')
    ptr,count = ctypes.c_void_p(),ctypes.c_size_t()
    code = _lib.raster(vertices.ctypes.data,len(vertices),triangles.ctypes.data,len(triangles),
                       config.resolution,math.cos(math.radians(config.candidate_slope_deg)),
                       ctypes.byref(ptr),ctypes.byref(count))
    if code:
        raise ValueError('Invalid mesh index/extent or raster allocation failure')
    out = defaultdict(list) if triangle_owners is None else defaultdict(lambda:defaultdict(list))
    if not count.value:
        return {}
    try:
        buffer = (ctypes.c_char * (count.value*_dtype.itemsize)).from_address(ptr.value)
        records = np.frombuffer(buffer,dtype=_dtype)
        valid = records['support'] != 0
        polygons = np.empty(len(records),dtype=object)
        polygons[:] = None
        polygons[valid] = shapely.polygons(records['xyz'][valid])
        owners = (np.zeros(len(records),dtype=np.int32) if triangle_owners is None else
                  triangle_owners[records['reserved']])
        for owner,x,y,lo,hi,poly in zip(owners,records['ix'],records['iy'],records['lo'],records['hi'],polygons):
            target = out if triangle_owners is None else out[int(owner)]
            target[(int(x),int(y))].append((float(lo),float(hi),poly))
    finally:
        _lib.free_records(ptr)
    return dict(out)


def rasterize_blocks(blocks, config):
    """One native/GEOS batch, retaining exact block ownership for deletion."""
    keys, vertices, triangles, owners = [], [], [], []
    offset = 0
    for key,(v,t) in blocks.items():
        v = np.asarray(v,dtype=np.float64).reshape(-1,3)
        t = np.asarray(t,dtype=np.int64).reshape(-1,3)
        if len(t) and (t.min()<0 or t.max()>=len(v)):
            raise ValueError('Mesh triangle index outside its own block')
        owners.extend([len(keys)]*len(t))
        keys.append(key)
        vertices.append(v)
        triangles.append(t+offset)
        offset += len(v)
    if not keys:
        return {}
    result = rasterize(np.concatenate(vertices),np.concatenate(triangles),config,
                       np.asarray(owners,dtype=np.int32))
    return {key:dict(result.get(i,{})) for i,key in enumerate(keys)}
