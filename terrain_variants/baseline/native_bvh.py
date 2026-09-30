"""Native inclusive AABB BVH, with deterministic original-index results."""
import ctypes
from pathlib import Path
import numpy as np

_lib=ctypes.CDLL(str(Path(__file__).with_name('raster.so')))
_lib.bvh_create.argtypes=[ctypes.c_void_p,ctypes.c_void_p,ctypes.c_size_t,ctypes.c_size_t]
_lib.bvh_create.restype=ctypes.c_void_p
_lib.bvh_destroy.argtypes=[ctypes.c_void_p]
_lib.bvh_query.argtypes=[ctypes.c_void_p]*3+[ctypes.c_int,ctypes.POINTER(ctypes.c_void_p),
                         ctypes.POINTER(ctypes.c_size_t),ctypes.POINTER(ctypes.c_size_t)]
_lib.bvh_query.restype=ctypes.c_int
_lib.bvh_free_indices.argtypes=[ctypes.c_void_p]


class NativeBVH:
    def __init__(self,low,high,leaf_size=8):
        low=np.ascontiguousarray(low,dtype=np.float64).reshape(-1,3)
        high=np.ascontiguousarray(high,dtype=np.float64).reshape(-1,3)
        if low.shape!=high.shape:raise ValueError('BVH shape mismatch')
        self._ptr=_lib.bvh_create(low.ctypes.data,high.ctypes.data,len(low),leaf_size)
        if not self._ptr:raise ValueError('Invalid BVH bounds or allocation failed')

    def __del__(self):
        ptr=getattr(self,'_ptr',None)
        if ptr:_lib.bvh_destroy(ptr)

    def query(self,low,high,first=False):
        low=np.ascontiguousarray(low,dtype=np.float64)
        high=np.ascontiguousarray(high,dtype=np.float64)
        ptr,count,visits=ctypes.c_void_p(),ctypes.c_size_t(),ctypes.c_size_t()
        if _lib.bvh_query(self._ptr,low.ctypes.data,high.ctypes.data,int(first),
                          ctypes.byref(ptr),ctypes.byref(count),ctypes.byref(visits)):
            raise RuntimeError('BVH query failed')
        try:
            ids=(np.ctypeslib.as_array((ctypes.c_int64*count.value).from_address(ptr.value)).copy()
                 if count.value else np.zeros(0,dtype=np.int64))
            return ids,visits.value
        finally:_lib.bvh_free_indices(ptr)

    def iter_hits(self,low,high,visits):
        ids,n=self.query(low,high,first=True)
        visits[0]+=n
        if not len(ids):
            return
        first=int(ids[0])
        yield first
        # Most existence queries stop at the first primitive. Only enumerate
        # remaining candidates if the caller needs to inspect another block.
        ids,n=self.query(low,high)
        visits[0]+=n
        for index in ids:
            if int(index)!=first:
                yield int(index)
