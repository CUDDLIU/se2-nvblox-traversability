"""Native persistent block map: batch validation, BVH updates and dirty regions."""
import ctypes
from pathlib import Path
import numpy as np
from paper_native import _span_dtype

_lib=ctypes.CDLL(str(Path(__file__).with_name('raster.so')))
_lib.mesh_store_create.restype=ctypes.c_void_p
_lib.mesh_store_destroy.argtypes=[ctypes.c_void_p]
_lib.mesh_store_clear.argtypes=[ctypes.c_void_p]
_lib.mesh_store_dirty_window.argtypes=[ctypes.c_void_p]*3
_lib.mesh_store_update.argtypes=([ctypes.c_void_p,ctypes.c_size_t]+[ctypes.c_void_p]*7+
    [ctypes.c_int,ctypes.POINTER(ctypes.c_void_p),ctypes.POINTER(ctypes.c_size_t),ctypes.c_void_p])
_lib.mesh_store_update.restype=ctypes.c_int
_lib.mesh_store_has.argtypes=[ctypes.c_void_p]*3
_lib.mesh_store_has.restype=ctypes.c_int
_lib.mesh_store_spans.argtypes=([ctypes.c_void_p]*3+[ctypes.c_double]*4+
    [ctypes.POINTER(ctypes.c_void_p),ctypes.POINTER(ctypes.c_size_t),ctypes.c_void_p])
_lib.mesh_store_spans.restype=ctypes.c_int
_lib.bvh_free_indices.argtypes=[ctypes.c_void_p]
_lib.free_span_records.argtypes=[ctypes.c_void_p]


class NativeMesh:
    def __init__(self):
        self._ptr=_lib.mesh_store_create()
        if not self._ptr:raise MemoryError('Cannot create native Mesh')
        self.stats={};self.blocks=range(0)

    def __del__(self):
        if getattr(self,'_ptr',None):_lib.mesh_store_destroy(self._ptr)

    def clear(self):
        _lib.mesh_store_clear(self._ptr);self.blocks=range(0)

    def set_dirty_window(self,low=None,high=None):
        if low is None:_lib.mesh_store_dirty_window(self._ptr,None,None);return
        low,high=[np.ascontiguousarray(v,dtype=np.int64) for v in (low,high)]
        if low.shape!=(3,) or high.shape!=(3,) or np.any(low>high):raise ValueError('Invalid dirty window')
        _lib.mesh_store_dirty_window(self._ptr,low.ctypes.data,high.ctypes.data)

    def update_blocks(self, blocks, config, reconfigure=False):
        keys=np.ascontiguousarray(list(blocks),dtype=np.int64).reshape(-1,3)
        arrays=[]
        for v,t in blocks.values():
            v=np.ascontiguousarray(v,dtype=np.float64)
            if np.asarray(t).size and np.asarray(t).dtype.kind not in 'iu':
                raise ValueError('Native Mesh requires integer triangle indices')
            t=np.ascontiguousarray(t,dtype=np.int64)
            if v.ndim!=2 or v.shape[1]!=3 or t.ndim!=2 or t.shape[1]!=3:
                raise ValueError('Invalid native Mesh array shape')
            arrays.append((v,t))
        data=[np.asarray(a,dtype=np.uintp) for a in (
            [v.ctypes.data for v,t in arrays],[t.ctypes.data for v,t in arrays],
            [len(v) for v,t in arrays],[len(t) for v,t in arrays])]
        border=config.border;size=config.slab_size
        ptr,count=ctypes.c_void_p(),ctypes.c_size_t();stats=np.zeros(3,dtype=np.uintp)
        code=_lib.mesh_store_update(self._ptr,len(keys),keys.ctypes.data,*[a.ctypes.data for a in data],
            border.ctypes.data,size.ctypes.data,int(reconfigure),ctypes.byref(ptr),ctypes.byref(count),stats.ctypes.data)
        try:
            if code:raise ValueError('Native Mesh batch update failed')
            if count.value:
                raw=(ctypes.c_int64*(3*count.value)).from_address(ptr.value)
                dirty={tuple(row) for row in np.ctypeslib.as_array(raw).reshape(-1,3).tolist()}
            else:dirty=set()
            self.blocks=range(int(stats[2]))
            return dirty,dict(changed_blocks=int(stats[0]),skipped_unchanged_blocks=int(stats[1]))
        finally:_lib.bvh_free_indices(ptr)

    def has_mesh(self,low,high):
        low,high=self.bounds(low,high)
        result=_lib.mesh_store_has(self._ptr,low.ctypes.data,high.ctypes.data)
        if result<0:raise ValueError('Native Mesh existence query failed')
        return bool(result)

    def query_spans(self,low,high,resolution,dz,slope,height):
        low,high=self.bounds(low,high)
        ptr,count=ctypes.c_void_p(),ctypes.c_size_t();stats=np.zeros(3,dtype=np.uintp)
        code=_lib.mesh_store_spans(self._ptr,low.ctypes.data,high.ctypes.data,resolution,dz,slope,height,
            ctypes.byref(ptr),ctypes.byref(count),stats.ctypes.data)
        try:
            if code:raise ValueError('Native Mesh span query failed')
            self.stats.update(candidate_triangles=int(stats[1]),raster_cache_misses=int(stats[2]))
            if not count.value:return np.empty(0,dtype=_span_dtype)
            raw=(ctypes.c_char*(count.value*_span_dtype.itemsize)).from_address(ptr.value)
            return np.frombuffer(raw,dtype=_span_dtype).copy()
        finally:_lib.free_span_records(ptr)

    @staticmethod
    def bounds(low,high):
        low,high=[np.ascontiguousarray(a,dtype=np.float64) for a in (low,high)]
        if low.shape!=(3,) or high.shape!=(3,) or not np.isfinite(low).all() or not np.isfinite(high).all() or np.any(low>high):
            raise ValueError('Invalid native Mesh query bounds')
        return low,high
