"""Native quantized span extraction for :mod:`paper_pipeline`.

The C++ merger has the same interval arithmetic and equal-top support rule as
``paper_pipeline.voxelize``.  Returned records are copies of native memory;
the final ceiling uses ``INT64_MAX`` as an unbounded sentinel and is exposed as
``math.inf`` only by :func:`span_values`.
"""
import ctypes
import math
from pathlib import Path

import numpy as np


_lib = ctypes.CDLL(str(Path(__file__).with_name("raster.so")))
_span_dtype = np.dtype([
    ("ix", "<i4"), ("iy", "<i4"),
    ("lo", "<i8"), ("hi", "<i8"), ("ceiling", "<i8"),
    ("slope_ok", "<i4"), ("walkable", "<i4"),
], align=True)

# Check the layout on both x86 and aarch64 rather than relying on implicit
# packing or a native pointer escaping beyond the lifetime of its allocation.
assert _span_dtype.itemsize == 40 and _span_dtype.fields["lo"][1] == 8
_lib.raster_spans.argtypes = [
    ctypes.c_void_p, ctypes.c_size_t, ctypes.c_void_p, ctypes.c_size_t,
    ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double,
    ctypes.POINTER(ctypes.c_void_p), ctypes.POINTER(ctypes.c_size_t),
]
_lib.raster_spans.restype = ctypes.c_int
_lib.free_span_records.argtypes = [ctypes.c_void_p]
_lib.free_span_records.restype = None
_lib.merge_raster_records.argtypes = [ctypes.c_void_p, ctypes.c_size_t,
    ctypes.c_double, ctypes.c_double, ctypes.POINTER(ctypes.c_void_p),
    ctypes.POINTER(ctypes.c_size_t)]
_lib.merge_raster_records.restype = ctypes.c_int
_lib.span_neighbors.argtypes = ([ctypes.c_size_t]+[ctypes.c_void_p]*5+
    [ctypes.c_double]*2+[ctypes.c_void_p]*4)
_lib.span_neighbors.restype = ctypes.c_int


def batch_mesh_spans(blocks, low, high, resolution, dz, slope, height):
    """Cached triangle BVH query/raster/merge under one released GIL."""
    fn=_lib.bvh_mesh_spans
    fn.argtypes=([ctypes.c_size_t]+[ctypes.c_void_p]*7+[ctypes.c_double]*4+
                 [ctypes.POINTER(ctypes.c_void_p),ctypes.POINTER(ctypes.c_size_t),ctypes.c_void_p])
    fn.restype=ctypes.c_int
    arrays=[np.asarray(a,dtype=np.uintp) for a in (
        [b.tree._ptr for b in blocks], [b.vertices.ctypes.data for b in blocks],
        [b.triangles.ctypes.data for b in blocks], [len(b.vertices) for b in blocks],
        [len(b.triangles) for b in blocks])]
    pointer,count=ctypes.c_void_p(),ctypes.c_size_t()
    stats=np.zeros(3,dtype=np.uintp)
    code=fn(len(blocks),*[a.ctypes.data for a in arrays],low.ctypes.data,high.ctypes.data,
            resolution,dz,slope,height,ctypes.byref(pointer),ctypes.byref(count),stats.ctypes.data)
    try:
        if code:raise ValueError('Batch mesh span extraction failed')
        if count.value:
            raw=(ctypes.c_char*(count.value*_span_dtype.itemsize)).from_address(pointer.value)
            result=np.frombuffer(raw,dtype=_span_dtype).copy()
        else:result=np.empty(0,dtype=_span_dtype)
        return result,[int(x) for x in stats]
    finally:_lib.free_span_records(pointer)


def span_neighbors(xy, z, roofs, walk, slopes, step, height):
    xy = np.ascontiguousarray(xy, dtype=np.int64).reshape(-1, 2)
    n = len(xy)
    z, roofs = [np.ascontiguousarray(a, dtype=np.float64) for a in (z, roofs)]
    walk, slopes = [np.ascontiguousarray(a, dtype=np.uint8) for a in (walk, slopes)]
    if any(a.shape != (n,) for a in (z, roofs, walk, slopes)):
        raise ValueError('Span connectivity array size mismatch')
    neighbors = np.empty((n+1, 4), dtype=np.int64)
    missing = np.empty((n+1, 4), dtype=np.uint8)
    reasons = np.empty(n, dtype=np.int32)
    components = np.empty(n, dtype=np.int64)
    code = _lib.span_neighbors(n, xy.ctypes.data, z.ctypes.data, roofs.ctypes.data,
        walk.ctypes.data, slopes.ctypes.data, step, height, neighbors.ctypes.data,
        missing.ctypes.data, reasons.ctypes.data, components.ctypes.data)
    if code:
        raise ValueError('Native span connectivity failed')
    return neighbors, missing, reasons, components


def raster_records(vertices, triangles, resolution, slope_cos):
    from fast_raster import _lib as raster_lib, _dtype
    pointer, count = ctypes.c_void_p(), ctypes.c_size_t()
    code = raster_lib.raster(vertices.ctypes.data, len(vertices), triangles.ctypes.data,
                             len(triangles), resolution, slope_cos,
                             ctypes.byref(pointer), ctypes.byref(count))
    try:
        if code:
            raise ValueError('Cached raster extraction failed')
        if not count.value:
            return np.empty(0, dtype=_dtype)
        raw = (ctypes.c_char*(count.value*_dtype.itemsize)).from_address(pointer.value)
        return np.frombuffer(raw, dtype=_dtype).copy()
    finally:
        raster_lib.free_records(pointer)


def merge_records(records, vertical_resolution, height):
    from fast_raster import _dtype
    records = np.ascontiguousarray(records, dtype=_dtype)
    pointer, count = ctypes.c_void_p(), ctypes.c_size_t()
    code = _lib.merge_raster_records(records.ctypes.data, len(records), vertical_resolution,
                                     height, ctypes.byref(pointer), ctypes.byref(count))
    try:
        if code:
            raise ValueError('Cached raster merge failed')
        if not count.value:
            return np.empty(0, dtype=_span_dtype)
        raw = (ctypes.c_char*(count.value*_span_dtype.itemsize)).from_address(pointer.value)
        return np.frombuffer(raw, dtype=_span_dtype).copy()
    finally:
        _lib.free_span_records(pointer)


def voxel_spans(vertices, triangles, resolution, vertical_resolution,
                slope_cos, height):
    """Return quantized occupied columns as an aligned structured ndarray.

    Fields are ``ix, iy, lo, hi, ceiling, slope_ok, walkable``.  ``lo/hi``
    and ``ceiling`` are integer vertical indices, with ``ceiling=INT64_MAX``
    denoting an unbounded roof.  The array is independent of the native buffer.
    """
    vertices = np.ascontiguousarray(vertices, dtype=np.float64).reshape(-1, 3)
    triangles = np.ascontiguousarray(triangles, dtype=np.int64).reshape(-1, 3)
    resolution = float(resolution)
    vertical_resolution = float(vertical_resolution)
    slope_cos = float(slope_cos)
    height = float(height)
    if (not np.isfinite(vertices).all() or not np.isfinite(resolution)
            or resolution <= 0 or not np.isfinite(vertical_resolution)
            or vertical_resolution <= 0 or not np.isfinite(slope_cos)
            or not np.isfinite(height) or height < 0):
        raise ValueError("invalid raster span arguments")
    if len(triangles) and (triangles.min() < 0 or triangles.max() >= len(vertices)):
        raise ValueError("triangle index outside vertex array")
    pointer, count = ctypes.c_void_p(), ctypes.c_size_t()
    code = _lib.raster_spans(vertices.ctypes.data, len(vertices), triangles.ctypes.data,
                              len(triangles), resolution, slope_cos,
                              vertical_resolution, height,
                              ctypes.byref(pointer), ctypes.byref(count))
    if code:
        raise ValueError("native mesh span extraction failed")
    try:
        if not count.value:
            return np.empty(0, dtype=_span_dtype)
        raw = (ctypes.c_char * (count.value * _span_dtype.itemsize)).from_address(pointer.value)
        return np.frombuffer(raw, dtype=_span_dtype, count=count.value).copy()
    finally:
        _lib.free_span_records(pointer)


def span_values(records, vertical_resolution):
    """Convert native records to ``(ix, iy, iz, z, ceiling, slope, walk)`` rows."""
    records = np.asarray(records, dtype=_span_dtype)
    dz = float(vertical_resolution)
    if not np.isfinite(dz) or dz <= 0:
        raise ValueError("vertical_resolution must be positive")
    out = []
    maximum = np.iinfo(np.int64).max
    # One bulk conversion avoids seven NumPy structured scalar lookups per
    # interval, especially costly on Jetson while DDS holds the Python GIL.
    for ix,iy,lo,iz,ceiling,slope,walk in records.tolist():
        ceiling = math.inf if ceiling == maximum else ceiling * dz
        out.append((ix, iy, iz, iz * dz, ceiling, bool(slope), bool(walk)))
    return out
