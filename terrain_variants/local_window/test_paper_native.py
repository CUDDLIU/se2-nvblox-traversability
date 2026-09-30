import ctypes
from collections import defaultdict
import math
import unittest

import numpy as np

from paper_native import voxel_spans, span_values
from paper_pipeline import Config
from fast_raster import _lib, _dtype


def plane(x0, y0, x1, y1, z=0., slope=0., downward=False):
    vertices = np.array([[x0, y0, z], [x1, y0, z + slope * (x1 - x0)],
                         [x1, y1, z + slope * (x1 - x0)], [x0, y1, z]])
    triangles = np.array([[0, 1, 2], [0, 2, 3]])
    return vertices, triangles[:, ::-1] if downward else triangles


def combine(*meshes):
    vertices, triangles, offset = [], [], 0
    for vertices_i, triangles_i in meshes:
        vertices.append(vertices_i)
        triangles.append(triangles_i + offset)
        offset += len(vertices_i)
    return np.concatenate(vertices), np.concatenate(triangles)


def rows_python(vertices, triangles, config):
    """Frozen pre-optimization merger, independent of the production caller."""
    vertices = np.ascontiguousarray(vertices, dtype=np.float64).reshape(-1, 3)
    triangles = np.ascontiguousarray(triangles, dtype=np.int64).reshape(-1, 3)
    pointer, count = ctypes.c_void_p(), ctypes.c_size_t()
    code = _lib.raster(vertices.ctypes.data, len(vertices), triangles.ctypes.data,
                       len(triangles), config.resolution,
                       math.cos(math.radians(config.max_slope_deg)),
                       ctypes.byref(pointer), ctypes.byref(count))
    if code:
        raise ValueError('Reference raster failure')
    columns = defaultdict(list)
    dz = config.vertical_resolution
    try:
        if count.value:
            buffer = (ctypes.c_char * (count.value * _dtype.itemsize)).from_address(pointer.value)
            for rec in np.frombuffer(buffer, dtype=_dtype):
                low = math.floor(float(rec['lo']) / dz + 1e-6)
                high = max(low + 1, math.ceil(float(rec['hi']) / dz - 1e-6))
                columns[int(rec['ix']), int(rec['iy'])].append((low, high, bool(rec['support'])))
    finally:
        _lib.free_records(pointer)
    result = []
    for (ix, iy), records in sorted(columns.items()):
        merged = []
        for low, high, slope in sorted(records):
            if not merged or low > merged[-1][1]:
                merged.append([low, high, slope])
            else:
                if high > merged[-1][1]:
                    merged[-1][2] = slope
                elif high == merged[-1][1]:
                    merged[-1][2] |= slope
                merged[-1][1] = max(high, merged[-1][1])
        for i, (low, high, slope) in enumerate(merged):
            z = high * dz
            ceiling = merged[i + 1][0] * dz if i + 1 < len(merged) else math.inf
            walkable = slope and ceiling - z >= config.required_height - 1e-9
            result.append((ix, iy, high, z, ceiling, slope, walkable))
    return result


class NativeSpanTests(unittest.TestCase):
    def compare(self, vertices, triangles, config):
        native = span_values(voxel_spans(vertices, triangles, config.resolution,
                                         config.vertical_resolution,
                                         math.cos(math.radians(config.max_slope_deg)),
                                         config.required_height), config.vertical_resolution)
        self.assertEqual(sorted(native), rows_python(vertices, triangles, config))

    def test_flat_and_multilevel(self):
        config = Config()
        self.compare(*combine(plane(0, 0, 1, 1), plane(0, 0, 1, 1, 1.2)), config)

    def test_slope_and_downward_roof(self):
        config = Config()
        self.compare(*plane(0, 0, 1, 1, slope=math.tan(math.radians(10))), config)
        self.compare(*combine(plane(0, 0, 1, 1),
                              plane(0, 0, 1, 1, .99, downward=True)), config)

    def test_tiny_and_touching_intervals(self):
        config = Config()
        self.compare(*plane(.021, .021, .025, .025), config)
        self.compare(*combine(plane(0, 0, 1, 1), plane(0, 0, 1, 1, .01)), config)

    def test_empty(self):
        config = Config()
        records = voxel_spans(np.empty((0, 3)), np.empty((0, 3), dtype=np.int64),
                              config.resolution, config.vertical_resolution,
                              math.cos(math.radians(config.max_slope_deg)),
                              config.required_height)
        self.assertEqual(len(records), 0)

    def test_equal_top_support_or_and_higher_top_replacement(self):
        config = Config()
        mixed = combine(plane(-.5, -.5, .5, .5, .006, downward=True),
                        plane(-.5, -.5, .5, .5, .005))
        self.compare(*mixed, config)
        records = voxel_spans(*mixed, .1, .01, math.cos(math.radians(15)), 1.)
        center = records[(records['ix'] == 0) & (records['iy'] == 0)]
        self.assertTrue(center[0]['slope_ok'])
        higher = combine(*[mixed], plane(-.5, -.5, .5, .5, .011, downward=True))
        self.compare(*higher, config)
        records = voxel_spans(*higher, .1, .01, math.cos(math.radians(15)), 1.)
        center = records[(records['ix'] == 0) & (records['iy'] == 0)]
        self.assertFalse(center[0]['slope_ok'])

    def test_negative_coordinates_and_quantization_boundaries(self):
        config = Config()
        for z in (-.200000011, -.200000001, -.2, -.199999999, .009999999, .01, .010000011):
            self.compare(*combine(plane(-.35, -.24, .65, .77, z),
                                  plane(-.35, -.24, .65, .77, z + 1.01, downward=True)), config)

    def test_random_triangle_intervals_match_reference(self):
        rng = np.random.default_rng(7749)
        for _ in range(5):
            starts = rng.uniform([-1., -1., -.3], [1., 1., 1.5], (80, 3))
            vertices = (starts[:, None, :] + rng.uniform(-.18, .18, (80, 3, 3))).reshape(-1, 3)
            triangles = np.arange(len(vertices)).reshape(-1, 3)
            self.compare(vertices, triangles, Config())

    def test_invalid_arguments_fail_without_native_crash(self):
        mesh = plane(0, 0, 1, 1)
        with self.assertRaises(ValueError):
            voxel_spans(*mesh, 0., .01, 1., 1.)
        with self.assertRaises(ValueError):
            voxel_spans(*mesh, .1, 0., 1., 1.)
        with self.assertRaises(ValueError):
            voxel_spans(mesh[0], [[0, 1, 99]], .1, .01, 1., 1.)
        with self.assertRaises(ValueError):
            voxel_spans([[0, 0, math.inf]], [], .1, .01, 1., 1.)

    def test_returned_array_owns_its_data(self):
        mesh = plane(0, 0, 1, 1)
        original = voxel_spans(*mesh, .1, .01, 1., 1.)
        expected = original.copy()
        for i in range(10):
            voxel_spans(*plane(-1, -1, 1, 1, i * .05), .1, .01, 1., 1.)
        np.testing.assert_array_equal(original, expected)


if __name__ == "__main__":
    unittest.main()
