from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
import numpy as np

from record_map import VoxelMap, cloud_xyz


class MapTests(unittest.TestCase):
    def test_voxel_export_preserves_coordinates_and_ignores_invalid_points(self):
        model = VoxelMap(.2)
        model.add([[.01, .01, 0], [.03, .02, 0], [-.01, .01, 0], [np.nan, 0, 0]])
        model.add([[1, 2, 3]])
        self.assertEqual(len(model.points), 3)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'map.pcd'
            model.save(path)
            header, binary = path.read_bytes().split(b'DATA binary\n')
            self.assertIn(b'POINTS 3\n', header)
            points = np.frombuffer(binary, dtype='<f4').reshape(-1, 3)
            self.assertTrue(np.isfinite(points).all())
            self.assertTrue(np.any(np.all(points == [1, 2, 3], axis=1)))

    def test_big_endian_cloud_with_padded_rows(self):
        fields = [SimpleNamespace(name=name, offset=i*4, datatype=7, count=1)
                  for i, name in enumerate(('x', 'y', 'z'))]
        xyz = np.array([[1, 2, 3], [4, 5, 6]], dtype='>f4')
        msg = SimpleNamespace(fields=fields, is_bigendian=True, point_step=16,
            row_step=20, width=1, height=2, data=xyz[0].tobytes()+b'\x00'*8+xyz[1].tobytes()+b'\x00'*8)
        np.testing.assert_equal(cloud_xyz(msg), [[1, 2, 3], [4, 5, 6]])


if __name__ == '__main__':
    unittest.main()
