"""Shared geometry loading and checked continuous motion for global queries."""
import math
from pathlib import Path
import sys
import numpy as np

CORE = Path(__file__).resolve().parents[1] / 'terrain_variants/local_window'
if str(CORE) not in sys.path:
    sys.path.insert(0, str(CORE))
from native_mesh import NativeMesh
from pose_graph import validate


def read_mesh(path):
    with np.load(path, allow_pickle=False) as data:
        blocks = {tuple(map(int, key)): (data['v' + str(i)], data['t' + str(i)])
                  for i, key in enumerate(data['keys'])}
        trajectory = data['trajectory'].copy() if 'trajectory' in data else np.empty((0, 8))
    return blocks, trajectory


def wrap(angle):
    return math.remainder(float(angle), 2 * math.pi)


class MotionChecker:
    """Validate bounded swept rectangles, with no optimization of the endpoints.

    Long segments are subdivided because the native checker queries a local
    support/obstacle neighbourhood. Floor heights and distinct levels are kept.
    """
    def __init__(self, blocks, config):
        self.config = config
        self.mesh = NativeMesh()
        self.mesh.update_blocks(blocks, config)
        self.calls = 0

    def motions(self, pairs):
        if not pairs:
            return np.zeros(0, dtype=bool)
        segments, owner = [], []
        for k, (a, b) in enumerate(pairs):
            a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
            delta = wrap(b[3] - a[3])
            n = max(1, math.ceil(np.linalg.norm(b[:3] - a[:3]) / (self.config.resolution+1e-9)),
                    math.ceil(abs(delta) / (2*math.pi/self.config.yaw_bins+1e-9)))
            for i in range(n):
                t, u = i / n, (i + 1) / n
                p, q = a + t * (b - a), a + u * (b - a)
                p[3], q[3] = a[3] + t * delta, a[3] + u * delta
                segments.append(np.r_[p, p[3], p[3], q, q[3], q[3]])
                owner.append(k)
        accepted = validate(self.mesh, segments, np.arange(len(segments)), self.config)
        good = np.ones(len(pairs), dtype=bool)
        for k, ok in zip(owner, accepted):
            good[k] &= ok
        self.calls += len(segments)
        return good

    def motion(self, a, b):
        return bool(self.motions([(a, b)])[0])
