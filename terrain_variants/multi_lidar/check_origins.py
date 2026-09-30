"""Independent geometric evidence for ring -> physical LiDAR mapping.

Input is raw body-coordinate points, before pose compensation. A scan ring
should form a cone about its own optical origin and rotation axis. Scores are
MAD of axis cosine, not a surface-registration result or an extrinsic fit.
"""
import argparse
import json
from pathlib import Path
import numpy as np


def main():
    p = argparse.ArgumentParser()
    p.add_argument('samples', type=Path)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    origins = np.array([[.32028, 0., -.013], [-.32028, 0., -.013], [0., 0., 0.]])
    result = dict(method=__doc__, origins=origins.tolist(), samples=[])
    for path in sorted(args.samples.glob('sensors_*.npz')):
        d = np.load(path)
        points, rings = d['lidar_body'], d['lidar_rings']
        scores = []
        for ring in np.unique(rings):
            xyz = points[rings == ring]
            if len(xyz) < 30:
                continue
            v = xyz[:, None, :] - origins[None, :, :]
            cosine = v[:, :, 0] / np.linalg.norm(v, axis=-1)
            mad = np.median(abs(cosine - np.median(cosine, axis=0)), axis=0)
            scores.append(dict(ring=int(ring), points=len(xyz), mad=mad.tolist()))
        groups = []
        for lo, hi in ((0, 96), (96, 192)):
            a = np.asarray([s['mad'] for s in scores if lo <= s['ring'] < hi])
            groups.append(dict(rings=[lo, hi - 1], sampled_rings=len(a),
                points=int(np.count_nonzero((rings >= lo) & (rings < hi))),
                median_mad=np.median(a, axis=0).tolist() if len(a) else None,
                preferred_front=int(np.count_nonzero(a[:, 0] < a[:, 1])) if len(a) else 0,
                preferred_rear=int(np.count_nonzero(a[:, 1] < a[:, 0])) if len(a) else 0))
        sample = dict(file=path.name, groups=groups, per_ring=scores)
        result['samples'].append(sample)
        print(path.name, json.dumps(groups))
    args.output.write_text(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
