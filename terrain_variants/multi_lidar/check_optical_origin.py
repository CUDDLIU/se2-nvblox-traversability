"""Read-only scan-model check against seven un-deskewed bag point samples.

RoboSense AIRY decoder adds RZ on the spindle and a rotating Rxy offset.
This inverse assumes the radial optical offset and beam azimuth agree; the
small per-channel horizontal correction makes it an approximation. No point,
pose, public TF or bag is rewritten.
"""
import json
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
RZ = .04532
RXY = float(np.hypot(.0075, .00664))


def angles(points, rings, axial_offset=0., radial_offset=0.):
    sign = np.where(rings < 96, 1., -1.)
    axis = sign * points[:, 0] - .32028 - axial_offset
    radius = np.hypot(points[:, 1], points[:, 2] + .013) - radial_offset
    return np.degrees(np.arctan2(axis, radius))


def main():
    variants = [('housing_origin', 0., 0.), ('axial_optical_plane', RZ, 0.),
                ('decoder_inverse', RZ, RXY)]
    references = {}
    rows = []
    for path in sorted((HERE.parent / 'sensor_alignment').glob('sensors_*.npz')):
        source = np.load(path)
        points, rings = source['lidar_body'].astype(float), source['lidar_rings']
        row = dict(sample=path.name, variants={})
        for name, axial, radial in variants:
            elevation = angles(points, rings, axial, radial)
            ring_rows = []
            for ring in np.unique(rings):
                values = elevation[rings == ring]
                if len(values) < 20:
                    continue
                ring_rows.append(dict(ring=int(ring), n=len(values),
                    mean=float(np.mean(values)), std=float(np.std(values))))
            summary = dict(rings=ring_rows, median_ring_std_deg=float(np.median([r['std'] for r in ring_rows])),
                           p95_ring_std_deg=float(np.percentile([r['std'] for r in ring_rows], 95)))
            if name not in references:
                references[name] = {r['ring']: r['mean'] for r in ring_rows}
            else:
                reference = references[name]
                errors = [abs(r['mean'] - reference[r['ring']]) for r in ring_rows if r['ring'] in reference]
                summary['heldout_median_ring_angle_error_deg'] = float(np.median(errors))
                summary['heldout_p95_ring_angle_error_deg'] = float(np.percentile(errors, 95))
            row['variants'][name] = summary
        rows.append(row)
    result = dict(scope=__doc__, axial_offset_m=RZ, rotating_radius_m=RXY,
        origin_world_endpoint_effect='none; diagnostic only',
        sources=['https://github.com/RoboSense-LiDAR/rs_driver/blob/main/src/rs_driver/driver/decoder/decoder_RSAIRY.hpp',
                 'https://github.com/RoboSense-LiDAR/rs_driver/blob/main/src/rs_driver/driver/decoder/decoder_mech.hpp'],
        samples=rows)
    (HERE / 'optical_origin_evidence.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    for row in rows:
        print(row['sample'], {name: {k:v for k,v in value.items() if k != 'rings'} for name,value in row['variants'].items()})


if __name__ == '__main__':
    main()
