#!/usr/bin/env python3
"""Offline geometry audit of the captured native templates; never sends control."""
import json, math
from pathlib import Path
root = Path(__file__).resolve().parents[2]
assets = root / 'vendor/paths/first'
rows = [[float(v) for v in line.split()] for line in (assets/'path_end.ply').read_text().splitlines()]
paths = {int(row[-1]): [] for row in rows}
for line in (assets/'pathList.ply').read_text().splitlines():
    x, y, _, path_id = map(float, line.split())
    paths[int(path_id)].append((x, y))
result = {'table_cells_xy': [90, 80], 'resolution_m': .05,
          'table_bounds_xy_m': [[-1.475, 3.025], [-1.975, 2.025]],
          'scope': 'Ten native samples with rotated footprint corners; counterexamples, not continuous safety certification.'}
for label, allowed, length, width in (
    ('official', rows, .42, .25),
    ('current_caps', [r for r in rows if abs(r[4]) <= 1.5 and abs(r[5]) <= .4 and abs(r[6]) <= .8 and math.hypot(r[4], r[5]) <= 1.5], .44536, .28836)):
    extent = [math.inf, -math.inf, math.inf, -math.inf]
    outside = []
    for row in allowed:
        exceeds = False
        for i, (x, y) in enumerate(paths[int(row[-1])]):
            angle = row[6] * (i+1) * .1
            c, s = math.cos(angle), math.sin(angle)
            for dx in (-length, length):
                for dy in (-width, width):
                    px, py = x+c*dx-s*dy, y+s*dx+c*dy
                    extent = [min(extent[0],px), max(extent[1],px), min(extent[2],py), max(extent[3],py)]
                    exceeds |= abs(py) > 1.1
        if exceeds:
            outside.append(int(row[-1]))
    result[label] = dict(primitives=len(allowed), swept_xy_bounds_m=extent,
                         outside_2_2m_width=len(outside), outside_path_ids=outside)
print(json.dumps(result, indent=2))
