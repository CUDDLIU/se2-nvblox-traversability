#!/usr/bin/env python3
"""Build a static, all-floor SE(2) polygon map from a complete bag mesh."""
import argparse
from collections import defaultdict
from dataclasses import asdict
import hashlib
import json
import math
import os
import shutil
from pathlib import Path
import sys
import time

os.environ.setdefault('OMP_NUM_THREADS', '3')
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
sys.path.insert(0, str(Path(__file__).resolve().parent))
from geometry import read_mesh
import numpy as np
from paper_pipeline import Config, Engine, Span
from paper_polygons import build_polygons, _line_parts
from shapely.geometry import Polygon
from shapely.strtree import STRtree


def build_map(mesh_path, config, output):
    started = time.monotonic()
    blocks, trajectory = read_mesh(mesh_path)
    if not blocks:
        raise ValueError('Empty mesh')
    engine = Engine(config, native_mesh=True)
    engine.index.update_blocks(blocks, config)
    low = np.min([v.min(axis=0) for v, _ in blocks.values()], axis=0) - config.border
    high = np.max([v.max(axis=0) for v, _ in blocks.values()], axis=0) + config.border
    records = engine.index.query_spans(low, high, config.resolution, config.vertical_resolution,
                                      math.cos(math.radians(80 if config.surface_normal_filter else config.max_slope_deg)),
                                      config.required_height)
    print(f'CLASSIFY {len(records)} spans across all floors', flush=True)
    masks, distances, reasons = engine.classifier.from_records(records, np.arange(len(records)))
    kept = np.flatnonzero(masks)
    states = engine.classifier.states[kept].copy()
    comfortable = engine.classifier.comfortable[kept].copy()
    proved = engine.classifier.proved[kept].copy() & masks[kept]
    print(f'POLYGONS {len(kept)} traversable spans', flush=True)
    groups = defaultdict(list)
    for i in kept:
        r = records[i]
        groups[(int(r['ix']) // config.tile_cells, int(r['iy']) // config.tile_cells,
                int(r['hi']) // config.slab_cells)].append(int(i))
    polygons = []
    for key, ids in sorted(groups.items()):
        spans = [Span(int(records[i]['ix']), int(records[i]['iy']), int(records[i]['hi']),
                      float(records[i]['hi']) * config.vertical_resolution,
                      math.inf if records[i]['ceiling'] == np.iinfo(np.int64).max
                      else float(records[i]['ceiling']) * config.vertical_resolution, True, True) for i in ids]
        polygons.extend(build_polygons(spans, masks[ids], distances[ids], config.resolution, key))
    print(f'PORTALS {len(polygons)} polygons', flush=True)
    shapes = [Polygon(np.asarray(p['vertices'])[:, :2]) for p in polygons]
    tree = STRtree(shapes)
    portals = []
    for i, a in enumerate(polygons):
        for raw_j in tree.query(shapes[i]):
            j = int(raw_j)
            if j <= i:
                continue
            b = polygons[j]
            if abs(a['floor'] - b['floor']) > config.max_step + 1e-8:
                continue
            if min(a['ceiling'], b['ceiling']) - max(a['floor'], b['floor']) < config.required_height - 1e-8:
                continue
            mask = int(a['yaw_mask']) & int(b['yaw_mask'])
            if not mask:
                continue
            for line in _line_parts(shapes[i].boundary.intersection(shapes[j].boundary)):
                xyz = [[float(x), float(y), max(a['floor'], b['floor'])] for x, y in [line.coords[0], line.coords[-1]]]
                portals.append(dict(polygons=[i, j], segment=xyz, yaw_mask=mask))
    cells = np.column_stack([records['ix'][kept], records['iy'][kept], records['hi'][kept]])
    by_cell = {tuple(map(int, cell)): int(mask) for cell, mask in zip(cells, proved)}
    for polygon in polygons:
        proof = (1 << config.yaw_bins)-1
        for cell in polygon['cells']:
            proof &= by_cell[tuple(cell)]
        polygon['proven_yaw_mask'] = proof
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    source = output / Path(mesh_path).name
    if source.resolve() != Path(mesh_path).resolve():
        shutil.copy2(mesh_path, source)
    np.savez_compressed(output / 'field.npz', cells=cells, masks=masks[kept],
                        comfortable=comfortable, proved=proved, states=states, trajectory=trajectory)
    for polygon in polygons:
        if not math.isfinite(polygon['ceiling']):
            polygon['ceiling'] = None
    report = dict(schema_version=1, frame='nvblox_odom', scope='static_complete_bag_mesh',
                  config=asdict(config), polygons=polygons, portals=portals,
                  mesh_sha256=hashlib.sha256(Path(mesh_path).read_bytes()).hexdigest(),
                  source_mesh=Path(mesh_path).name,
                  metrics=dict(spans=len(records), traversable_spans=len(kept),
                               polygons=len(polygons), portals=len(portals),
                               build_seconds=time.monotonic() - started),
                  method='SE(2) polygon portals + ASA, with physical-pose and swept-body verification')
    temporary = output / 'navmesh.json.tmp'
    temporary.write_text(json.dumps(report, separators=(',', ':'), allow_nan=False))
    temporary.replace(output / 'navmesh.json')
    print(json.dumps(report['metrics']), flush=True)
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('mesh', type=Path)
    p.add_argument('--output', type=Path)
    args = p.parse_args()
    output = args.output or args.mesh.parent
    output.mkdir(parents=True, exist_ok=True)
    config = Config(tile_cells=8, max_step=.20, max_slope_deg=40.,
                    surface_normal_filter=True, stair_riser_filter=True)
    # Preserve recorded physical dimensions; only stair analysis defaults differ.
    import yaml
    cfg = args.mesh.parent / 'terrain_config.yaml'
    if cfg.exists():
        d = yaml.safe_load(cfg.read_text())['se2_terrain_check']['ros__parameters']
        config = Config(**{**asdict(config), **{k: d[k] for k in asdict(config) if k in d}})
    build_map(args.mesh, config, output)


if __name__ == '__main__':
    main()
