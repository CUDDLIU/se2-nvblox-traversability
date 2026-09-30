"""Replay captured nvblox meshes through the paper-guided pipeline, without ROS.

Example: PYTHONPATH=/tmp/se2-terrain-deps python3 replay_paper.py \
    scene_current_no_green.npz --output-dir /tmp/se2-paper-replay --plot

Only captured mesh arrays are consumed. Depth frames and legacy support
diagnostics deliberately do not participate in navigation classification.
"""
import argparse
from collections import Counter
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

from paper_pipeline import Config, Engine


def json_safe(value):
    """Represent unbounded ceilings as JSON null; never emit NaN/Infinity."""
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, np.ndarray)):
        return [json_safe(item) for item in value]
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(value) else None
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.bool_):
        return bool(value)
    return value


def load_mesh(path):
    with np.load(path, allow_pickle=False) as archive:
        vertices = np.asarray(archive['vertices'], dtype=np.float64)
        triangles = np.asarray(archive['triangles'], dtype=np.int64)
        keys, bounds = archive['keys'], archive['bounds']
        if len(keys) != len(bounds):
            raise ValueError('Mesh keys/bounds length mismatch')
        blocks = {}
        for key, (v0, v1, t0, t1) in zip(keys, bounds):
            key = tuple(map(int, key))
            if key in blocks:
                raise ValueError(f'Duplicate mesh block {key}')
            if not 0 <= v0 <= v1 <= len(vertices) or not 0 <= t0 <= t1 <= len(triangles):
                raise ValueError(f'Invalid mesh array bounds for block {key}')
            blocks[key] = (vertices[v0:v1], triangles[t0:t1] - v0)
    return vertices, triangles, blocks


def plot_scene(vertices, triangles, snapshot, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection

    fig, ax = plt.subplots(figsize=(11, 9), constrained_layout=True)
    mesh = vertices[triangles, :2]
    ax.add_collection(PolyCollection(mesh, facecolors='#dce1e6', edgecolors='none', alpha=.4))
    full_mask = (1 << snapshot['config']['yaw_bins']) - 1
    colors, outlines = [], []
    for polygon in snapshot['polygons']:
        outlines.append(np.asarray(polygon['vertices'])[:, :2])
        colors.append('#26ad5f' if polygon['yaw_mask'] == full_mask else '#eaa22e')
    if outlines:
        ax.add_collection(PolyCollection(outlines, facecolors=colors, edgecolors='#36434a',
                                         linewidths=.35, alpha=.85))
    ax.autoscale_view()
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('Paper-guided SE(2) mesh replay: green = all yaw, orange = some yaw\n'
                 'XY projection of all height layers; gray = input mesh')
    fig.savefig(output, dpi=170)
    plt.close(fig)


def replay(path, config, batch_slabs=4, quiet=False):
    started = time.perf_counter()
    vertices, triangles, blocks = load_mesh(path)
    engine = Engine(config)
    update = engine.update(blocks, clear=True)
    initial_pending = len(engine.pending)
    queried_triangles, process_ms = 0, []
    while engine.pending:
        metrics = engine.process(max_slabs=batch_slabs)
        queried_triangles += metrics['queried_triangles']
        process_ms.append(metrics['process_ms'])
        if not quiet:
            print(json.dumps({'scene': Path(path).name, **metrics}), file=sys.stderr, flush=True)
    processing_seconds = time.perf_counter() - started
    snapshot_started = time.perf_counter()
    snapshot = engine.snapshot()
    snapshot_seconds = time.perf_counter() - snapshot_started
    all_masks = [int(mask) for result in engine.results.values() for mask in result.masks]
    reasons = Counter(reason for result in engine.results.values() for reason in result.reasons)
    yaw_counts = Counter(mask.bit_count() for mask in all_masks)
    full_mask = (1 << config.yaw_bins) - 1
    summary = {
        'scene': str(Path(path).resolve()),
        'input_blocks': len(blocks), 'input_vertices': len(vertices),
        'input_triangles': len(triangles), 'initial_pending_slabs': initial_pending,
        'populated_slabs': len(engine.results), 'spans': len(all_masks),
        'any_yaw_spans': sum(mask != 0 for mask in all_masks),
        'all_yaw_spans': sum(mask == full_mask for mask in all_masks),
        'restricted_spans': sum(0 < mask < full_mask for mask in all_masks),
        'reason_counts': dict(sorted(reasons.items())),
        'yaw_count_histogram': dict(sorted(yaw_counts.items())),
        'polygons': len(snapshot['polygons']),
        'graph_nodes': len(snapshot['graph']['nodes']),
        'graph_edges': len(snapshot['graph']['edges']),
        'processing_seconds': round(processing_seconds, 3),
        'snapshot_seconds': round(snapshot_seconds, 3),
        'total_seconds': round(time.perf_counter() - started, 3),
        'batch_ms_max': max(process_ms, default=0),
        'queried_triangles_total': queried_triangles,
        'config': asdict(config), 'update': update,
    }
    return summary, snapshot, vertices, triangles


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scenes', nargs='*', help='NPZ captures; default: scene*.npz beside this script')
    parser.add_argument('--batch-slabs', type=int, default=4)
    parser.add_argument('--output-dir', type=Path, help='Write strict JSON summary and complete graph per scene')
    parser.add_argument('--plot', action='store_true', help='Also save a static XY mesh/polygon PNG')
    parser.add_argument('--quiet', action='store_true', help='Suppress incremental progress on stderr')
    args = parser.parse_args()
    if args.batch_slabs < 1:
        parser.error('--batch-slabs must be positive')
    if args.plot and args.output_dir is None:
        parser.error('--plot requires --output-dir')
    scenes = [Path(p) for p in args.scenes] or sorted(Path(__file__).parent.glob('scene*.npz'))
    if not scenes:
        parser.error('No scenes found')
    if args.output_dir:
        args.output_dir.mkdir(parents=True, exist_ok=True)
    for path in scenes:
        summary, snapshot, vertices, triangles = replay(path, Config(), args.batch_slabs, args.quiet)
        print(json.dumps(json_safe(summary), allow_nan=False), flush=True)
        if args.output_dir:
            payload = {'summary': summary, 'snapshot': snapshot}
            destination = args.output_dir / f'{path.stem}.json'
            destination.write_text(json.dumps(json_safe(payload), allow_nan=False, indent=2) + '\n')
            if args.plot:
                plot_scene(vertices, triangles, snapshot, args.output_dir / f'{path.stem}.png')


if __name__ == '__main__':
    main()
