"""Bounded, offline timing of an existing mesh; never starts ROS or devices."""
import argparse
from collections import defaultdict
import json
import platform
import time
from unittest.mock import patch

import paper_pipeline as pipeline
import paper_polygons as polygons
from replay_paper import load_mesh, json_safe


def run(path):
    _, triangles, blocks = load_mesh(path)
    e = pipeline.Engine()
    times = defaultdict(float)
    counts = defaultdict(int)

    def timed(name, fn):
        def inner(*args, **kwargs):
            before = time.perf_counter()
            result = fn(*args, **kwargs)
            times[name] += time.perf_counter()-before
            counts[name] += 1
            return result
        return inner

    e.index.replace = timed('block_bvh_update', e.index.replace)
    e.index.query = timed('bvh_mesh_query', e.index.query)
    e.index.has_mesh = timed('bvh_empty_test', e.index.has_mesh)
    ticks = []
    started = time.perf_counter()
    update_start = time.perf_counter()
    update = e.update(blocks, clear=True)
    update_ms = (time.perf_counter()-update_start)*1000
    with patch.object(pipeline, 'voxelize', timed('voxelize', pipeline.voxelize)), \
         patch.object(pipeline, 'classify', timed('classify', pipeline.classify)), \
         patch.object(polygons, 'build_polygons', timed('build_polygons', polygons.build_polygons)):
        while e.pending:
            ticks.append(e.process(max_slabs=4).copy())
    before = time.perf_counter()
    snapshot = e.snapshot()
    graph_ms = (time.perf_counter()-before)*1000
    before = time.perf_counter()
    data = json.dumps(json_safe(snapshot), separators=(',', ':'), allow_nan=False)
    serialization_ms = (time.perf_counter()-before)*1000
    return dict(machine=platform.machine(), input_blocks=len(blocks), input_triangles=len(triangles),
                update_ms=round(update_ms, 2), stages_ms={k:round(v*1000,2) for k,v in times.items()},
                stage_calls=dict(counts), graph_ms=round(graph_ms,2), json_ms=round(serialization_ms,2),
                offline_total_ms=round((time.perf_counter()-started)*1000,2),
                callbacks=len(ticks), ticks=ticks, initial_pending=update['pending_slabs'],
                polygons=len(snapshot['polygons']), graph_nodes=len(snapshot['graph']['nodes']),
                graph_edges=len(snapshot['graph']['edges']), json_bytes=len(data))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('scene')
    p.add_argument('--repeat', type=int, default=2)
    a=p.parse_args()
    for _ in range(a.repeat):
        print(json.dumps(run(a.scene)), flush=True)


if __name__ == '__main__':
    main()
