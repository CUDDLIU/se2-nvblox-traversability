"""Replay a captured scene without ROS or robot outputs."""
import argparse
import json
import numpy as np
from terrain import (Config, Terrain, footprint_masks, evaluate, required_free_keys,
                     certify_free, observed_clearance_tops)
from fast_raster import rasterize


def replay(path, config):
    with np.load(path) as archive:
        scene = {key:archive[key] for key in archive.files}
    terrain = Terrain(config)
    for key, (v0,v1,t0,t1) in zip(scene['keys'],scene['bounds']):
        terrain.replace_block(tuple(key), rasterize(scene['vertices'][v0:v1],
                              scene['triangles'][t0:t1]-v0, config))
    terrain.rebuild()
    surfaces = terrain.local_surfaces(*scene['odom'][:2], 4.8)
    evidence = {}
    keys = required_free_keys(surfaces,config)
    k = scene['k']
    for depth, camera_to_world, stamp in zip(scene['depth'],scene['camera_to_world'],scene['stamps']):
        for start in range(0,len(keys),12000):
            chunk = keys[start:start+12000]
            free = certify_free(chunk,depth,(k[0],k[4],k[2],k[5]),np.linalg.inv(camera_to_world),config)
            evidence.update((tuple(key),stamp) for key in chunk[free])
    tops = observed_clearance_tops(surfaces,evidence,config,scene['stamps'][-1],5.)
    diagnostics = {}
    geometry, verified = evaluate(surfaces, config, footprint_masks(config),tops,diagnostics)
    result = dict(spans=len(surfaces), covered=sum(s.covered for s in surfaces),
                  any_yaw_geometry_pass=int(np.count_nonzero(geometry)),
                  selected_yaw_geometry_pass=int(np.count_nonzero(geometry & 1)),
                  any_yaw_observed_pass=int(np.count_nonzero(verified)),
                  selected_yaw_unknown_support=int(np.count_nonzero(diagnostics['unknown_support'] & 1)),
                  free_voxels_last_5s=int(sum(scene['stamps'][-1]-t<=5 for t in evidence.values())),
                  columns_observed_1m=int(np.count_nonzero(tops-np.array([s.z for s in surfaces])>=1.)))
    print(json.dumps(result))
    return terrain, surfaces


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('scene')
    args = parser.parse_args()
    replay(args.scene, Config())
