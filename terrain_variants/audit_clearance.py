"""Frozen-Mesh ablations: separate clearance prefilter from full footprint checks."""
import argparse
from collections import Counter
from dataclasses import replace
import json
import math
from pathlib import Path
import sys
import time
import numpy as np

sys.path.insert(0,str(Path(__file__).parent/'local_window'))
from paper_pipeline import Config,Engine
from native_nav import Classifier,_lib,LABELS
from terrain import footprint_masks


def query(snapshot,position=None,config=None):
    meta=json.loads(snapshot.with_suffix('.json').read_text())
    pose=np.asarray(position if position is not None else meta['pose']['position'])
    saved=np.load(snapshot)
    blocks={tuple(k):(saved['v'+str(i)],saved['t'+str(i)]) for i,k in enumerate(saved['keys'])}
    c=config or Config(tile_cells=8,max_step=.2,max_slope_deg=40.,surface_normal_filter=True,stair_riser_filter=True)
    e=Engine(c,native_mesh=True);e.update(blocks)
    lower=np.floor((pose-[1.6,1.6,1.2])/c.slab_size).astype(np.int64)
    upper=np.floor((pose+[1.6,1.6,.6])/c.slab_size).astype(np.int64)
    low=lower*c.slab_size-c.border;high=(upper+1)*c.slab_size+c.border
    records=e.index.query_spans(low,high,c.resolution,c.vertical_resolution,
        math.cos(math.radians(80 if c.surface_normal_filter else c.max_slope_deg)),c.required_height)
    records=records[(records['ix']*c.resolution>=low[0])&(records['ix']*c.resolution<high[0])&
                    (records['iy']*c.resolution>=low[1])&(records['iy']*c.resolution<high[1])]
    keys=np.column_stack((records['ix']//c.tile_cells,records['iy']//c.tile_cells,records['hi']//c.slab_cells))
    roots=np.flatnonzero(np.all((keys>=lower)&(keys<=upper),axis=1))
    return c,pose,records,roots


def classify(records,roots,c,skip_distance=False):
    xy=np.column_stack((records['ix'],records['iy'])).astype(np.int64)
    z=records['hi'].astype(float)*c.vertical_resolution
    roof=records['ceiling'].astype(float)*c.vertical_resolution
    roof[records['ceiling']==np.iinfo(np.int64).max]=np.inf
    walk=records['walkable'].astype(np.uint8);slope=records['slope_ok'].astype(np.uint8)
    # Re-evaluate per-column headroom when testing top margin on identical Mesh.
    walk=(slope.astype(bool)&(roof-z>=c.required_height-1e-9)).astype(np.uint8)
    n=len(z);cf=Classifier(c,footprint_masks(c));roots=np.ascontiguousarray(roots,dtype=np.int64)
    if c.surface_normal_filter:
        _lib.support_from_patch(n,xy.ctypes.data,z.ctypes.data,roof.ctypes.data,walk.ctypes.data,slope.ctypes.data,
            c.max_step,c.required_height,c.resolution,c.max_slope_deg)
    if c.stair_riser_filter:
        _lib.promote_stair_risers(n,xy.ctypes.data,z.ctypes.data,roof.ctypes.data,walk.ctypes.data,slope.ctypes.data,
            c.max_step,c.required_height,c.resolution,c.max_slope_deg)
    masks=np.zeros(n,dtype=np.uint64);distances=np.zeros(n);reasons=np.zeros(n,dtype=np.int32)
    radius=0. if skip_distance else min(c.length/2+c.side_margin,c.width/2+c.side_margin)
    _lib.classify_selected(n,xy.ctypes.data,z.ctypes.data,roof.ctypes.data,walk.ctypes.data,slope.ctypes.data,
        c.max_step,c.required_height,c.resolution,radius,(1<<c.yaw_bins)-1,roots.ctypes.data,len(roots),
        *[a.ctypes.data for a in cf.buffers],cf.mask_count,masks.ctypes.data,distances.ctypes.data,reasons.ctypes.data)
    return masks,distances,reasons,walk,slope,roof


def audit(snapshot,position=None):
    c,pose,records,roots=query(snapshot,position)
    xy=(np.column_stack((records['ix'],records['iy']))+.5)*c.resolution
    z=records['hi']*c.vertical_resolution
    near=roots[(np.linalg.norm(xy[roots]-pose[:2],axis=1)<1.4)&(z[roots]>pose[2]-1.)&(z[roots]<pose[2]-.2)]
    result=dict(snapshot=str(snapshot),pose=pose.tolist(),near_roots=len(near),cases=[])
    base=None
    for name,cc,skip in [('current',c,False),('full_footprint_only',c,True),
        ('zero_side',replace(c,side_margin=0.),False),('zero_top',replace(c,top_margin=0.),False),
        ('zero_both',replace(c,side_margin=0.,top_margin=0.),False)]:
        begin=time.perf_counter();m,d,r,w,s,roof=classify(records,near,cc,skip)
        if base is None:base=m.copy()
        restored=near[(base[near]==0)&(m[near]!=0)]
        cells=[dict(xy=[int(records['ix'][i]),int(records['iy'][i])],z=float(z[i]),mask=int(m[i]),
                    distance=float(d[i]),reason=LABELS[r[i]],ceiling=float(roof[i]) if math.isfinite(roof[i]) else None)
               for i in near]
        result['cases'].append(dict(name=name,allowed=sum(bool(m[i]) for i in near),
            green=sum(bool(m[i]==(1<<40)-1) for i in near),restored=len(restored),
            reasons=dict(Counter(LABELS[r[i]] for i in near)),cells=cells,ms=(time.perf_counter()-begin)*1000))
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('snapshots',type=Path,nargs='+');p.add_argument('--position',type=float,nargs=3)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    for snapshot in a.snapshots:
        result=audit(snapshot,a.position);(a.output/(snapshot.stem+'.json')).write_text(json.dumps(result,indent=2))
        print(json.dumps(dict(snapshot=snapshot.stem,pose=result['pose'],near=result['near_roots'],
            cases=[{k:v for k,v in c.items() if k not in ('cells','reasons')} for c in result['cases']])),flush=True)
