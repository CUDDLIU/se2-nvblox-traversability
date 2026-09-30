"""Separate local-window eviction from rejection, then compare a fixed Mesh ROI."""
import argparse
import ctypes
from collections import Counter
import json
import math
from pathlib import Path
import sys
import numpy as np

sys.path.insert(0,str(Path(__file__).parent/'local_window'))
from paper_pipeline import Config, Engine
from native_nav import _lib, LABELS


def transitions(experiment):
    manifest=json.loads((experiment/'experiment.json').read_text())
    start=manifest['bag_start'];slabs={};counts=Counter();examples=[];pose=None
    for line in (experiment/'metrics.jsonl').open():
        row=json.loads(line);d=row['data'];clock=row['clock']
        if row['kind']=='odom':pose=d['position']
        if row['kind']!='local' or d.get('event')!='replace':continue
        for slab in d.get('updated_slabs',[]):
            key=tuple(slab['key']);old=slabs.get(key,[]);new=slab['cells']
            for previous in old:
                if not previous[3]:continue
                candidates=[r for r in new if r[:2]==previous[:2] and abs(r[2]-previous[2])<=12]
                if any(r[3] for r in candidates):continue
                kind='empty_replacement' if not new else 'rejected' if candidates else 'surface_missing'
                counts[kind]+=1
                if len(examples)<2000:examples.append(dict(t=clock-start if clock else None,
                    kind=kind,cell=previous,slab=list(key),pose=pose,candidates=candidates))
            slabs[key]=new
    return dict(counts=dict(counts),examples=examples)


def classify(path,position):
    p=np.asarray(position);saved=np.load(path)
    blocks={tuple(k):(saved['v'+str(i)],saved['t'+str(i)]) for i,k in enumerate(saved['keys'])}
    c=Config(tile_cells=8,max_step=.2,max_slope_deg=40.,surface_normal_filter=True,stair_riser_filter=True)
    e=Engine(c,native_mesh=True);e.update(blocks)
    core_low=np.floor((p-[1.6,1.6,1.2])/c.slab_size).astype(np.int64)
    core_high=np.floor((p+[1.6,1.6,.6])/c.slab_size).astype(np.int64)
    low=core_low*c.slab_size-c.border;high=(core_high+1)*c.slab_size+c.border
    records=e.index.query_spans(low,high,c.resolution,c.vertical_resolution,math.cos(math.radians(80)),c.required_height)
    records=records[(records['ix']*c.resolution>=low[0]) & (records['ix']*c.resolution<high[0]) &
                    (records['iy']*c.resolution>=low[1]) & (records['iy']*c.resolution<high[1])]
    keys=np.column_stack((records['ix']//c.tile_cells,records['iy']//c.tile_cells,records['hi']//c.slab_cells))
    roots=np.flatnonzero(np.all((keys>=core_low)&(keys<=core_high),axis=1))
    masks,distances,reasons=e.classifier.from_records(records,roots)
    xy=np.column_stack((records['ix'],records['iy'])).astype(np.int64)
    z=records['hi'].astype(float)*c.vertical_resolution
    roof=records['ceiling'].astype(float)*c.vertical_resolution
    roof[records['ceiling']==np.iinfo(np.int64).max]=np.inf
    walk=records['walkable'].astype(np.uint8);slope=records['slope_ok'].astype(np.uint8)
    n=len(records)
    _lib.support_from_patch(n,xy.ctypes.data,z.ctypes.data,roof.ctypes.data,walk.ctypes.data,slope.ctypes.data,
        c.max_step,c.required_height,c.resolution,c.max_slope_deg)
    patch=walk.copy()
    _lib.promote_stair_risers(n,xy.ctypes.data,z.ctypes.data,roof.ctypes.data,walk.ctypes.data,slope.ctypes.data,
        c.max_step,c.required_height,c.resolution,c.max_slope_deg)
    neighbors=np.zeros((n+1,4),dtype=np.int64);missing=np.zeros((n+1,4),dtype=np.uint8)
    codes=np.zeros(n,dtype=np.int32);components=np.zeros(n,dtype=np.int64)
    _lib.span_neighbors.argtypes=[ctypes.c_size_t]+[ctypes.c_void_p]*5+[ctypes.c_double]*2+[ctypes.c_void_p]*4
    _lib.span_neighbors(n,xy.ctypes.data,z.ctypes.data,roof.ctypes.data,walk.ctypes.data,slope.ctypes.data,
        c.max_step,c.required_height,neighbors.ctypes.data,missing.ctypes.data,codes.ctypes.data,components.ctypes.data)
    invalid=(~walk.astype(bool))|np.any(neighbors[:n]==n,axis=1)
    cells=[dict(xy=xy[i].tolist(),z=float(z[i]),mask=int(masks[i]),reason=reasons[i],
        ceiling=float(roof[i]) if math.isfinite(roof[i]) else None,raw_slope=bool(records['slope_ok'][i]),
        patch_walk=bool(patch[i]),walk=bool(walk[i]),distance=float(distances[i])) for i in roots]
    for i,cell in zip(roots,cells):
        if reasons[i]!='footprint_or_boundary_distance' or distances[i]>.334:continue
        seeds=np.flatnonzero(invalid & (components==components[i]))
        if not len(seeds):continue
        seed=seeds[np.argmin(np.linalg.norm(xy[seeds]-xy[i],axis=1))]
        cell['nearest_boundary']=dict(xy=xy[seed].tolist(),z=float(z[seed]),reason=LABELS[codes[seed]])
    return dict(snapshot=path.name,pose=position,cells=cells,counts=dict(Counter(x['reason'] for x in cells)),
                allowed=sum(bool(x['mask']) for x in cells),green=sum(x['mask']==(1<<40)-1 for x in cells))


def compare(old,new):
    columns={}
    for row in new['cells']:columns.setdefault(tuple(row['xy']),[]).append(row)
    lost=[];kept=0
    for row in old['cells']:
        if not row['mask']:continue
        near=[x for x in columns.get(tuple(row['xy']),[]) if abs(x['z']-row['z'])<=.12]
        if any(x['mask'] for x in near):kept+=1
        else:lost.append(dict(before=row,after=near))
    return dict(before=old['snapshot'],after=new['snapshot'],kept=kept,lost=len(lost),
                reasons=dict(Counter(x['after'][0]['reason'] if x['after'] else 'surface_missing' for x in lost)),examples=lost)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('experiment',type=Path)
    p.add_argument('--reference',default='mesh_100');p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'transitions.json').write_text(json.dumps(transitions(args.experiment),indent=2))
    trace=args.experiment/'mesh_trace'
    position=json.loads((trace/(args.reference+'.json')).read_text())['pose']['position']
    results=[]
    for path in sorted(trace.glob('mesh_*.npz')):
        result=classify(path,position);results.append(result)
        (args.output/(path.stem+'.json')).write_text(json.dumps(result,indent=2))
        print(json.dumps({k:v for k,v in result.items() if k!='cells'}),flush=True)
    comparisons=[compare(a,b) for a,b in zip(results,results[1:])]
    (args.output/'comparison.json').write_text(json.dumps(comparisons,indent=2))
