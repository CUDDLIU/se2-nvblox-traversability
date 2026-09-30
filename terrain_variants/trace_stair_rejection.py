"""Expose the measured column/footprint reason at a real Mesh route gap."""
import argparse
import ctypes
import json
import math
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).parent/'local_window'))
from paper_pipeline import Config,Engine
from native_nav import _lib,LABELS


def trace(snapshot,position=None,margin=.08,step=.20,slope_deg=40.):
    meta=json.loads(snapshot.with_suffix('.json').read_text());p=np.array(position or meta['pose']['position'])
    saved=np.load(snapshot);blocks={tuple(k):(saved['v'+str(i)],saved['t'+str(i)]) for i,k in enumerate(saved['keys'])}
    c=Config(tile_cells=8,max_step=step,max_slope_deg=slope_deg,side_margin=margin,
             surface_normal_filter=True,stair_riser_filter=True)
    e=Engine(c,native_mesh=True);e.update(blocks)
    low=p-np.array([2.4,2.4,4.8]);high=p+np.array([2.4,2.4,4.8])
    records=e.index.query_spans(low,high,c.resolution,c.vertical_resolution,math.cos(math.radians(80)),c.required_height)
    xy=np.column_stack((records['ix'],records['iy'])).astype(np.int64)
    z=records['hi'].astype(float)*c.vertical_resolution
    roof=records['ceiling'].astype(float)*c.vertical_resolution
    roof[records['ceiling']==np.iinfo(np.int64).max]=np.inf
    walk=records['walkable'].astype(np.uint8);slope=records['slope_ok'].astype(np.uint8)
    raw_walk=walk.copy();raw_slope=slope.copy();n=len(z)
    _lib.support_from_patch(n,xy.ctypes.data,z.ctypes.data,roof.ctypes.data,walk.ctypes.data,slope.ctypes.data,
                            c.max_step,c.required_height,c.resolution,c.max_slope_deg)
    patch_walk=walk.copy()
    _lib.promote_stair_risers(n,xy.ctypes.data,z.ctypes.data,roof.ctypes.data,walk.ctypes.data,slope.ctypes.data,
                            c.max_step,c.required_height,c.resolution,c.max_slope_deg)
    neighbors=np.zeros((n+1,4),dtype=np.int64);missing=np.zeros((n+1,4),dtype=np.uint8)
    reason=np.zeros(n,dtype=np.int32);labels=np.zeros(n,dtype=np.int64)
    _lib.span_neighbors.argtypes=[ctypes.c_size_t]+[ctypes.c_void_p]*5+[ctypes.c_double]*2+[ctypes.c_void_p]*4
    rc=_lib.span_neighbors(n,xy.ctypes.data,z.ctypes.data,roof.ctypes.data,walk.ctypes.data,slope.ctypes.data,
                          c.max_step,c.required_height,neighbors.ctypes.data,missing.ctypes.data,reason.ctypes.data,labels.ctypes.data)
    assert rc==0
    xyz=np.column_stack(((xy+.5)*c.resolution,z));ground=p-[0,0,.565]
    roots=np.argsort(np.linalg.norm(xyz-ground,axis=1))[:25]
    geometry,distances,final_reasons=e.classifier.from_records(records,roots)
    def cell(i):
        return dict(index=int(i),xy=xy[i].tolist(),z=float(z[i]),ceiling=float(roof[i]) if math.isfinite(roof[i]) else None,
            raw_slope=bool(raw_slope[i]),raw_walk=bool(raw_walk[i]),patch_walk=bool(patch_walk[i]),
            walk=bool(walk[i]),neighbor_reason=LABELS[reason[i]])
    trials=[]
    invalid=(~walk.astype(bool))|np.any(neighbors[:n]==n,axis=1)
    for root in roots[:8]:
        trial=cell(root);trial.update(mask=int(geometry[root]),distance=float(distances[root]),reason=final_reasons[root])
        seeds=np.flatnonzero(invalid&(labels==labels[root]))
        nearest=sorted(seeds,key=lambda i:np.linalg.norm(xy[i]-xy[root]))[:4]
        trial['near_invalid']=[cell(i) for i in nearest]
        failures=[]
        for yaw,(offsets,parents,directions,checks) in enumerate(e.masks):
            mapped=[int(root)];fault=None
            for j in range(1,len(parents)):
                prev=mapped[int(parents[j])];q=int(neighbors[prev,int(directions[j])])
                if q==n:
                    dx,dy=((1,0),(-1,0),(0,1),(0,-1))[int(directions[j])]
                    candidates=np.flatnonzero((xy[:,0]==xy[prev,0]+dx)&(xy[:,1]==xy[prev,1]+dy))
                    fault=dict(at=cell(prev),direction=int(directions[j]),candidates=[cell(i) for i in candidates]);break
                mapped.append(q)
            if fault:failures.append(dict(yaw=yaw,fault=fault))
        trial['footprint_failures']=failures[::max(1,len(failures)//4)]
        trials.append(trial)
    near=(np.linalg.norm(xyz[:,:2]-p[:2],axis=1)<1.8)&(z>p[2]-1.5)&(z<p[2]+.8)
    columns=[cell(i) for i in np.flatnonzero(near)]
    result=dict(snapshot=str(snapshot),position=p.tolist(),margin=margin,step=step,slope=slope_deg,
                nearest_roots=trials,columns=columns)
    return result


if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('snapshot',type=Path);a.add_argument('--position',type=float,nargs=3)
    a.add_argument('--margin',type=float,default=.08);a.add_argument('--step',type=float,default=.2)
    a.add_argument('--slope',type=float,default=40.);a.add_argument('--output',type=Path,required=True)
    args=a.parse_args();result=trace(args.snapshot,args.position,args.margin,args.step,args.slope)
    args.output.write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='columns'},indent=2))
