"""Offline explanation of missing clearance evidence; never changes parameters."""
import argparse
import json
import math
from collections import defaultdict
import numpy as np
from terrain import (Config,Terrain,required_free_keys,certify_free,
                     observed_clearance_tops,footprint_masks,evaluate)
from fast_raster import rasterize_blocks


def diagnose(path):
    with np.load(path) as f:
        d={k:f[k] for k in f.files}
    c=Config()
    terrain=Terrain(c)
    blocks={tuple(key):(d['vertices'][v0:v1],d['triangles'][t0:t1]-v0)
            for key,(v0,v1,t0,t1) in zip(d['keys'],d['bounds'])}
    for key,records in rasterize_blocks(blocks,c).items():
        terrain.replace_block(key,records)
    terrain.rebuild()
    surfaces=terrain.local_surfaces(*d['odom'][:2],4.8)
    keys=required_free_keys(surfaces,c)
    k=d['k']; intr=(k[0],k[4],k[2],k[5]); now=d['stamps'][-1]
    evidence={}
    for depth,tf,stamp in zip(d['depth'],d['camera_to_world'],d['stamps']):
        if now-stamp>5:
            continue
        for start in range(0,len(keys),12000):
            chunk=keys[start:start+12000]
            free=certify_free(chunk,depth,intr,np.linalg.inv(tf),c)
            evidence.update((tuple(key),stamp) for key in chunk[free])
    tops=observed_clearance_tops(surfaces,evidence,c,now,5)
    masks=footprint_masks(c)
    geometry,verified=evaluate(surfaces,c,masks,tops)
    by=defaultdict(list)
    for i,s in enumerate(surfaces):
        by[s.ix,s.iy].append(i)
    best=None
    for root in np.flatnonzero(geometry):
        for yaw,(order,parents,_,_) in enumerate(masks):
            if not int(geometry[root])&(1<<yaw):
                continue
            mapped=[root]
            for j in range(1,len(order)):
                x,y=order[j]; p=surfaces[root]; a=surfaces[mapped[parents[j]]]
                choices=[i for i in by[p.ix+x,p.iy+y] if surfaces[i].covered and
                         abs(surfaces[i].z-a.z)<=c.max_step+1e-9]
                assert len(choices)==1
                mapped.append(choices[0])
            target=max(surfaces[i].z for i in mapped)+c.height+c.top_margin
            good=sum(tops[i]>=target for i in mapped)
            if best is None or good/len(mapped)>best[0]:
                best=(good/len(mapped),int(good),len(mapped),yaw,target,mapped)
    result=dict(spans=len(surfaces),geometry_any=int(np.count_nonzero(geometry)),
                observed_any=int(np.count_nonzero(verified)),free_voxels=len(evidence))
    if best is None:
        return result
    _,good,total,yaw,target,mapped=best
    result['best_footprint']=dict(clear_columns=good,total_columns=total,yaw_deg=yaw*9)
    missing=[]
    for i in mapped:
        p=surfaces[i]
        for iz in range(math.ceil((p.z+c.ground_skin)/.05-1e-9),math.ceil(target/.05)):
            key=(p.ix,p.iy,iz)
            if key not in evidence:
                missing.append(key)
    if not missing:
        return result
    keys=np.asarray(missing)
    corners=np.array([(x,y,z) for x in (0,1) for y in (0,1) for z in (0,1)])
    world=(keys[:,None,:]+corners[None,:,:])*[.1,.1,.05]
    tf=np.linalg.inv(d['camera_to_world'][-1])
    cam=world@tf[:3,:3].T+tf[:3,3]; zs=cam[:,:,2]
    uv=cam[:,:,:2]/np.maximum(zs[:,:,None],1e-8)*[intr[0],intr[1]]+[intr[2],intr[3]]
    lo=np.floor(uv.min(axis=1)).astype(int); hi=np.ceil(uv.max(axis=1)).astype(int)
    depth=d['depth'][-1]; h,w=depth.shape
    valid=(zs.min(axis=1)>.1)&(zs.max(axis=1)<=4)&(lo[:,0]>=0)&(lo[:,1]>=0)&(hi[:,0]<w)&(hi[:,1]<h)
    invalid=near=clear=0
    for j in np.flatnonzero(valid):
        patch=depth[lo[j,1]:hi[j,1]+1,lo[j,0]:hi[j,0]+1]
        if np.any(~np.isfinite(patch)|(patch<=0)):
            invalid+=1
        elif np.min(patch)<=zs[j].max()+.04:
            near+=1
        else:
            clear+=1
    result['remaining_voxels_latest_frame']=dict(total=len(keys),
        outside_view_or_range=int((~valid).sum()),invalid_depth=invalid,
        depth_or_margin=near,unexpected_clear=clear)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('scene')
    print(json.dumps(diagnose(parser.parse_args().scene)))
