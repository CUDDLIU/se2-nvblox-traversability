"""Diagnostic multi-frame point-to-plane extrinsic check with held-out frames.

Outputs a candidate correction and evidence, never changes the installed TF.
Residuals include sensor noise, time/pose errors and nonoverlapping surfaces.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

def voxel_indices(points,resolution):
    _,indices=np.unique(np.floor(points/resolution).astype(np.int32),axis=0,return_index=True)
    return indices

def load(path):
    a=np.load(path)
    body=a['lidar_body'];keep=~((np.abs(body[:,0])<=.49)&(np.abs(body[:,1])<=.333)&(body[:,2]>=-.60)&(body[:,2]<=.50))
    xyz=a['lidar_world'][keep];xyz=xyz[voxel_indices(xyz,.035)]
    tree=cKDTree(xyz);_,neighbors=tree.query(xyz,k=16)
    centered=xyz[neighbors]-xyz[neighbors].mean(axis=1,keepdims=True)
    cov=np.einsum('nki,nkj->nij',centered,centered)/16
    values,vectors=np.linalg.eigh(cov)
    planar=(values[:,0]/np.maximum(values.sum(axis=1),1e-12)<.025)&(values[:,1]>.000025)
    t=a['T_base_camera'];camera=a['depth_camera']
    points=camera@t[:3,:3].T+t[:3,3];points=points[voxel_indices(points,.035)]
    return dict(name=path.stem,xyz=xyz,tree=tree,normal=vectors[:,:,0],planar=planar,
                points=points,r=a['R_world_base'],t=a['t_world_base'])

def pairs(frame,transform):
    body=frame['points']@transform[:3,:3].T+transform[:3,3]
    world=body@frame['r'].T+frame['t']
    distance,index=frame['tree'].query(world)
    keep=(distance<.20)&frame['planar'][index]
    n=frame['normal'][index[keep]]
    residual=np.einsum('ij,ij->i',n,world[keep]-frame['xyz'][index[keep]])
    nb=n@frame['r'];jacobian=np.column_stack((np.cross(body[keep],nb),nb))
    return jacobian,residual,distance

def metrics(frames,transform):
    result=[]
    for f in frames:
        j,r,d=pairs(f,transform)
        result.append(dict(frame=f['name'],points=len(d),plane_pairs=len(r),
            plane_abs_p50_mm=float(np.median(np.abs(r))*1000),
            plane_abs_p90_mm=float(np.percentile(np.abs(r),90)*1000),
            nearest_p50_mm=float(np.median(d)*1000),within_5cm=float(np.mean(d<.05))))
    return result

def main():
    p=argparse.ArgumentParser();p.add_argument('directory',type=Path);args=p.parse_args()
    frames=[load(f) for f in sorted(args.directory.glob('sensors_*.npz'))]
    if len(frames)<4:raise ValueError('Need at least four frames for independent held-out check')
    train=frames[::2];test=frames[1::2];transform=np.eye(4);trace=[]
    for iteration in range(20):
        pieces=[pairs(f,transform)[:2] for f in train]
        j=np.concatenate([x[0] for x in pieces]);r=np.concatenate([x[1] for x in pieces])
        w=np.sqrt(np.minimum(1.,.02/np.maximum(np.abs(r),1e-9)))
        update,_,_,s=np.linalg.lstsq(j*w[:,None],-r*w,rcond=None)
        # Bound each relinearization, not the fitted total correction.
        update[:3]*=min(1.,.025/max(np.linalg.norm(update[:3]),1e-12))
        update[3:]*=min(1.,.025/max(np.linalg.norm(update[3:]),1e-12))
        delta=np.eye(4);delta[:3,:3]=Rotation.from_rotvec(update[:3]).as_matrix();delta[:3,3]=update[3:]
        transform=delta@transform
        trace.append(dict(iteration=iteration,pairs=len(r),median_abs_mm=float(np.median(np.abs(r))*1000),
                          update=update.tolist(),singular_values=s.tolist()))
        if np.linalg.norm(update)<1e-6:break
    result=dict(scope='diagnostic only; no TF modification',training=[f['name'] for f in train],held_out=[f['name'] for f in test],
        candidate_delta_base=transform.tolist(),translation_m=transform[:3,3].tolist(),
        rotation_degrees=Rotation.from_matrix(transform[:3,:3]).as_rotvec().tolist(),
        baseline=metrics(frames,np.eye(4)),candidate=metrics(frames,transform),iterations=trace)
    result['rotation_degrees']=(Rotation.from_matrix(transform[:3,:3]).as_rotvec()*180/np.pi).tolist()
    (args.directory/'alignment_check.json').write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k!='iterations'},indent=2))

if __name__=='__main__':main()
