"""Diagnostic only: test a common camera-to-pose time offset on held-out frames.

Keeps the baseline point-to-plane correspondences fixed so a bad offset cannot
improve a score simply by discarding previously matched points. No TF, bag or
mapper settings are modified. Occlusion and calibration remain confounders.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from check_alignment import load


def main():
    p=argparse.ArgumentParser();p.add_argument('directory',type=Path);args=p.parse_args()
    frames=[]
    for path in sorted(args.directory.glob('sensors_*.npz')):
        f=load(path);a=np.load(path);meta=json.loads(path.with_suffix('.json').read_text())
        world=f['points']@f['r'].T+f['t'];distance,index=f['tree'].query(world)
        keep=(distance<.20)&f['planar'][index]
        f.update(points=f['points'][keep],target=f['xyz'][index[keep]],
            normals=f['normal'][index[keep]],stamp=meta['depth_stamp'],
            times=a['pose_times'],positions=a['pose_positions'],
            slerp=Slerp(a['pose_times'],Rotation.from_quat(a['pose_quaternions'])))
        frames.append(f)
    offsets=np.arange(-.25,.2501,.005)
    scores=[]
    for offset in offsets:
        row=[]
        for f in frames:
            t=f['stamp']+offset
            if not f['times'][0]<=t<=f['times'][-1]:raise ValueError('pose context too short')
            r=f['slerp'](t).as_matrix()
            v=np.array([np.interp(t,f['times'],f['positions'][:,j]) for j in range(3)])
            world=f['points']@r.T+v
            residual=np.einsum('ij,ij->i',f['normals'],world-f['target'])
            row.append(float(np.median(abs(residual))))
        scores.append(row)
    scores=np.asarray(scores);baseline=np.argmin(abs(offsets))
    train=np.arange(0,len(frames),2);test=np.arange(1,len(frames),2)
    best=np.argmin(scores[:,train].mean(axis=1))
    result=dict(scope=__doc__,training=[frames[i]['name'] for i in train],
        held_out=[frames[i]['name'] for i in test],candidate_offset_s=float(offsets[best]),
        held_out_baseline_m=float(scores[baseline,test].mean()),
        held_out_candidate_m=float(scores[best,test].mean()),
        frames=[dict(name=f['name'],fixed_pairs=len(f['points']),
            baseline_m=float(scores[baseline,i]),candidate_m=float(scores[best,i]),
            individually_best_offset_s=float(offsets[np.argmin(scores[:,i])])) for i,f in enumerate(frames)],
        sweep_offsets_s=offsets.tolist(),sweep_median_plane_residuals_m=scores.tolist())
    result['held_out_improves']=result['held_out_candidate_m']<result['held_out_baseline_m']
    out=args.directory/'time_alignment_check.json';out.write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items() if not k.startswith('sweep_')},indent=2))


if __name__=='__main__':main()
