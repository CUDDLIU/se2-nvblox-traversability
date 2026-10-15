"""Whole-map cold start with coarse distance lookup and independent validation."""
import json
import math
from pathlib import Path
import numpy as np
from scipy.ndimage import distance_transform_edt,map_coordinates
from scipy.spatial.transform import Rotation
from alignment import rotation


def map_seeds(directory):
    directory=Path(directory);doc=json.loads((directory/'navmesh.json').read_text())
    config=doc['config'];r=config['resolution'];rz=config['vertical_resolution']
    cells=np.unique(np.asarray([c for p in doc['polygons'] for c in p['cells']]),axis=0)
    # A coarse body-height hypothesis only; full 3D registration estimates Z.
    points=cells*[r,r,rz]+[r/2,r/2,.55]
    with np.load(directory/doc['source_mesh'],allow_pickle=False) as mesh:
        points=np.vstack([points,mesh['trajectory'][:,1:4]])
    points=points[np.isfinite(points).all(axis=1)]
    _,indices=np.unique(np.floor(points/.5).astype(np.int64),axis=0,return_index=True)
    return points[indices]


class GlobalInitialSearch:
    def __init__(self,target,seeds,radius=8.,candidate_limit=64):
        self.seeds=np.asarray(seeds,dtype=float)
        if not len(self.seeds):raise ValueError('map has no initial localization seeds')
        self.radius=radius;self.candidate_limit=candidate_limit
        lower=self.seeds.min(0)-radius-.5;upper=self.seeds.max(0)+radius+.5
        self.resolution=.2
        while np.prod(np.ceil((upper-lower)/self.resolution)+3)>8_000_000:self.resolution*=1.1
        self.origin=np.floor(lower/self.resolution)*self.resolution-self.resolution
        shape=np.ceil((upper-self.origin)/self.resolution).astype(int)+2
        selected=target[np.all((target>=lower)&(target<=upper),axis=1)]
        if not len(selected):raise ValueError('localization reference does not cover map seeds')
        index=np.floor((selected-self.origin)/self.resolution).astype(int)
        empty=np.ones(shape,dtype=bool);empty[tuple(index.T)]=False
        self.field=np.minimum(distance_transform_edt(empty,sampling=self.resolution),.3).astype(np.float32)

    def candidates(self,train,body):
        coarse=train[np.linspace(0,len(train)-1,min(len(train),256)).astype(int)]-body
        candidates=[]
        for yaw in np.arange(-math.pi,math.pi,math.pi/24):
            rotated=coarse@rotation(yaw).T
            for start in range(0,len(self.seeds),64):
                positions=self.seeds[start:start+64]
                points=positions[:,None,:]+rotated[None,:,:]
                coordinates=((points-self.origin)/self.resolution-.5).reshape(-1,3).T
                distance=map_coordinates(self.field,coordinates,order=1,mode='constant',cval=.3)
                scores=distance.reshape(len(positions),-1).mean(1)
                candidates.extend((float(score),p.copy(),float(yaw)) for score,p in zip(scores,positions))
        candidates.sort(key=lambda item:item[0]);selected=[]
        for item in candidates:
            _,point,yaw=item
            if any(np.linalg.norm(point-other[1])<.8 and
                   abs(math.atan2(math.sin(yaw-other[2]),math.cos(yaw-other[2])))<.4 for other in selected):continue
            selected.append(item)
            if len(selected)>=self.candidate_limit:break
        return selected

    @staticmethod
    def observability(aligner,test,t,r):
        points=test@r.T+t;distance,index=aligner.tree.query(points,workers=1)
        normals=aligner.normals[index]
        residual=np.abs(np.sum((points-aligner.target[index])*normals,axis=1))
        good=(distance<.3)&(residual<.08)
        if good.sum()<200:return 0.
        points=points[good];normals=normals[good]
        # Normalize angular columns by scene radius for comparable conditioning.
        scale=max(1.,float(np.sqrt(np.mean(np.sum((points-points.mean(0))**2,axis=1)))))
        jacobian=np.column_stack([normals,np.cross(points-points.mean(0),normals)/scale])
        eigen=np.linalg.eigvalsh(jacobian.T@jacobian/len(points))
        return float(eigen[0]/max(eigen[-1],1e-12))

    def fit(self,aligner,source,body):
        source=source[np.linalg.norm(source-body,axis=1)<self.radius]
        if len(source)<1500:raise ValueError('too few points for whole-map initialization')
        held=np.arange(len(source))%5==0;train=source[~held];test=source[held]
        train=train[np.linspace(0,len(train)-1,min(len(train),3000)).astype(int)]
        coarse_train=train[np.linspace(0,len(train)-1,min(len(train),800)).astype(int)]
        candidates=self.candidates(train,body);fits=[];rejected=[]
        for coarse_score,seed,yaw in candidates:
            r=rotation(yaw);t=seed-r@body
            try:
                t,r=aligner.refine_rigid(coarse_train,t,r)
                t,r=aligner.refine_rigid_planes(train,t,r)
            except ValueError:continue
            position=r@body+t
            if np.linalg.norm(position-seed)>1. or math.acos(float(np.clip(r[2,2],-1,1)))>math.radians(10):continue
            q=aligner.quality(test,body,t,r)
            if not aligner.acceptable(q):rejected.append(q);continue
            q['initial_observability_ratio']=self.observability(aligner,test,t,r)
            q['coarse_search_score']=coarse_score
            fits.append((q['clipped_mean_m'],t,r,q))
        # Cold start needs strong structural overlap plus independently supported
        # 6-DoF geometry. Total point overlap alone is biased by changed clutter
        # and sparse floor samples. Competing lower-quality poses still veto it.
        qualified=[f for f in fits if f[3]['structure_inliers_15cm']>=.78 and
                   f[3]['initial_observability_ratio']>=.01]
        if not qualified:
            all_quality=[f[3] for f in fits]+rejected
            best=min(all_quality,key=lambda q:q['clipped_mean_m']) if all_quality else {}
            raise ValueError('no whole-map candidate has sufficient independent structure: '+str(best))
        qualified.sort(key=lambda item:item[0]);score,t,r,q=qualified[0]
        alternative=None
        for other_score,ot,other,oq in fits:
            different=np.linalg.norm((other@body+ot)-(r@body+t))>.4 or Rotation.from_matrix(other@r.T).magnitude()>.20
            if not different:continue
            alternative=other_score if alternative is None else min(alternative,other_score)
            if other_score<=score+.02:raise ValueError('ambiguous whole-map localization; waiting for distinctive measured structure')
        q.update(reference=aligner.reference_kind,initial_search_scope='whole_map',
                 initial_seed_count=len(self.seeds),initial_yaw_bins=48,
                 initial_candidates_refined=len(candidates),qualified_initial_candidates=len(qualified),
                 alternative_score_gap=None if alternative is None else alternative-score)
        return t,r,q
