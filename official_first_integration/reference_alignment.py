"""Same-recording LiDAR reference and quality-gated initial hypotheses."""
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from alignment import MapAligner,rotation

class ReferenceAligner(MapAligner):
    def __init__(self,directory):
        super().__init__(directory)
        self.reference_kind='mesh'
        manifest=Path(__file__).resolve().parent/'maps/localization_reference.json'
        if not manifest.exists():return
        d=json.loads(manifest.read_text())
        if d['mesh_sha256']!=self.mesh_sha256:return
        path=manifest.parent/d['file']
        if hashlib.sha256(path.read_bytes()).hexdigest()!=d['sha256']:raise ValueError('localization reference hash mismatch')
        with np.load(path,allow_pickle=False) as a:
            points=a['points'];normals=a['normals']
        if points.ndim!=2 or points.shape[1]!=3 or points.shape!=normals.shape or not np.isfinite(points).all() or not np.isfinite(normals).all():raise ValueError('invalid localization reference')
        self.target=points;self.normals=normals;self.tree=cKDTree(points);self.reference_kind='same_bag_lidar'

    @staticmethod
    def initial_acceptable(q):
        return MapAligner.acceptable(q) and q['inliers_15cm']>=.85 and q['structure_inliers_15cm']>=.78

    def fit_rigid(self,source,body,previous=None):
        if previous is not None:return super().fit_rigid(source,body,previous)
        source=np.asarray(source);body=np.asarray(body)
        source=source[np.linalg.norm(source-body,axis=1)<4.]
        if len(source)<600:raise ValueError('too few near-field points')
        held=np.arange(len(source))%5==0;train=source[~held];test=source[held]
        if len(train)>3000:train=train[np.linspace(0,len(train)-1,3000).astype(int)]
        coarse=train[::max(1,len(train)//800)];candidates=[]
        for y in np.arange(-math.pi,math.pi,math.pi/12)+getattr(self,'seed_yaw',0.):
            r=rotation(y)
            for dx in (-2.,-1.,0.,1.,2.):
                for dy in (-2.,-1.,0.,1.,2.):
                    t=self.anchor+np.array([dx,dy,0.])-body@r.T
                    distance,_=self.tree.query(coarse@r.T+t,workers=1)
                    candidates.append((float(np.minimum(distance,.3).mean()),t,r))
        candidates.sort(key=lambda item:item[0]);fits=[];rejected=[]
        for _,t,r in candidates[:24]:
            try:t,r=self.refine_rigid(train,t,r)
            except ValueError:continue
            if math.acos(float(np.clip(r[2,2],-1,1)))>math.radians(10):continue
            p=body@r.T+t
            if np.linalg.norm(p[:2]-self.anchor[:2])>3. or abs(p[2]-self.anchor[2])>.5:continue
            q=self.quality(test,body,t,r)
            # A hypothesis that cannot initialize localization must not veto
            # another hypothesis by being counted as an ambiguous alternative.
            if not self.initial_acceptable(q):rejected.append(q);continue
            fits.append((q['clipped_mean_m'],t,r,q))
        if not fits:
            best=min(rejected,key=lambda q:q['clipped_mean_m']) if rejected else {}
            raise ValueError('no initial candidate passes quality checks: '+str(best))
        fits.sort(key=lambda item:item[0]);score,t,r,q=fits[0]
        for other_score,ot,other,oq in fits[1:]:
            if other_score>score+.012:continue
            if np.linalg.norm((body@other.T+ot)-(body@r.T+t))>.4 or Rotation.from_matrix(other@r.T).magnitude()>.20:
                raise ValueError('ambiguous qualified map matches; waiting for distinctive measured structure')
        q.update(reference=self.reference_kind,qualified_initial_candidates=len(fits))
        return t,r,q
