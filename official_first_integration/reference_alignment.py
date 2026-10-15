"""Same-recording LiDAR reference and quality-gated initial hypotheses."""
import hashlib
import json
import math
from itertools import combinations
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from alignment import MapAligner,rotation,correction_at

_PATCH_TRIANGLES=np.asarray(list(combinations(range(6),3)))

class ReferenceAligner(MapAligner):
    def __init__(self,directory,global_search=False):
        super().__init__(directory)
        self.global_search=None;self.input_range=8. if global_search else 4.
        self.reference_kind='mesh'
        manifest=Path(__file__).resolve().parent/'maps/localization_reference.json'
        if not manifest.exists():
            self.configure_search(directory,global_search);return
        d=json.loads(manifest.read_text())
        if d['mesh_sha256']!=self.mesh_sha256:
            self.configure_search(directory,global_search);return
        path=manifest.parent/d['file']
        if hashlib.sha256(path.read_bytes()).hexdigest()!=d['sha256']:raise ValueError('localization reference hash mismatch')
        with np.load(path,allow_pickle=False) as a:
            points=a['points'];normals=a['normals']
        if points.ndim!=2 or points.shape[1]!=3 or points.shape!=normals.shape or not np.isfinite(points).all() or not np.isfinite(normals).all():raise ValueError('invalid localization reference')
        self.target=points;self.normals=normals;self.tree=cKDTree(points);self.reference_kind='same_bag_lidar'
        self.configure_search(directory,global_search)

    def configure_search(self,directory,enabled):
        if enabled:
            from initial_alignment import GlobalInitialSearch,map_seeds
            self.global_search=GlobalInitialSearch(self.target,map_seeds(directory))

    @staticmethod
    def initial_acceptable(q):
        return MapAligner.acceptable(q) and q['inliers_15cm']>=.85 and q['structure_inliers_15cm']>=.78

    def surface_distances(self,points):
        """Distance to bounded, normal-consistent reference triangles.

        Sparse reference samples are not isolated physical point obstacles.
        Only interpolate inside triangles supported by nearby map samples;
        never extrapolate an infinite plane across an unobserved region.
        """
        distance,index=self.tree.query(points,k=6,workers=1)
        if len(self.target)<6:return distance[:,0],np.zeros(len(points),dtype=bool)
        vertices=self.target[index][:,_PATCH_TRIANGLES]
        normals=self.normals[index][:,_PATCH_TRIANGLES]
        u=vertices[:,:,1]-vertices[:,:,0];v=vertices[:,:,2]-vertices[:,:,0]
        cross=np.cross(u,v);area=np.linalg.norm(cross,axis=2)
        normal=cross/np.maximum(area[:,:,None],1e-12)
        valid=(area>.0004)&(np.max(np.linalg.norm(vertices-points[:,None,None,:],axis=3),axis=2)<.35)
        valid &= np.min(np.abs(np.sum(normals*normal[:,:,None,:],axis=3)),axis=2)>.95
        offset=points[:,None,:]-vertices[:,:,0]
        signed=np.sum(offset*normal,axis=2);projected=offset-signed[:,:,None]*normal
        uu=np.sum(u*u,axis=2);uv=np.sum(u*v,axis=2);vv=np.sum(v*v,axis=2)
        up=np.sum(u*projected,axis=2);vp=np.sum(v*projected,axis=2)
        determinant=np.maximum(uu*vv-uv*uv,1e-16)
        alpha=(vv*up-uv*vp)/determinant;beta=(uu*vp-uv*up)/determinant
        valid &= (alpha>=0)&(beta>=0)&(alpha+beta<=1)
        surface=np.min(np.where(valid,np.abs(signed),np.inf),axis=1)
        return np.minimum(distance[:,0],surface),np.any(valid,axis=1)

    def quality(self,source,body,translation,yaw):
        q=super().quality(source,body,translation,yaw)
        r=rotation(yaw) if np.ndim(yaw)==0 else np.asarray(yaw)
        distance,supported=self.surface_distances(source@r.T+translation)
        structure=source[:,2]>body[2]-.30
        q.update(vertex_median_m=q['median_m'],vertex_inliers_15cm=q['inliers_15cm'],
            median_m=float(np.median(distance)),clipped_mean_m=float(np.minimum(distance,.3).mean()),
            inliers_15cm=float(np.mean(distance<.15)),
            structure_inliers_15cm=float(np.mean(distance[structure]<.15)) if structure.any() else 0.,
            distance_metric='bounded_reference_surface',surface_supported_fraction=float(np.mean(supported)))
        return q

    def recover_continuous(self,train,test,body,seed):
        """Local recovery under the existing tracking correction bounds.

        This prior is supplied only by the same uninterrupted odometry session.
        It is never an alternative cold-start or a trusted output pose.
        """
        old_t,old_r=seed;center=old_r@body+old_t;fits=[]
        hypotheses=[(old_t,old_r)]
        for axis in np.eye(3):
            for sign in (-1.,1.):hypotheses.append((old_t+sign*.08*axis,old_r))
        for angle in (-.04,.04):
            r=rotation(angle)@old_r;hypotheses.append((center-r@body,r))
        for t,r in hypotheses:
            try:t,r=self.refine_rigid_planes(train,t,r)
            except ValueError:continue
            shift=float(np.linalg.norm(correction_at(body,old_t,old_r,t,r)))
            angle=float(Rotation.from_matrix(r@old_r.T).magnitude())
            if shift>.15 or angle>.06 or math.acos(float(np.clip(r[2,2],-1,1)))>math.radians(10):continue
            q=self.quality(test,body,t,r)
            if not self.acceptable(q):continue
            q.update(reference=self.reference_kind,alignment_scope='continuous_odometry_recovery',
                     correction_at_body_m=shift,correction_rotation_rad=angle)
            fits.append((q['clipped_mean_m'],t,r,q))
        if not fits:return None
        fits.sort(key=lambda item:item[0]);score,t,r,q=fits[0]
        for other_score,ot,other,oq in fits[1:]:
            if other_score>score+.012:continue
            if np.linalg.norm(correction_at(body,t,r,ot,other))>.08 or Rotation.from_matrix(other@r.T).magnitude()>.04:
                return None
        q['qualified_recovery_candidates']=len(fits)
        return t,r,q

    def fit_rigid(self,source,body,previous=None):
        if previous is not None:return super().fit_rigid(source,body,previous)
        source=np.asarray(source);body=np.asarray(body)
        global_search=getattr(self,'global_search',None)
        long_source=source
        seed=getattr(self,'recovery_seed',None)
        if global_search is not None and seed is None:return global_search.fit(self,long_source,body)
        source=source[np.linalg.norm(source-body,axis=1)<4.]
        if len(source)<600:
            if global_search is not None:return global_search.fit(self,long_source,body)
            raise ValueError('too few near-field points')
        held=np.arange(len(source))%5==0;train=source[~held];test=source[held]
        if len(train)>3000:train=train[np.linspace(0,len(train)-1,3000).astype(int)]
        if seed is not None:
            recovered=self.recover_continuous(train,test,body,seed)
            if recovered is not None:return recovered
        if global_search is not None:return global_search.fit(self,long_source,body)
        anchor=self.anchor if seed is None else seed[1]@body+seed[0]
        coarse=train[::max(1,len(train)//800)];candidates=[]
        # ICP's capture basin can be narrower than the old 1 m seed spacing.
        # Cover half-metre offsets within the same bounded search area; retain
        # held-out quality, floor/tilt bounds and qualified-pose ambiguity checks.
        offsets=np.arange(-2.,2.01,.5)
        for y in np.arange(-math.pi,math.pi,math.pi/12)+getattr(self,'seed_yaw',0.):
            r=rotation(y)
            for dx in offsets:
                for dy in offsets:
                    t=anchor+np.array([dx,dy,0.])-body@r.T
                    distance,_=self.tree.query(coarse@r.T+t,workers=1)
                    candidates.append((float(np.minimum(distance,.3).mean()),t,r))
        candidates.sort(key=lambda item:item[0]);fits=[];rejected=[]
        for _,t,r in candidates[:24]:
            try:t,r=self.refine_rigid(train,t,r)
            except ValueError:continue
            if math.acos(float(np.clip(r[2,2],-1,1)))>math.radians(10):continue
            p=body@r.T+t
            if np.linalg.norm(p[:2]-anchor[:2])>3. or abs(p[2]-anchor[2])>.5:continue
            q=self.quality(test,body,t,r)
            # A hypothesis that cannot initialize localization must not veto
            # another hypothesis by being counted as an ambiguous alternative.
            if not self.initial_acceptable(q):rejected.append(q);continue
            fits.append((q['clipped_mean_m'],t,r,q))
        # Reacquisition may preserve a measured roll/pitch estimate that a
        # yaw-only lattice loses. Evaluate it with the tracking plane objective
        # and the SAME strict initial quality/ambiguity checks. It is only a
        # hypothesis; never publish the rejected old transform as a fallback.
        if seed is not None:
            try:
                t,r=self.refine_rigid_planes(train,*seed)
                p=body@r.T+t;q=self.quality(test,body,t,r)
                if (math.acos(float(np.clip(r[2,2],-1,1)))<=math.radians(10) and
                        np.linalg.norm(p[:2]-anchor[:2])<=.5 and abs(p[2]-anchor[2])<=.5 and self.initial_acceptable(q)):
                    fits.append((q['clipped_mean_m'],t,r,q))
            except ValueError:pass
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
