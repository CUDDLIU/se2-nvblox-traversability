"""Measured four-DoF scan-to-saved-mesh alignment; no ROS or guessed pose."""
from pathlib import Path
import math
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp


def rotation(yaw):
    c,s=math.cos(yaw),math.sin(yaw)
    return np.array([[c,-s,0.],[s,c,0.],[0.,0.,1.]])


def voxel(points,resolution=.12):
    _,indices=np.unique(np.floor(points/resolution).astype(np.int64),axis=0,return_index=True)
    return points[indices]


def deskew(raw,poses):
    """raw = x,y,z,acquisition_seconds; poses = seconds,x,y,z,qx,qy,qz,qw."""
    poses=np.asarray(poses,dtype=float)
    if len(poses)<2:raise ValueError('waiting for measured pose history')
    _,indices=np.unique(poses[:,0],return_index=True);poses=poses[indices]
    valid=np.isfinite(raw).all(1)&(raw[:,3]>=poses[0,0])&(raw[:,3]<=poses[-1,0])
    raw=raw[valid]
    if len(raw)<200:raise ValueError('scan is not bracketed by measured odometry')
    lo,hi=float(raw[:,3].min()),float(raw[:,3].max())
    times=np.linspace(lo,hi,max(1,math.ceil((hi-lo)/.0005))+1)
    r=Slerp(poses[:,0],Rotation.from_quat(poses[:,4:]))(times).as_matrix()
    p=np.column_stack([np.interp(times,poses[:,0],poses[:,i]) for i in [1,2,3]])
    indices=np.rint((raw[:,3]-lo)/max(hi-lo,1e-12)*(len(times)-1)).astype(int)
    return voxel(np.einsum('nij,nj->ni',r[indices],raw[:,:3])+p[indices])


class MapAligner:
    def __init__(self,directory):
        import json
        self.directory=Path(directory)
        doc=json.loads((self.directory/'navmesh.json').read_text())
        with np.load(self.directory/doc['source_mesh'],allow_pickle=False) as a:
            points=[];normals=[]
            for i in range(len(a['keys'])):
                v=a['v'+str(i)];tri=a['t'+str(i)];n=np.zeros_like(v)
                face=np.cross(v[tri[:,1]]-v[tri[:,0]],v[tri[:,2]]-v[tri[:,0]])
                for j in range(3):np.add.at(n,tri[:,j],face)
                length=np.linalg.norm(n,axis=1);good=length>1e-10
                points.append(v[good]);normals.append(n[good]/length[good,None])
            points=np.concatenate(points);normals=np.concatenate(normals)
            _,indices=np.unique(np.floor(points/.10).astype(np.int64),axis=0,return_index=True)
            self.target=points[indices];self.normals=normals[indices]
            self.anchor=a['trajectory'][0,1:4].copy()
        self.tree=cKDTree(self.target)
        self.mesh_sha256=doc['mesh_sha256']

    def quality(self,source,body,translation,yaw):
        q=source@rotation(yaw).T+translation
        d,_=self.tree.query(q,workers=1)
        structure=source[:,2]>body[2]-.30
        return dict(points=len(source),median_m=float(np.median(d)),
            clipped_mean_m=float(np.minimum(d,.3).mean()),inliers_15cm=float(np.mean(d<.15)),
            structure_points=int(structure.sum()),
            structure_inliers_15cm=float(np.mean(d[structure]<.15)) if structure.any() else 0.)

    @staticmethod
    def acceptable(quality):
        return (quality['points']>=200 and quality['median_m']<.10 and
                quality['inliers_15cm']>=.72 and quality['structure_points']>=100 and
                quality['structure_inliers_15cm']>=.60)

    def refine(self,source,translation,yaw,iterations=45):
        t=np.asarray(translation,dtype=float).copy();y=float(yaw)
        for _ in range(iterations):
            q=source@rotation(y).T+t;d,indices=self.tree.query(q,workers=1)
            good=(d<.4)&(d<=np.quantile(d,.85))
            if good.sum()<100:raise ValueError('too few matched surfaces')
            u=q[good];v=self.target[indices[good]];uc=u-u.mean(0);vc=v-v.mean(0)
            dy=math.atan2(float(np.sum(uc[:,0]*vc[:,1]-uc[:,1]*vc[:,0])),float(np.sum(uc[:,:2]*vc[:,:2])))
            r=rotation(dy);dt=v.mean(0)-u.mean(0)@r.T;t=t@r.T+dt;y+=dy
            if np.linalg.norm(dt)<.001 and abs(dy)<.0003:break
        return t,math.atan2(math.sin(y),math.cos(y))

    def refine_planes(self,source,translation,yaw):
        """Track against measured surface normals, avoiding tangential floor bias."""
        t=np.asarray(translation,dtype=float).copy();y=float(yaw)
        for _ in range(8):
            q=source@rotation(y).T+t;d,index=self.tree.query(q,workers=1);good=d<.3
            if good.sum()<200:raise ValueError('too few local plane correspondences')
            q=q[good];v=self.target[index[good]];n=self.normals[index[good]];center=q.mean(0)
            residual=np.einsum('ij,ij->i',n,v-q)
            jac=np.column_stack((n,-n[:,0]*(q[:,1]-center[1])+n[:,1]*(q[:,0]-center[0])))
            weights=np.minimum(1.,.04/np.maximum(np.abs(residual),1e-8))*np.where(abs(n[:,2])>.8,.25,1.)
            lhs=jac.T@(weights[:,None]*jac);rhs=jac.T@(weights*residual)
            eigen=np.linalg.eigvalsh(lhs)
            if eigen[0]<eigen[-1]*1e-4:raise ValueError('map match lacks independent surface constraints')
            delta=np.linalg.solve(lhs,rhs)
            if np.linalg.norm(delta[:3])>.1 or abs(delta[3])>.08:raise ValueError('plane correction too large')
            r=rotation(delta[3]);t=(t-center)@r.T+center+delta[:3];y+=delta[3]
            if np.linalg.norm(delta[:3])<.001 and abs(delta[3])<.0003:break
        return t,math.atan2(math.sin(y),math.cos(y))

    def fit(self,source,body,previous=None):
        source=np.asarray(source);body=np.asarray(body)
        source=source[np.linalg.norm(source-body,axis=1)<4.]
        if len(source)<600:raise ValueError('too few near-field points')
        # Deterministic holdout points are never used to fit the transform.
        held=np.arange(len(source))%5==0;train=source[~held];test=source[held]
        if len(train)>3000:train=train[np.linspace(0,len(train)-1,3000).astype(int)]
        if previous is None:
            coarse=train[::max(1,len(train)//800)]
            candidates=[]
            for y in np.arange(-math.pi,math.pi,math.pi/12):
                r=rotation(y)
                for dx in (-.5,0.,.5):
                    for dy in (-.5,0.,.5):
                        t=self.anchor+np.array([dx,dy,0.])-body@r.T
                        d,_=self.tree.query(coarse@r.T+t,workers=1)
                        candidates.append((float(np.minimum(d,.3).mean()),t,y))
            candidates.sort(key=lambda x:x[0]);fits=[]
            for _,t,y in candidates[:10]:
                t,y=self.refine(train,t,y,65);q=self.quality(test,body,t,y)
                if np.linalg.norm(body@rotation(y).T+t-self.anchor)>1.5:continue
                fits.append((q['clipped_mean_m'],t,y,q))
            if not fits:raise ValueError('no match near the confirmed map start')
            fits.sort(key=lambda x:x[0]);_,t,y,q=fits[0]
            for other,ot,oy,oq in fits[1:]:
                if other>q['clipped_mean_m']+.012:continue
                if np.linalg.norm(ot-t)>.4 or abs(math.atan2(math.sin(oy-y),math.cos(oy-y)))>.20:
                    raise ValueError('ambiguous map-start match; need a more distinctive view')
        else:
            old_t,old_y=previous
            t,y=(self.refine_planes(train,old_t,old_y) if hasattr(self,'normals') else self.refine(train,old_t,old_y,10))
            if np.linalg.norm(t-old_t)>.15 or abs(math.atan2(math.sin(y-old_y),math.cos(y-old_y)))>.06:
                raise ValueError('map correction jumped; stop and relocalize')
            # Apply small tracking corrections smoothly in the global frame.
            t=.25*t+.75*old_t;y=old_y+.25*math.atan2(math.sin(y-old_y),math.cos(y-old_y))
            q=self.quality(test,body,t,y)
        if not self.acceptable(q):raise ValueError('map match failed quality check: '+str(q))
        if previous is None and (q['inliers_15cm']<.85 or q['structure_inliers_15cm']<.78):
            raise ValueError('initial map match needs a more distinctive view: '+str(q))
        return t,y,q
