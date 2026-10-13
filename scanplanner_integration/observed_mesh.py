"""Conservative, disposable local mesh overlay from explicitly observed voxels.

This is a candidate-validation experiment, not a new trajectory generator. The
saved map is immutable. Unknown cells retain saved triangles; a triangle is
removed only if its entire voxel AABB is explicitly free. Support patches need
four neighbouring measured occupied columns in the same ground layer. The
normal swept-body/support checker validates the resulting geometry.
"""
import math
import numpy as np


class ObservedVoxels:
    def __init__(self, records):
        a=np.asarray(records,dtype=float).reshape(-1,5)
        if not len(a) or not np.isfinite(a).all():raise ValueError('empty or invalid observation')
        self.resolution=float(a[0,3]);r=self.resolution
        if not .025<=r<=.1 or not np.allclose(a[:,3],r,atol=1e-8):raise ValueError('invalid voxel resolution')
        if not np.isin(a[:,4],[0.,1.]).all():raise ValueError('invalid observation state')
        ids=np.floor(a[:,:3]/r+1e-5).astype(np.int64)
        if np.max(np.abs(a[:,:3]-(ids+.5)*r))>r*1e-3:raise ValueError('unaligned observed voxels')
        self.low=ids.min(axis=0);shape=ids.max(axis=0)-self.low+1
        if np.prod(shape)>4_000_000:raise ValueError('observation extent exceeds local bound')
        self.state=np.full(shape,-1,dtype=np.int8)
        # Occupied wins conflicting records; absence never writes free.
        for state in (0,1):self.state[tuple((ids[a[:,4]==state]-self.low).T)]=state
        free=(self.state==0).astype(np.int32)
        self.integral=np.pad(free,((1,0),)*3).cumsum(0).cumsum(1).cumsum(2)
        self.occupied=(np.argwhere(self.state==1)+self.low+.5)*r

    def box_evidence(self, low, high):
        """Complete boxes, including both sides of a voxel boundary."""
        r=self.resolution
        lo=np.floor(np.asarray(low)/r-1e-7).astype(np.int64)-self.low
        hi=np.floor(np.asarray(high)/r+1e-7).astype(np.int64)-self.low+1
        valid=(lo>=0).all(axis=1)&(hi<=self.state.shape).all(axis=1)
        result=np.zeros(len(lo),dtype=float);a=lo[valid];b=hi[valid]
        if not len(a):return result
        total=np.zeros(len(a),dtype=np.int64)
        for bits in range(8):
            idx=np.column_stack([b[:,d] if bits&(1<<d) else a[:,d] for d in range(3)])
            sign=(-1)**(3-bits.bit_count())
            total+=sign*self.integral[tuple(idx.T)]
        result[valid]=total/np.prod(b-a,axis=1)
        return result

    def free_boxes(self,low,high):
        return self.box_evidence(low,high)==1.

    def retain_unobserved(self,triangles):
        # Clip only with whole-AABB evidence. Recursion refines boundary faces;
        # terminal mixed/unknown patches stay, regardless of their centroid.
        pending=np.asarray(triangles).reshape(-1,3,3);kept=[];removed=0
        for depth in range(7):
            if not len(pending):break
            evidence=self.box_evidence(pending.min(1),pending.max(1))
            removed+=int((evidence==1.).sum())
            split=(evidence>0)&(evidence<1)&(depth<6)&(np.ptp(pending,axis=1).max(1)>self.resolution/4)
            kept.append(pending[(evidence<1)&~split])
            a,b,c=pending[split].transpose(1,0,2);ab=(a+b)/2;bc=(b+c)/2;ca=(c+a)/2
            pending=np.concatenate([np.stack(t,axis=1) for t in ((a,ab,ca),(ab,b,bc),(ca,bc,c),(ab,bc,ca))])
        return np.concatenate(kept) if kept else np.empty((0,3,3)),removed

    def support_triangles(self, layer, max_step, slope_deg):
        """2x2 observed columns only; never bridge an unknown column or floor.

        Occupied voxel centres approximate measured support to half a voxel.
        No patch is inferred from free rays or an isolated occupied sample.
        Multi-height columns select the nearest sample within one ground layer.
        """
        r=self.resolution;p=self.occupied
        p=p[np.abs(p[:,2]-layer)<=max_step]
        if not len(p):return np.empty((0,3,3))
        ids=np.floor(p[:,:2]/r+1e-5).astype(int)
        columns={}
        for key,q in zip(map(tuple,ids),p):
            if key not in columns or abs(q[2]-layer)<abs(columns[key][2]-layer):columns[key]=q
        tris=[];cosine=math.cos(math.radians(slope_deg))
        for (x,y),a in columns.items():
            b=columns.get((x+1,y));c=columns.get((x+1,y+1));d=columns.get((x,y+1))
            if b is None or c is None or d is None:continue
            if np.ptp([a[2],b[2],c[2],d[2]])>max_step:continue
            pair=[np.array([a,b,c]),np.array([a,c,d])]
            normals=np.array([np.cross(t[1]-t[0],t[2]-t[0]) for t in pair])
            if np.all(normals[:,2]>=cosine*np.linalg.norm(normals,axis=1)):tris.extend(pair)
        return np.asarray(tris).reshape(-1,3,3)

    def fuse(self, saved, layer, config):
        saved=np.asarray(saved).reshape(-1,3,3)
        retained,removed=self.retain_unobserved(saved)
        support=self.support_triangles(layer,config.max_step,config.max_slope_deg)
        return np.concatenate((retained,support)),dict(saved_triangles=len(saved),
            removed_triangles=removed,support_triangles=len(support),
            known_free=int((self.state==0).sum()),occupied=len(self.occupied))
