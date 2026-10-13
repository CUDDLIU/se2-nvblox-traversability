"""Snapshot checks for choosing connectors; the C++ guard still gates execution."""
import math
import numpy as np
from scipy.spatial import cKDTree
from core import wrap

class LiveOccupancy:
    def __init__(self,points,resolution=.1,z_resolution=None):
        self.points=np.asarray(points,dtype=float).reshape(-1,3)
        sizes=np.broadcast_to(np.asarray(resolution,dtype=float),(len(self.points),)).copy()
        valid=np.isfinite(self.points).all(axis=1)&np.isfinite(sizes)&(sizes>0)
        self.points=self.points[valid];self.sizes=sizes[valid]
        self.tree=cKDTree(self.points);self.resolution=float(z_resolution if z_resolution is not None else np.max(sizes) if len(sizes) else .1)
    def clear(self,p):
        return not len(self.contacts(p))
    def contacts(self,p):
        x,y,z,yaw=p;c,s=math.cos(yaw),math.sin(yaw);r=self.resolution
        ids=self.tree.query_ball_point([x,y,z],1.)
        if not ids:return np.empty(0,dtype=int)
        q=self.points[ids];delta=q[:,:2]-[x,y];pad=.5*self.sizes[ids]*(abs(c)+abs(s))
        # Match the execution guard's voxel bounds and complete torso rectangle.
        low=math.floor((z-.1-r)/r)*r
        high=(math.floor((z+.4+r)/r)+1)*r
        inside=((abs(delta[:,0]*c+delta[:,1]*s)<=.41+pad)&
                (abs(-delta[:,0]*s+delta[:,1]*c)<=.253+pad)&
                (q[:,2]>=low)&(q[:,2]<high))
        return np.asarray(ids,dtype=int)[inside]
    def motions(self,pairs):
        result=[]
        for a,b in pairs:
            a=np.asarray(a);b=np.asarray(b);dy=float(wrap(b[3]-a[3]))
            n=max(1,math.ceil(np.linalg.norm(b[:3]-a[:3])/.04),math.ceil(abs(dy)/.06))
            clear=True
            for i in range(n+1):
                q=a+(b-a)*(i/n);q[3]=a[3]+dy*i/n
                if not self.clear(q):clear=False;break
            result.append(clear)
        return np.asarray(result,dtype=bool)
