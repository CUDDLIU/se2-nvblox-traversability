"""The same complete-voxel proofs as the global planner, with metric fallback.

A refined yaw bit never certifies an entire cell. Continuous motions must remain
inside a single convex, completely certified polygon; heights use the map's
existing vertical quantization. All other motions use the native mesh sweep.
"""
import math
import threading
import numpy as np
from shapely.geometry import Point,box
from core import wrap

class StaticGuard:
    def __init__(self,planner):self.planner=planner;self.lock=threading.Lock()
    def proof(self,a,b):
        p=self.planner;dz=p.config.vertical_resolution
        za=int(round(a[2]/dz));zb=int(round(b[2]/dz))
        if za!=zb:return False
        delta=float(wrap(b[3]-a[3]));width=2*math.pi/p.bins
        count=max(1,math.ceil(abs(delta)/width)*2+1)
        needed=sum(1<<j for j in {int(math.floor((a[3]+delta*i/count)/width+.5))%p.bins for i in range(count+1)})
        for raw in p.tree.query(box(min(a[0],b[0]),min(a[1],b[1]),max(a[0],b[0]),max(a[1],b[1]))):
            i=int(raw);poly=p.polygons[i]
            if int(round(poly['floor']/dz))!=za:continue
            if int(poly.get('proven_yaw_mask',0))&needed!=needed:continue
            # Convexity is established by navmesh build; both endpoints imply
            # containment of the entire XY segment, not just sparse samples.
            if p.shapes[i].covers(Point(a[:2])) and p.shapes[i].covers(Point(b[:2])):return True
        return False
    def motions(self,pairs):
        with self.lock:return self._motions(pairs)
    def _motions(self,pairs):
        ok=np.asarray([self.proof(a,b) for a,b in pairs],dtype=bool)
        indices=np.flatnonzero(~ok)
        if len(indices):ok[indices]=self.planner.checker.motions([pairs[i] for i in indices])
        return ok
