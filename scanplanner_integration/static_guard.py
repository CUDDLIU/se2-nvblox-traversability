"""The same complete-voxel proofs as the global planner, with metric fallback.

A refined yaw bit never certifies an entire cell. Continuous motions must remain
inside completely certified polygons for the whole yaw interval. Adjacent
quantized floors require explicit crossed NavMesh portals. All other motions
use the native mesh sweep.
"""
import math
import threading
import numpy as np
from shapely.geometry import Point,box,LineString
from shapely.ops import unary_union
from core import wrap

class StaticGuard:
    def __init__(self,planner):self.planner=planner;self.lock=threading.Lock()
    def surface_poses(self,poses):
        """Resolve a smoothed guide height onto its certified local surface.

        Only a containing, yaw-certified polygon within the map's step limit
        supplies a height. A sparse reference interpolates across local floor
        undulations; its Z is a layer hint, not the support height at detour XY.
        XY/yaw are unchanged. This is not motion approval:
        transitions still require the normal continuous portal/mesh proof.
        """
        result=np.asarray(poses,dtype=float).copy();p=self.planner
        for pose in result:
            point=Point(pose[:2]);needed=1<<(int(math.floor(pose[3]*p.bins/(2*math.pi)+.5))%p.bins)
            floors=[]
            for raw in p.tree.query(point):
                i=int(raw);poly=p.polygons[i];floor=poly['floor']
                if (abs(floor-pose[2])<=p.config.max_step+1e-9 and
                    int(poly.get('proven_yaw_mask',0))&needed and p.shapes[i].covers(point)):
                    floors.append(floor)
            if floors:pose[2]=min(floors,key=lambda z:(abs(z-pose[2]),z))
        return result
    def proof(self,a,b):
        p=self.planner;dz=p.config.vertical_resolution
        za=int(round(a[2]/dz));zb=int(round(b[2]/dz))
        delta=float(wrap(b[3]-a[3]));width=2*math.pi/p.bins
        count=max(1,math.ceil(abs(delta)/width)*2+1)
        needed=sum(1<<j for j in {int(math.floor((a[3]+delta*i/count)/width+.5))%p.bins for i in range(count+1)})
        certified=[];nearby={}
        for raw in p.tree.query(box(min(a[0],b[0]),min(a[1],b[1]),max(a[0],b[0]),max(a[1],b[1]))):
            i=int(raw);poly=p.polygons[i]
            if int(poly.get('proven_yaw_mask',0))&needed!=needed:continue
            # Body-Z is a smooth guide; the dog follows the support surface.
            # Keep adjacent quantized surfaces only within one map quantum.
            if max(abs(poly['floor']-a[2]),abs(poly['floor']-b[2]))<=dz+1e-9:nearby[i]=p.shapes[i]
            if za!=zb or int(round(poly['floor']/dz))!=za:continue
            # Convexity is established by navmesh build; both endpoints imply
            # containment of the entire XY segment, not just sparse samples.
            if p.shapes[i].covers(Point(a[:2])) and p.shapes[i].covers(Point(b[:2])):return True
            certified.append(p.shapes[i])
        # Every centre along the segment must belong to a fully certified
        # polygon at this same floor and for the whole required yaw interval.
        # Taking the union does not bridge gaps between endpoint certificates.
        if len(certified)>1:
            motion=LineString([a[:2],b[:2]]) if np.linalg.norm(a[:2]-b[:2])>1e-12 else Point(a[:2])
            if unary_union(certified).covers(motion):return True
        # Reuse explicit NavMesh portals for a one-quantum floor transition.
        # Mere XY contact, endpoint clearance, or nearby stacked floors do not
        # establish connectivity. The actual segment must cross every portal.
        if len(nearby)>1 and np.linalg.norm(a[:2]-b[:2])>1e-12:
            motion=LineString([a[:2],b[:2]]);unseen=set(nearby)
            while unseen:
                component={unseen.pop()};pending=list(component)
                while pending:
                    i=pending.pop()
                    for j in list(unseen):
                        if abs(p.polygons[i]['floor']-p.polygons[j]['floor'])>dz+1e-9:continue
                        portals=getattr(p,'portal_lookup',{}).get(tuple(sorted((i,j))),[])
                        if any(int(v['yaw_mask'])&needed==needed and motion.intersects(LineString(np.asarray(v['segment'])[:,:2])) for v in portals):
                            unseen.remove(j);component.add(j);pending.append(j)
                floors=[p.polygons[i]['floor'] for i in component]
                if (len(component)>1 and 1e-9<max(floors)-min(floors)<=dz+1e-9 and
                    unary_union([nearby[i] for i in component]).covers(motion)):
                    return True
        return False
    def motions(self,pairs):
        with self.lock:return self._motions(pairs)
    def _direct(self,pairs):
        ok=np.asarray([self.proof(a,b) for a,b in pairs],dtype=bool)
        indices=np.flatnonzero(~ok)
        if len(indices):ok[indices]=self.planner.checker.motions([pairs[i] for i in indices])
        return ok
    def _motions(self,pairs):
        ok=self._direct(pairs)
        # One enclosing rectangle can overestimate a translating/rotating
        # body. Refine only failed intervals; every leaf still requires a
        # complete certified-region or native swept-volume proof, never endpoint-only clearance.
        for i in np.flatnonzero(~ok):
            a,b=(np.asarray(v,dtype=float) for v in pairs[i])
            if not self._direct([(a,a),(b,b)]).all():continue
            pending=[(a,b)]
            for _ in range(6):
                halves=[]
                for a,b in pending:
                    mid=(a+b)*.5;mid[3]=a[3]+float(wrap(b[3]-a[3]))*.5
                    halves.extend([(a,mid),(mid,b)])
                accepted=self._direct(halves)
                pending=[v for v,good in zip(halves,accepted) if not good]
                if not pending:ok[i]=True;break
        return ok
