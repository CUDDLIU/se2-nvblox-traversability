"""Compact native A* with lazy checks against the same SE(2) representation."""
import ctypes
import math
from pathlib import Path
import time
import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra


class Search:
    def __init__(self, planner):
        self.p = planner
        self.stride = planner.bins+1
        self.library = ctypes.CDLL(str(Path(__file__).with_name('astar.so')))
        self.library.se2_astar.argtypes = ([ctypes.c_int64]+[ctypes.c_void_p]*6+
                                         [ctypes.c_int64]+[ctypes.c_void_p]*5)
        self.library.se2_astar.restype = ctypes.c_int
        self.base_pairs = None
        self.pose_cache = {}

    def encode(self,node):return node[0]*self.stride+node[1]+1
    def decode(self,node):return int(node)//self.stride,int(node)%self.stride-1

    def topology(self, groups, extra, include):
        p=self.p; pairs=[];position_pairs=set()
        for ids in groups:
            ids=list(ids)
            for i,a in enumerate(ids):
                for b in ids[i+1:]:
                    if include(a,b):position_pairs.add(tuple(sorted((a,b))))
        for a,targets in extra.items():
            for b in targets:
                if include(a,b):position_pairs.add(tuple(sorted((a,b))))
        for a,b in sorted(position_pairs):
            for y in p.angles[a].keys() & p.angles[b].keys():
                pairs.append((self.encode((a,y)),self.encode((b,y))))
        return pairs

    def graph(self, chain):
        p=self.p;base=p.base_positions
        if self.base_pairs is None and chain is None:
            self.base_pairs=self.topology(([i for i in ids if i<base] for ids in p.region_positions.values()),p.extra,
                                          lambda a,b:a<base and b<base)
            for pid in range(base):
                for yaw in p.angles[pid]:
                    y=(yaw+1)%p.bins
                    if y in p.angles[pid]:self.base_pairs.append((self.encode((pid,yaw)),self.encode((pid,y))))
        if chain is None:
            pairs=list(self.base_pairs)
            pairs.extend(self.topology(p.region_positions.values(),{},lambda a,b:a>=base or b>=base))
            pids=range(base,len(p.positions))
        else:
            pairs=self.topology(([a,b] for a,targets in chain.items() for b in targets),{},lambda a,b:True)
            pids=list(chain)
        for pid in pids:
            for yaw in p.angles[pid]:
                ys = list(p.angles[pid]) if yaw==-1 else [(yaw+1)%p.bins]
                for y in ys:
                    if y in p.angles[pid] and y!=yaw:pairs.append((self.encode((pid,yaw)),self.encode((pid,y))))
        pair=np.asarray(pairs,dtype=np.int64).reshape(-1,2)
        pair=np.unique(np.vstack([pair,pair[:,::-1]]),axis=0)
        n=len(p.positions)*self.stride
        coords=np.zeros((n,4),dtype=np.float64)
        for pid in range(len(p.positions)):
            for yaw,value in p.angles[pid].items():coords[self.encode((pid,yaw))]=[*p.positions[pid],value]
        a,b=coords[pair[:,0]],coords[pair[:,1]]
        delta=b[:,:2]-a[:,:2];c,s=np.cos(a[:,3]),np.sin(a[:,3])
        angle=np.abs((b[:,3]-a[:,3]+math.pi)%(2*math.pi)-math.pi)
        cost=np.abs(c*delta[:,0]+s*delta[:,1])/p.speeds[0]+np.abs(-s*delta[:,0]+c*delta[:,1])/p.speeds[1]+angle/p.speeds[2]
        offsets=np.r_[0,np.cumsum(np.bincount(pair[:,0],minlength=n))].astype(np.int64)
        return pair,coords,np.ascontiguousarray(cost),offsets,np.ascontiguousarray(pair[:,1])

    def run(self,start,goals,chain,timeout):
        from planner import NoPath
        began=time.monotonic();p=self.p
        pair,coords,cost,offsets,target=self.graph(chain)
        n=len(coords);start_id=self.encode(start)
        goal_ids=np.asarray([self.encode(g) for g in goals],dtype=np.int64)
        # Reverse-graph Dijkstra gives a relaxed lower bound that ignores only
        # collision checks. A* still searches the complete SE(2) state graph.
        optimistic=csr_matrix((cost,(pair[:,1],pair[:,0])),shape=(n,n))
        heuristic=dijkstra(optimistic,directed=True,indices=goal_ids,min_only=True)
        if not math.isfinite(heuristic[start_id]):
            raise NoPath('起点与目标的楼层/朝向在当前 SE(2) 图中不连通。')
        blocked_edges=np.zeros(len(pair),dtype=np.uint8)
        blocked_nodes=np.zeros(n,dtype=np.uint8)
        # Reuse static-map rejections between clicks and between ASA stages.
        for node, valid in self.pose_cache.items():
            if not valid and self.encode(node)<n:
                blocked_nodes[self.encode(node)]=1
        for (a,b), valid in p.edge_cache.items():
            if valid:continue
            for u,v in ((self.encode(a),self.encode(b)),(self.encode(b),self.encode(a))):
                if u>=n or v>=n:continue
                index=offsets[u]+np.searchsorted(target[offsets[u]:offsets[u+1]],v)
                if index<offsets[u+1] and target[index]==v:blocked_edges[index]=1
        goal_mask=np.zeros(n,dtype=np.uint8);goal_mask[goal_ids]=1
        output=np.empty(n,dtype=np.int64)
        expansions=0;passes=0
        while True:
            if time.monotonic()-began>timeout:
                raise NoPath(f'规划超时（几何复核 {passes} 轮）；未发布未验证路径。')
            length=ctypes.c_int64();total=ctypes.c_double();expanded=ctypes.c_int64()
            code=self.library.se2_astar(n,offsets.ctypes.data,target.ctypes.data,cost.ctypes.data,
                blocked_edges.ctypes.data,blocked_nodes.ctypes.data,heuristic.ctypes.data,start_id,
                goal_mask.ctypes.data,output.ctypes.data,ctypes.byref(length),ctypes.byref(total),ctypes.byref(expanded))
            expansions+=expanded.value;passes+=1
            if code:
                self.last_failure=dict(passes=passes,expanded=expansions,blocked_nodes=int(blocked_nodes.sum()),
                                       blocked_edges=int(blocked_edges.sum()),start_blocked=int(blocked_nodes[start_id]),
                                       goal_blocked=blocked_nodes[goal_ids].tolist(),
                                       invalid_poses=coords[np.flatnonzero(blocked_nodes)].tolist())
                raise NoPath('当前地图没有满足机身轮廓、支撑与朝向约束的跨层路径。')
            ids=output[:length.value].copy();nodes=[self.decode(v) for v in ids]
            unknown=[]
            for node in nodes:
                if node not in self.pose_cache:
                    if p.voxel_proof(node,node):self.pose_cache[node]=True
                    else:unknown.append(node)
            unknown=list(dict.fromkeys(unknown))
            for node,ok in zip(unknown,p.checker.motions([(p.pose(v),p.pose(v)) for v in unknown])):
                self.pose_cache[node]=bool(ok)
            invalid=[self.encode(node) for node in nodes if not self.pose_cache[node]]
            if invalid:
                blocked_nodes[invalid]=1
                continue
            unknown=[]
            for a,b in zip(nodes,nodes[1:]):
                key=tuple(sorted((a,b)))
                if key not in p.edge_cache:
                    if p.voxel_proof(a,b):p.edge_cache[key]=True
                    else:unknown.append(key)
            unknown=list(dict.fromkeys(unknown))
            for key,ok in zip(unknown,p.checker.motions([(p.pose(a),p.pose(b)) for a,b in unknown])):
                p.edge_cache[key]=bool(ok)
            invalid=False
            for a,b in zip(nodes,nodes[1:]):
                if p.edge_cache[tuple(sorted((a,b)))]:continue
                invalid=True
                for u,v in ((self.encode(a),self.encode(b)),(self.encode(b),self.encode(a))):
                    index=offsets[u]+np.searchsorted(target[offsets[u]:offsets[u+1]],v)
                    if index<offsets[u+1] and target[index]==v:blocked_edges[index]=1
            if not invalid:
                self.pose_cache={k:v for k,v in self.pose_cache.items() if k[0]<p.base_positions}
                return nodes,dict(cost=total.value,expanded=expansions,seconds=time.monotonic()-began,
                                  geometric_validation_passes=passes,search='native lazy A*')
