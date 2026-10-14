"""Compact native A* with lazy checks against the same SE(2) representation."""
import ctypes
import hashlib
import math
import os
from pathlib import Path
import tempfile
import time
import numpy as np
from scipy.sparse import csr_matrix


class NativeGraph(ctypes.Structure):
    _fields_ = [('nodes',ctypes.c_int64),('offsets',ctypes.c_void_p),('target',ctypes.c_void_p),
                ('costs',ctypes.c_void_p),('blocked',ctypes.c_void_p)]


class Search:
    def __init__(self, planner):
        self.p = planner
        self.stride = planner.bins+1
        self.library = ctypes.CDLL(str(Path(__file__).with_name('astar.so')))
        self.library.se2_astar.argtypes = ([ctypes.c_int64]+[ctypes.c_void_p]*6+
                                         [ctypes.c_int64]+[ctypes.c_void_p]*5)
        self.library.se2_astar.restype = ctypes.c_int
        self.library.se2_astar_graphs.argtypes = ([ctypes.c_int64,ctypes.c_void_p,ctypes.c_int64]+
            [ctypes.c_void_p]*2+[ctypes.c_int64]+[ctypes.c_void_p]*5)
        self.library.se2_astar_graphs.restype = ctypes.c_int
        self.library.se2_goal_distances_graphs.argtypes = ([ctypes.c_int64,ctypes.c_void_p,ctypes.c_int64,
            ctypes.c_void_p,ctypes.c_int64]+[ctypes.c_void_p]*2)
        self.library.se2_goal_distances_graphs.restype = ctypes.c_int
        self.base_pairs = None
        self.base_graph = None
        self.pose_cache = {}
        self.cache_key = None
        self.saved_counts = (0,0)

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

    def rotations(self, pids):
        p=self.p; pairs=[]
        for pid in pids:
            for yaw in p.angles[pid]:
                ys = list(p.angles[pid]) if yaw==-1 else [(yaw+1)%p.bins]
                for y in ys:
                    if y in p.angles[pid] and y!=yaw:pairs.append((self.encode((pid,yaw)),self.encode((pid,y))))
        return pairs

    def assemble(self, pairs, pids, n):
        """Sort scalar edge keys, not millions of two-column records."""
        p=self.p
        pair=np.asarray(pairs,dtype=np.int64).reshape(-1,2)
        keys=np.unique(np.r_[pair[:,0]*n+pair[:,1],pair[:,1]*n+pair[:,0]])
        pair=np.column_stack((keys//n,keys%n))
        coords=np.zeros((n,4),dtype=np.float64)
        for pid in pids:
            for yaw,value in p.angles[pid].items():coords[self.encode((pid,yaw))]=[*p.positions[pid],value]
        a,b=coords[pair[:,0]],coords[pair[:,1]]
        delta=b[:,:2]-a[:,:2];c,s=np.cos(a[:,3]),np.sin(a[:,3])
        angle=np.abs((b[:,3]-a[:,3]+math.pi)%(2*math.pi)-math.pi)
        cost=np.abs(c*delta[:,0]+s*delta[:,1])/p.speeds[0]+np.abs(-s*delta[:,0]+c*delta[:,1])/p.speeds[1]+angle/p.speeds[2]
        # Compute each spatial edge once, shared by all yaw layers/directions.
        # The reverse-graph heuristic uses these same nonnegative costs.
        if p.clearance.weight:
            spatial=np.sort(pair//self.stride,axis=1)
            count=len(p.positions)
            unique,inverse=np.unique(spatial[:,0]*count+spatial[:,1],return_inverse=True)
            positions=np.asarray(p.positions)
            cost *= p.clearance.edge_factors(positions[unique//count],positions[unique%count])[inverse]
        offsets=np.r_[0,np.cumsum(np.bincount(pair[:,0],minlength=n))].astype(np.int64)
        return pair,coords,np.ascontiguousarray(cost),offsets,np.ascontiguousarray(pair[:,1])

    def prepare(self):
        """Immutable map topology, coordinates and costs, prepared before clicks."""
        p=self.p;base=p.base_positions
        if self.base_graph is None:
            self.base_pairs=self.topology(([i for i in ids if i<base] for ids in p.region_positions.values()),p.extra,
                                          lambda a,b:a<base and b<base)
            self.base_pairs.extend(self.rotations(range(base)))
            self.base_graph=self.assemble(self.base_pairs,range(base),base*self.stride)
            self.base_reverse=self.reverse(self.base_graph)
            # Python tuples consumed considerably more memory than the CSR graph.
            self.base_pairs=None
            self.load_certificates()

    def load_certificates(self):
        """Certificates belong to exact map/pose/checker content, never a robot pose."""
        from geometry import CORE
        digest=hashlib.sha256(self.base_graph[1].tobytes())
        paths=[self.p.directory/'navmesh.json',self.p.directory/'field.npz']
        paths += [Path(__file__).with_name(name) for name in ('search.py','planner.py','geometry.py','clearance.py')]
        paths += [CORE/name for name in ('raster.so','native_mesh.py','pose_graph.py','paper_pipeline.py','paper_native.py')]
        for path in paths:
            if path.exists():digest.update(path.read_bytes())
        self.cache_key=digest.hexdigest()
        self.cache_path=self.p.directory/'runtime/asa_static_checks.npz'
        try:
            with np.load(self.cache_path,allow_pickle=False) as data:
                if str(data['key'])!=self.cache_key:return
                nodes=data['nodes'];edges=data['edges']
                if nodes.ndim!=2 or nodes.shape[1]!=3 or edges.ndim!=2 or edges.shape[1]!=5:return
                def valid(pid,yaw):
                    return 0<=pid<self.p.base_positions and yaw in self.p.angles[pid]
                poses={};checks={}
                for pid,yaw,ok in nodes.tolist():
                    if not valid(pid,yaw) or ok not in (0,1):return
                    poses[(pid,yaw)]=bool(ok)
                for a,ya,b,yb,ok in edges.tolist():
                    if not valid(a,ya) or not valid(b,yb) or ok not in (0,1):return
                    checks[tuple(sorted(((a,ya),(b,yb))))]=bool(ok)
            self.pose_cache.update(poses);self.p.edge_cache.update(checks)
            self.saved_counts=(len(poses),len(checks))
        except (OSError,ValueError,KeyError,EOFError):
            pass  # Missing/stale/corrupt cache only costs more validation work.

    def save_certificates(self):
        # Called after query vertices/caches have been removed in Planner.finally.
        counts=(len(self.pose_cache),len(self.p.edge_cache))
        if self.cache_key is None or counts==self.saved_counts:return
        path=None
        try:
            self.cache_path.parent.mkdir(parents=True,exist_ok=True)
            with tempfile.NamedTemporaryFile(dir=self.cache_path.parent,prefix='.asa_checks_',delete=False) as f:
                path=Path(f.name)
                nodes=np.asarray([(*n,int(v)) for n,v in self.pose_cache.items()],dtype=np.int64).reshape(-1,3)
                edges=np.asarray([(*a,*b,int(v)) for (a,b),v in self.p.edge_cache.items()],dtype=np.int64).reshape(-1,5)
                np.savez_compressed(f,key=self.cache_key,nodes=nodes,edges=edges)
            os.replace(path,self.cache_path);self.saved_counts=counts
        except OSError:
            pass  # A read-only map remains usable.
        finally:
            if path is not None:path.unlink(missing_ok=True)

    @staticmethod
    def reverse(graph):
        _,coords,cost,offsets,target=graph
        matrix=csr_matrix((cost,target,offsets),shape=(len(coords),len(coords))).transpose().tocsr()
        return (np.asarray(matrix.indptr,dtype=np.int64),np.asarray(matrix.indices,dtype=np.int64),matrix.data)

    def graphs(self, chain):
        p=self.p;base=p.base_positions;n=len(p.positions)*self.stride
        if chain is not None:
            pairs=self.topology(([a,b] for a,targets in chain.items() for b in targets),{},lambda a,b:True)
            pairs.extend(self.rotations(chain))
            graph=self.assemble(pairs,chain,n)
            return [graph],[self.reverse(graph)],graph[1]
        self.prepare()
        # Only enumerate edges incident to query vertices. In particular, do not
        # enumerate each region's static clique again merely to discard it.
        groups=([a,b] for a in range(base,len(p.positions)) for r in p.owners[a]
                for b in p.region_positions[r] if a!=b)
        pairs=self.topology(groups,{},lambda a,b:True)
        pairs.extend(self.rotations(range(base,len(p.positions))))
        touched={node//self.stride for edge in pairs for node in edge}
        delta=self.assemble(pairs,touched,n)
        # Native search traverses the immutable map and tiny endpoint overlay
        # together; no per-click copy/merge/transpose of millions of map edges.
        coords=delta[1];coords[:len(self.base_graph[1])]=self.base_graph[1]
        return [self.base_graph,delta],[self.base_reverse,self.reverse(delta)],coords

    def run(self,start,goals,chain,timeout):
        from planner import NoPath
        began=time.monotonic();p=self.p
        graphs,reverse,coords=self.graphs(chain)
        graph_seconds=time.monotonic()-began
        n=len(coords);start_id=self.encode(start)
        goal_ids=np.asarray([self.encode(g) for g in goals],dtype=np.int64)
        # Stop reverse Dijkstra once the start is settled; cap all larger
        # distances at that bound. This remains admissible after lazy checks.
        rnative=(NativeGraph*len(reverse))(*(NativeGraph(len(o)-1,o.ctypes.data,t.ctypes.data,c.ctypes.data,None)
                                           for o,t,c in reverse))
        heuristic=np.empty(n,dtype=np.float64)
        goal_mask=np.zeros(n,dtype=np.uint8);goal_mask[goal_ids]=1
        relaxed_expanded=ctypes.c_int64()
        self.library.se2_goal_distances_graphs(n,rnative,len(reverse),goal_mask.ctypes.data,start_id,
            heuristic.ctypes.data,ctypes.byref(relaxed_expanded))
        heuristic_seconds=time.monotonic()-began-graph_seconds
        if not math.isfinite(heuristic[start_id]):
            raise NoPath('起点与目标的楼层/朝向在当前 SE(2) 图中不连通。')
        blocked_edges=[np.zeros(len(g[0]),dtype=np.uint8) for g in graphs]
        native=(NativeGraph*len(graphs))(*(NativeGraph(len(o)-1,o.ctypes.data,t.ctypes.data,c.ctypes.data,b.ctypes.data)
            for (_,_,c,o,t),b in zip(graphs,blocked_edges)))
        def block(u,v):
            for (_,_,_,offsets,target),blocked in zip(graphs,blocked_edges):
                if u+1>=len(offsets):continue
                index=offsets[u]+np.searchsorted(target[offsets[u]:offsets[u+1]],v)
                if index<offsets[u+1] and target[index]==v:blocked[index]=1
        blocked_nodes=np.zeros(n,dtype=np.uint8)
        # Reuse static-map rejections between clicks and between ASA stages.
        for node, valid in self.pose_cache.items():
            if not valid and self.encode(node)<n:
                blocked_nodes[self.encode(node)]=1
        for (a,b), valid in p.edge_cache.items():
            if valid:continue
            for u,v in ((self.encode(a),self.encode(b)),(self.encode(b),self.encode(a))):
                if u>=n or v>=n:continue
                block(u,v)
        output=np.empty(n,dtype=np.int64)
        expansions=0;passes=0
        while True:
            p.check_cancelled()
            if time.monotonic()-began>timeout:
                raise NoPath(f'规划超时（几何复核 {passes} 轮）；未发布未验证路径。')
            length=ctypes.c_int64();total=ctypes.c_double();expanded=ctypes.c_int64()
            code=self.library.se2_astar_graphs(n,native,len(graphs),blocked_nodes.ctypes.data,heuristic.ctypes.data,start_id,
                goal_mask.ctypes.data,output.ctypes.data,ctypes.byref(length),ctypes.byref(total),ctypes.byref(expanded))
            expansions+=expanded.value;passes+=1
            if code:
                self.last_failure=dict(passes=passes,expanded=expansions,blocked_nodes=int(blocked_nodes.sum()),
                                       blocked_edges=sum(int(b.sum()) for b in blocked_edges),start_blocked=int(blocked_nodes[start_id]),
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
            invalid_nodes=bool(invalid)
            # Validate the remaining usable portions in this same pass, rather
            # than discarding all edge work when one distant pose was rejected.
            edges=[(a,b) for a,b in zip(nodes,nodes[1:]) if self.pose_cache[a] and self.pose_cache[b]]
            unknown=[]
            for a,b in edges:
                key=tuple(sorted((a,b)))
                if key not in p.edge_cache:
                    if p.voxel_proof(a,b):p.edge_cache[key]=True
                    else:unknown.append(key)
            unknown=list(dict.fromkeys(unknown))
            for key,ok in zip(unknown,p.checker.motions([(p.pose(a),p.pose(b)) for a,b in unknown])):
                p.edge_cache[key]=bool(ok)
            invalid=False
            for a,b in edges:
                if p.edge_cache[tuple(sorted((a,b)))]:continue
                invalid=True
                for u,v in ((self.encode(a),self.encode(b)),(self.encode(b),self.encode(a))):
                    block(u,v)
            if not invalid and not invalid_nodes:
                return nodes,dict(cost=total.value,expanded=expansions,seconds=time.monotonic()-began,
                                  graph_seconds=graph_seconds,heuristic_seconds=heuristic_seconds,
                                  heuristic_expanded=relaxed_expanded.value,
                                  geometric_validation_passes=passes,search='native lazy A*')
