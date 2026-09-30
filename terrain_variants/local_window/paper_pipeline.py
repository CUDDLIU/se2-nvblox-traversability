"""Paper-guided SE(2) pipeline, separate from the legacy support heuristics.

Mesh -> two-level BVH -> local quantized spans -> distance/yaw feasibility ->
watershed/convex regions -> yaw graph. No coverage, plane-fit, crack-fill,
roughness residual, or depth-observation gate is used here.
"""
from dataclasses import dataclass, asdict
from collections import defaultdict, deque
import math
import time
import ctypes
import heapq
import numpy as np
from scipy.spatial import cKDTree
from paper_bvh import MeshIndex
from terrain import footprint_masks
from fast_raster import _lib, _dtype, evaluate_masks_native


@dataclass(frozen=True)
class Config:
    resolution: float = .10
    # 1 cm resolves the user-requested 3 cm step constraint. Paper uses 10 cm.
    vertical_resolution: float = .01
    tile_cells: int = 16
    slab_cells: int = 80
    length: float = .82
    width: float = .506
    height: float = .90
    side_margin: float = .08
    top_margin: float = .10
    max_slope_deg: float = 15.
    max_step: float = .03
    yaw_bins: int = 40
    sweep_sample_deg: float = 1.
    stair_riser_filter: bool = False
    surface_normal_filter: bool = False

    def __post_init__(self):
        if not all(math.isfinite(float(v)) for v in asdict(self).values()):
            raise ValueError('Nonfinite configuration')
        if min(self.resolution,self.vertical_resolution,self.length,self.width,self.height,self.sweep_sample_deg)<=0:
            raise ValueError('Dimensions must be positive')
        if min(self.side_margin,self.top_margin,self.max_step)<0 or not 0<=self.max_slope_deg<90:
            raise ValueError('Invalid robot constraints')
        if not 4<=self.yaw_bins<=64 or self.tile_cells<2 or self.slab_cells<2:
            raise ValueError('Invalid discretization')
        if any(int(v)!=v for v in (self.yaw_bins,self.tile_cells,self.slab_cells)):
            raise ValueError('Voxel counts and yaw bins must be integers')
        if self.max_step>0 and self.vertical_resolution>self.max_step:
            raise ValueError('Vertical resolution must resolve the selected step limit')

    @property
    def required_height(self):
        return self.height+self.top_margin

    @property
    def slab_size(self):
        return np.array([self.tile_cells*self.resolution]*2+
                        [self.slab_cells*self.vertical_resolution])

    @property
    def border(self):
        radius=math.hypot(self.length/2+self.side_margin,self.width/2+self.side_margin)
        xy=math.ceil(radius/self.resolution)*self.resolution+2*self.resolution
        # Floors can rise at every neighbor inside the rectangular mask. Include
        # this cumulative rise when querying ceilings near a slab's upper face.
        rise=(2*math.ceil(radius/self.resolution)+2)*self.max_step
        return np.array([xy,xy,self.required_height+rise+2*self.vertical_resolution])


@dataclass
class Span:
    ix: int
    iy: int
    iz: int
    z: float
    ceiling: float
    walkable: bool
    slope_ok: bool


@dataclass
class SlabResult:
    spans: list
    masks: np.ndarray
    distances: np.ndarray
    polygons: list
    solids: list
    reasons: list
    pose_states: object = None
    comfortable_masks: object = None


def voxelize_reference(vertices,triangles,config):
    """Surface voxelization, retaining all faces as occupied height intervals.

    The walkable flag comes from upward triangle normal and configured slope,
    as in a Recast heightfield. Face area coverage and fitted planes are absent.
    Each occupied interval is [lo,hi) in integer vertical coordinates.
    """
    vertices=np.ascontiguousarray(vertices,dtype=np.float64).reshape(-1,3)
    triangles=np.ascontiguousarray(triangles,dtype=np.int64).reshape(-1,3)
    if not np.isfinite(vertices).all():
        raise ValueError('Nonfinite mesh vertex')
    ptr,count=ctypes.c_void_p(),ctypes.c_size_t()
    code=_lib.raster(vertices.ctypes.data,len(vertices),triangles.ctypes.data,len(triangles),
                     config.resolution,math.cos(math.radians(config.max_slope_deg)),
                     ctypes.byref(ptr),ctypes.byref(count))
    if code:
        raise ValueError('Mesh rasterization failed')
    cells=defaultdict(list)
    dz=config.vertical_resolution
    try:
        if count.value:
            buffer=(ctypes.c_char*(count.value*_dtype.itemsize)).from_address(ptr.value)
            records=np.frombuffer(buffer,dtype=_dtype)
            for rec in records:
                lo=math.floor(float(rec['lo'])/dz+1e-6)
                hi=max(lo+1,math.ceil(float(rec['hi'])/dz-1e-6))
                cells[int(rec['ix']),int(rec['iy'])].append((lo,hi,bool(rec['support'])))
    finally:
        _lib.free_records(ptr)
    spans=[]; solids=[]
    for (ix,iy),intervals in sorted(cells.items()):
        merged=[]
        for lo,hi,walk in sorted(intervals):
            if not merged or lo>merged[-1][1]:
                merged.append([lo,hi,walk])
            else:
                top=merged[-1][1]
                # Surface at the top of the merged solid owns its slope flag.
                if hi>top:
                    merged[-1][2]=walk
                elif hi==top:
                    merged[-1][2] |= walk
                merged[-1][1]=max(top,hi)
        for j,(lo,hi,slope_ok) in enumerate(merged):
            ceiling=merged[j+1][0]*dz if j+1<len(merged) else math.inf
            z=hi*dz
            walkable=bool(slope_ok and ceiling-z>=config.required_height-1e-9)
            solids.append((ix,iy,lo,hi))
            spans.append(Span(ix,iy,hi,z,ceiling,walkable,bool(slope_ok)))
    return spans,solids


def voxelize(vertices,triangles,config):
    """Native equivalent of the reference surface interval construction."""
    from paper_native import voxel_spans, span_values
    records=voxel_spans(vertices,triangles,config.resolution,config.vertical_resolution,
                        math.cos(math.radians(config.max_slope_deg)),config.required_height)
    return records_to_spans(records,config)


def records_to_spans(records,config):
    from paper_native import span_values
    spans=[Span(ix,iy,iz,z,ceiling,walk,slope)
           for ix,iy,iz,z,ceiling,slope,walk in span_values(records,config.vertical_resolution)]
    solids=[(ix,iy,lo,hi) for ix,iy,lo,hi,ceiling,slope,walk in records.tolist()]
    return spans,solids


def local_boundary_distances(neighbors,invalid,xy,resolution,walk):
    """Smallest XY disk whose root-connected surface patch meets a boundary.

    Connectivity alone cannot identify a floor: stairs join overlapping floors.
    A boundary reached only by leaving the disk cannot invalidate that disk.
    """
    n=len(xy);distances=np.zeros(n)
    for root in np.flatnonzero(walk[:n]):
        queue=[(0.,int(root))];seen={int(root)};distances[root]=math.inf
        while queue:
            cost,i=heapq.heappop(queue)
            if invalid[i]:
                distances[root]=math.sqrt(cost);break
            for j in neighbors[i]:
                j=int(j)
                if j==n or j in seen:continue
                delta=(xy[j]-xy[root])*resolution
                seen.add(j);heapq.heappush(queue,(max(cost,float(delta@delta)),j))
    return distances


def classify_reference(spans,config,masks):
    """Distance to invalid surface + complete swept footprint on layered graph.

    Unknown/ledge boundaries are seeds at distance zero. Exact Euclidean
    distance is computed on the root-connected local surface patch. The
    inscribed disk rejects impossible states; full masks validate all others,
    including the exterior-disk fast-safe candidates (an extra verification,
    rather than unconditionally allowing them on folded multi-level surfaces).
    """
    n=len(spans)
    if not n:
        return np.zeros(0,dtype=np.uint64),np.zeros(0),[]
    by=defaultdict(list)
    for i,s in enumerate(spans):by[s.ix,s.iy].append(i)
    neighbors=np.full((n+1,4),n,dtype=np.int64)
    missing=np.zeros((n+1,4),dtype=np.uint8)
    walk=np.array([s.walkable for s in spans]+[False])
    height=config.required_height
    reason=['walkable' if s.walkable else 'slope' if not s.slope_ok else 'headroom' for s in spans]
    for i,s in enumerate(spans):
        for direction,(dx,dy) in enumerate(((1,0),(-1,0),(0,1),(0,-1))):
            adj=by.get((s.ix+dx,s.iy+dy),[])
            choices=[j for j in adj if spans[j].walkable and
                     abs(spans[j].z-s.z)<=config.max_step+1e-9 and
                     min(s.ceiling,spans[j].ceiling)-max(s.z,spans[j].z)>=height-1e-9]
            missing[i,direction]=not adj
            if s.walkable and len(choices)==1:
                neighbors[i,direction]=choices[0]
            elif s.walkable:
                reason[i]='unknown_boundary' if not adj else 'ledge_or_nonwalkable_neighbor'
    # Enforce reciprocal same-layer connectivity; no directed ledge bridges.
    for i in range(n):
        for d in range(4):
            j=neighbors[i,d]
            if j!=n and neighbors[j,d^1]!=i:
                neighbors[i,d]=n
                reason[i]='ambiguous_layer'
    invalid=~walk[:n] | np.any(neighbors[:n]==n,axis=1)
    xy=np.array([(s.ix,s.iy) for s in spans])
    distances=local_boundary_distances(neighbors,invalid,xy,config.resolution,walk)
    unique={}
    for yaw,mask in enumerate(masks):
        key=tuple(mask[0])
        if key not in unique:unique[key]=[mask,np.uint64(0)]
        unique[key][1] |= np.uint64(1)<<np.uint64(yaw)
    z=np.array([s.z for s in spans]+[math.inf])
    ceilings=np.array([s.ceiling for s in spans]+[-math.inf])
    # Enforce the complete swept footprint's common free vertical interval.
    # Per-span and edge overlap checks reject obvious ledges; this second
    # interval check also catches a higher floor and a low roof separated by
    # non-adjacent footprint cells.
    geometry,_,_=evaluate_masks_native(neighbors,missing,z,ceilings,
                                       np.full(n+1,math.inf),walk,unique.values(),height)
    rin=min(config.length/2+config.side_margin,config.width/2+config.side_margin)
    geometry[distances<rin-1e-9]=0
    full=(1<<config.yaw_bins)-1
    for i,g in enumerate(geometry):
        if g:reason[i]='safe' if int(g)==full else 'restricted'
        elif spans[i].walkable and not invalid[i]:reason[i]='footprint_or_boundary_distance'
    return geometry,distances,reason


def classify(spans,config,masks):
    from paper_native import span_neighbors
    n=len(spans)
    if not n:
        return np.zeros(0,dtype=np.uint64),np.zeros(0),[]
    xy=np.array([(s.ix,s.iy) for s in spans],dtype=np.int64)
    z=np.array([s.z for s in spans]+[math.inf])
    ceilings=np.array([s.ceiling for s in spans]+[-math.inf])
    walk=np.array([s.walkable for s in spans]+[False])
    slopes=np.array([s.slope_ok for s in spans])
    neighbors,missing,codes,components=span_neighbors(
        xy,z[:n],ceilings[:n],walk[:n],slopes,config.max_step,config.required_height)
    labels=('walkable','slope','headroom','unknown_boundary',
            'ledge_or_nonwalkable_neighbor','ambiguous_layer')
    reason=[labels[i] for i in codes]
    invalid=~walk[:n] | np.any(neighbors[:n]==n,axis=1)
    distances=local_boundary_distances(neighbors,invalid,xy,config.resolution,walk)
    unique={}
    for yaw,mask in enumerate(masks):
        key=tuple(mask[0])
        if key not in unique:unique[key]=[mask,np.uint64(0)]
        unique[key][1] |= np.uint64(1)<<np.uint64(yaw)
    geometry,_,_=evaluate_masks_native(neighbors,missing,z,ceilings,
        np.full(n+1,math.inf),walk,unique.values(),config.required_height)
    rin=min(config.length/2+config.side_margin,config.width/2+config.side_margin)
    geometry[distances<rin-1e-9]=0
    full=(1<<config.yaw_bins)-1
    for i,g in enumerate(geometry):
        if g:reason[i]='safe' if int(g)==full else 'restricted'
        elif spans[i].walkable and not invalid[i]:reason[i]='footprint_or_boundary_distance'
    return geometry,distances,reason


class Engine:
    def __init__(self,config=Config(),cache_raster=True,native_classify=True,native_mesh=False):
        self.config=config
        self.native_mesh=native_mesh
        if native_mesh:
            from native_mesh import NativeMesh
            if not cache_raster:raise ValueError('Native mesh requires cached raster path')
            self.index=NativeMesh()
        else:self.index=MeshIndex()
        self.masks=footprint_masks(config)
        from native_nav import Classifier
        self.classifier=Classifier(config,self.masks) if native_classify else None
        if native_mesh and self.classifier is not None:self.classifier.mesh=self.index
        self.results={}
        self.pending=set()
        self.generation=0
        self.revision=0
        self.signatures={}
        self.last_metrics={}
        self.completed_keys=[]
        self.dirty_keys=set()
        self._group_order={}
        self._group_sequence=0
        self.cache_raster=cache_raster
        self._process_turn=0
        self._span_signatures={}
        self._completed_groups=set()
        self._dependency_border=tuple(config.border)
        self._slab_size=tuple(config.slab_size)
        self.current_position=None
        self._local_window=set()

    def reconfigure(self, config):
        """Reclassify the retained Mesh with new physical constraints."""
        if (config.resolution, config.vertical_resolution, config.tile_cells, config.slab_cells) != (
                self.config.resolution, self.config.vertical_resolution,
                self.config.tile_cells, self.config.slab_cells):
            raise ValueError('Changing grid dimensions requires a restart')
        masks=footprint_masks(config)
        if self.classifier is not None:
            from native_nav import Classifier
            classifier=Classifier(config,masks)
            self.classifier=classifier
            if self.native_mesh:self.classifier.mesh=self.index
        self.config=config
        self.masks=masks
        self._dependency_border=tuple(config.border)
        self._slab_size=tuple(config.slab_size)
        self.pending.update(self.results)
        self._span_signatures.clear()
        if self.native_mesh:
            dirty,_=self.index.update_blocks({},config,reconfigure=True)
            self.pending.update(dirty)
        else:
            for block in self.index.blocks.values():
                block.raster_cache.clear()
                self.pending.update(self._affected(block.bounds()))
        self.dirty_keys=set(self.pending)
        for key in sorted(self.pending):
            if key[:2] not in self._group_order:
                self._group_order[key[:2]]=self._group_sequence
                self._group_sequence+=1
        self.revision+=1
        return set(self.pending)

    def _affected(self,bounds):
        if bounds is None:return set()
        low,high=bounds
        # Expand in XYZ by robot dependency extent, then floor to slab IDs.
        first=tuple(math.floor((float(low[i])-self._dependency_border[i])/self._slab_size[i]) for i in range(3))
        last=tuple(math.floor((float(high[i])+self._dependency_border[i])/self._slab_size[i]) for i in range(3))
        if math.prod(last[i]-first[i]+1 for i in range(3))>100000:
            raise ValueError('Mesh update extent too large')
        return {(x,y,z) for x in range(first[0],last[0]+1)
                for y in range(first[1],last[1]+1) for z in range(first[2],last[2]+1)}

    def update(self,blocks,clear=False):
        self.dirty_keys=set()
        if clear:
            self.index.clear();self.results.clear();self.pending.clear();self.signatures.clear()
            self._group_order.clear()
            self._span_signatures.clear()
            self._completed_groups.clear()
            self.generation+=1
        if self.native_mesh:
            if self.current_position is not None:
                p=np.asarray(self.current_position,dtype=float);size=self.config.slab_size
                self.index.set_dirty_window(np.floor((p-np.array([1.6,1.6,1.2]))/size).astype(np.int64),
                                            np.floor((p+np.array([1.6,1.6,.6]))/size).astype(np.int64))
            dirty,metrics=self.index.update_blocks(blocks,self.config)
            if self.current_position is not None:
                p=np.asarray(self.current_position,dtype=float);size=self.config.slab_size
                lo=np.floor((p-np.array([1.6,1.6,1.2]))/size).astype(np.int64)
                hi=np.floor((p+np.array([1.6,1.6,.6]))/size).astype(np.int64)
                window={(x,y,z) for x in range(lo[0],hi[0]+1) for y in range(lo[1],hi[1]+1)
                        for z in range(lo[2],hi[2]+1)}
                # Only current-window results are certified. Leaving results
                # receive explicit empty replacements and become history.
                dirty=(dirty & window)|(window-self._local_window)|(set(self.results)-window)
                self._local_window=window
            self.dirty_keys=dirty
            self.pending.update(dirty)
            for slab in sorted(dirty):
                if slab[:2] not in self._group_order:
                    self._group_order[slab[:2]]=self._group_sequence
                    self._group_sequence+=1
            if metrics['changed_blocks'] or clear:self.revision+=1
            return dict(**metrics,pending_slabs=len(self.pending))
        changed=skipped=0
        for key,(v,t) in blocks.items():
            v,t=MeshIndex._arrays(v,t)
            signature=(v.tobytes(),t.tobytes())
            if self.signatures.get(key)==signature or (not len(t) and key not in self.index.blocks):
                skipped+=1;continue
            old,new=self.index.replace_validated(key,v,t)
            self.signatures[key]=signature
            affected=self._affected(old)|self._affected(new)
            self.pending.update(affected)
            self.dirty_keys.update(affected)
            for slab in sorted(affected):
                if slab[:2] not in self._group_order:
                    self._group_order[slab[:2]]=self._group_sequence
                    self._group_sequence+=1
            changed+=1
        if changed or clear:self.revision+=1
        return dict(changed_blocks=changed,skipped_unchanged_blocks=skipped,pending_slabs=len(self.pending))

    def process(self,max_slabs=4,budget_ms=None,defer_polygons=False,priority_position=None):
        """One shared raster/context graph for the full dirty batch.

        A result completes every affected slab for this admitted input. The
        wall budget is measured, not used to hide pending work in later events.
        Native classification releases the GIL and parallelizes root masks.
        """
        from paper_polygons import build_polygons
        started=time.perf_counter(); self.completed_keys=[]
        if not self.pending or max_slabs<=0 or (budget_ms is not None and budget_ms<=0):
            return dict(processed_slabs=0,pending_slabs=len(self.pending),process_ms=0.)
        c=self.config;keys=sorted(self.pending);keyset=set(keys)
        if priority_position is None:
            # A local current-floor output needs a pose; do not silently treat
            # another floor as the current floor.
            return dict(processed_slabs=0,pending_slabs=len(keys),waiting_for_pose=True,process_ms=0.)
        p=np.asarray(priority_position,dtype=float)
        core_low=np.floor((p-np.array([1.6,1.6,1.2]))/c.slab_size).astype(np.int64)
        core_high=np.floor((p+np.array([1.6,1.6,.6]))/c.slab_size).astype(np.int64)
        low=core_low*c.slab_size-c.border
        high=(core_high+1)*c.slab_size+c.border
        stage=time.perf_counter()
        records=self.index.query_spans(low,high,c.resolution,c.vertical_resolution,
            math.cos(math.radians(80. if c.surface_normal_filter else c.max_slope_deg)),c.required_height)
        if len(records):
            records=records[(records['ix']*c.resolution>=low[0]) &
                (records['ix']*c.resolution<high[0]) &
                (records['iy']*c.resolution>=low[1]) &
                (records['iy']*c.resolution<high[1])]
        query_ms=(time.perf_counter()-stage)*1000
        stage=time.perf_counter()
        record_keys=np.column_stack((records['ix']//c.tile_cells,
            records['iy']//c.tile_cells,records['hi']//c.slab_cells))
        bounds=np.asarray(keys,dtype=np.int64)
        origin=bounds.min(axis=0);extent=bounds.max(axis=0)-origin+1
        stride=np.array([extent[1]*extent[2],extent[2],1],dtype=np.int64)
        codes=(record_keys-origin)@stride
        wanted=(bounds-origin)@stride
        core=np.all((record_keys>=core_low)&(record_keys<=core_high),axis=1)
        core &= np.all((record_keys>=origin)&(record_keys<origin+extent),axis=1)
        roots=np.flatnonzero(core & np.isin(codes,wanted))
        order=roots[np.argsort(codes[roots],kind='stable')]
        splits=np.flatnonzero(np.diff(codes[order]))+1
        selected={tuple(record_keys[group[0]].tolist()):group.tolist()
                  for group in np.split(order,splits) if len(group)}
        select_ms=(time.perf_counter()-stage)*1000
        stage=time.perf_counter()
        geometry,distances,reasons=self.classifier.from_records(records,roots)
        classify_ms=(time.perf_counter()-stage)*1000
        stage=time.perf_counter()
        # Debug solids are optional output and are omitted from the hot path.
        # All intervals still participate in native collision/headroom checks.
        values=dict(zip(roots.tolist(),records[roots].tolist()))
        max_int=np.iinfo(np.int64).max
        for key in keys:
            ids=selected.get(key,[])
            if ids:
                core=[Span(ix,iy,hi,hi*c.vertical_resolution,
                    math.inf if roof==max_int else roof*c.vertical_resolution,
                    bool(walk),bool(slope)) for ix,iy,lo,hi,roof,slope,walk in (values[i] for i in ids)]
                masks=geometry[ids];dist=distances[ids]
                polygons=build_polygons(core,self.classifier.comfortable[ids] if self.classifier.states is not None else masks,dist,c.resolution,key) if not defer_polygons else []
                self.results[key]=SlabResult(core,masks,dist,polygons,[],[reasons[i] for i in ids])
                if self.classifier.states is not None:
                    self.results[key].pose_states=self.classifier.states[ids].copy()
                    self.results[key].comfortable_masks=self.classifier.comfortable[ids].copy()
            else:self.results.pop(key,None)
        assemble_ms=(time.perf_counter()-stage)*1000
        self.completed_keys=keys;self.pending.clear();self._group_order.clear()
        elapsed=(time.perf_counter()-started)*1000
        self.last_metrics=dict(output_scope='robot-centered local window; outside retained as historical',
            core_bounds=[(core_low*c.slab_size).tolist(),((core_high+1)*c.slab_size).tolist()],
            processed_slabs=len(keys),pending_slabs=0,
            processed_xy_groups=len({k[:2] for k in keys}),queried_spans=len(records),
            classified_roots=len(roots),queried_triangles=self.index.stats['candidate_triangles'],
            query_spans_ms=query_ms,classify_ms=classify_ms,assemble_ms=assemble_ms,
            selection_ms=select_ms,process_ms=elapsed,
            budget_overrun_ms=max(0.,elapsed-budget_ms) if budget_ms is not None else 0.)
        return self.last_metrics

    def prepare_pose_graph(self):
        if not getattr(self,'native_mesh',False) or 'core_bounds' not in self.last_metrics:
            return None
        from pose_graph import capture
        bounds=np.asarray(self.last_metrics['core_bounds'])
        triangles=capture(self.index,bounds[0]-self.config.border,bounds[1]+self.config.border)
        return triangles,bounds

    def snapshot(self,rebuild_polygons=False):
        from paper_polygons import build_graph, build_polygons
        if self.pending:
            raise RuntimeError('Cannot publish a graph while affected slabs are pending')
        polygons=[p for key,result in sorted(self.results.items())
                  for p in (build_polygons(result.spans,result.comfortable_masks if result.comfortable_masks is not None else result.masks,result.distances,
                                            self.config.resolution,key)
                            if rebuild_polygons else result.polygons)]
        context=getattr(self,'pose_graph_context',None)
        if context is None and getattr(self,'native_mesh',False):context=self.prepare_pose_graph()
        if context is not None:
            from pose_graph import build
            graph=build(self.results,self.config,*context)
        else:
            graph=build_graph(polygons,self.config.yaw_bins,self.config.max_step,self.config.required_height)
        return dict(schema_version=2 if context is not None else 1,pipeline='paper_guided_se2_navmesh',
                    generation=self.generation,revision=self.revision,config=asdict(self.config),
                    polygons=polygons,graph=graph)
