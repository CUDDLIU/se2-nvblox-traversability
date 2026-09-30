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


def classify_reference(spans,config,masks):
    """Distance to invalid surface + complete swept footprint on layered graph.

    Unknown/ledge boundaries are seeds at distance zero. Exact Euclidean
    distance is computed within each connected walkable component. The
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
    distances=np.zeros(n)
    seen=set()
    for start in np.flatnonzero(walk[:n]):
        if int(start) in seen:continue
        todo=[int(start)];seen.add(int(start))
        for i in todo:
            for j in neighbors[i]:
                if j!=n and int(j) not in seen:
                    seen.add(int(j));todo.append(int(j))
        sources=[i for i in todo if invalid[i]]
        if not sources:
            distances[todo]=math.inf
        else:
            coords=np.array([(spans[i].ix,spans[i].iy) for i in todo])*config.resolution
            seeds=np.array([(spans[i].ix,spans[i].iy) for i in sources])*config.resolution
            distances[todo]=cKDTree(seeds).query(coords)[0]
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
    distances=np.zeros(n)
    coords=xy*config.resolution
    for label in np.unique(components):
        if label<0:continue
        selected=np.flatnonzero(components==label)
        sources=selected[invalid[selected]]
        distances[selected]=(cKDTree(coords[sources]).query(coords[selected])[0]
                             if len(sources) else math.inf)
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
            dirty,metrics=self.index.update_blocks(blocks,self.config)
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
        """Process whole XY tile groups, sharing work across dirty Z slabs.

        ``max_slabs`` is a soft completion budget: once a tile is selected,
        all its dirty vertical siblings finish together. This avoids repeating
        the same triangles/heightfield per vertical halo slab; the next tile
        is deferred after the slab budget is reached. A zero budget does no
        work. Geometry context spans the union of the selected Z slabs plus
        the existing robot dependency halo.
        """
        from paper_polygons import build_polygons
        started=time.perf_counter();done=0;triangles=0;groups_done=0;empty=0
        stage_ms=dict(query_spans_ms=0., classify_ms=0., assemble_ms=0.)
        reused_groups=0
        self.completed_keys=[]
        deadline=math.inf if budget_ms is None else started+max(0.,budget_ms)/1000
        c=self.config
        border=c.border
        size=c.slab_size
        groups=defaultdict(list)
        for key in sorted(self.pending):
            groups[key[:2]].append(key)
        self._process_turn+=1
        def priority(xy):
            age=self._group_order.get(xy,0)
            # Every fourth callback services the oldest group, guaranteeing
            # progress on distant history despite continuous nearby updates.
            if priority_position is None or self._process_turn%4==0:
                return (0,0,age)
            # Distance to tile extent, not center: a robot crossing a seam
            # needs both adjoining tiles. The old 4 m bucket gave a tile
            # under the robot the same priority as distant surfaces.
            dx=max(xy[0]*size[0]-priority_position[0],0.,priority_position[0]-(xy[0]+1)*size[0])
            dy=max(xy[1]*size[1]-priority_position[1],0.,priority_position[1]-(xy[1]+1)*size[1])
            ring=math.ceil(max(0.,math.hypot(dx,dy)-.8)/.8)
            # Empty vertical halo slabs legitimately have no result entry;
            # they must not make every previously computed tile look new.
            return (ring,0 if xy not in self._completed_groups else 1,age)
        for xy in sorted(groups,key=priority):
            keys=groups[xy]
            if (max_slabs<=0 or (budget_ms is None and done-empty>=max_slabs)
                    or time.perf_counter()>=deadline):
                break
            populated=[]
            for key in keys:
                low=np.asarray(key)*size;high=low+size
                # A top span can quantize up by one dz into the next slab.
                if self.index.has_mesh(low-[0,0,c.vertical_resolution],high):
                    populated.append(key)
                else:
                    self.results.pop(key,None)
                    self.pending.remove(key);done+=1;empty+=1
                    self.completed_keys.append(key)
            if not populated:
                groups_done+=1
                self._completed_groups.add(xy)
                self._group_order.pop(xy,None)
                continue
            low=np.asarray(populated[0])*size
            high=np.asarray(populated[-1])*size+size
            stage_start=time.perf_counter()
            if self.cache_raster:
                records=self.index.query_spans(low-border,high+border,c.resolution,
                            c.vertical_resolution,math.cos(math.radians(c.max_slope_deg)),c.required_height)
                triangles+=self.index.stats['candidate_triangles']
                span_signature=(tuple(populated),records.tobytes())
                if self._span_signatures.get(xy)==span_signature:
                    # Confirmed identical full query intervals under the same
                    # config: existing local masks remain exact for this input.
                    # Still finish provenance, so this is not a display replay.
                    stage_ms['query_spans_ms']+=(time.perf_counter()-stage_start)*1000
                    for key in populated:
                        self.pending.remove(key);self.completed_keys.append(key);done+=1
                    self._group_order.pop(xy,None);groups_done+=1;reused_groups+=1
                    self._completed_groups.add(xy)
                    if budget_ms is not None:break
                    continue
                if self.classifier is None:spans,solids=records_to_spans(records,c)
            else:
                v,t=self.index.query(low-border,high+border)
                triangles+=len(t)
                spans,solids=voxelize(v,t,c)
            stage_ms['query_spans_ms']+=(time.perf_counter()-stage_start)*1000
            stage_start=time.perf_counter()
            classify_ms=0.
            # Triangle AABB query is conservative: clip the rasterized columns
            # back to query XY bounds to bound heightfield work.
            selected=defaultdict(list)
            wanted={key[2] for key in populated}
            array_path=self.cache_raster and self.classifier is not None
            if array_path:
                rx,ry=records['ix']*c.resolution,records['iy']*c.resolution
                records=records[(rx>=low[0]-border[0]) & (rx<high[0]+border[0]) &
                                (ry>=low[1]-border[1]) & (ry<high[1]+border[1])]
                core_xy=(records['ix']//c.tile_cells==xy[0]) & (records['iy']//c.tile_cells==xy[1])
                zkeys=records['hi']//c.slab_cells
                for zkey in wanted:
                    selected[zkey]=np.flatnonzero(core_xy & (zkeys==zkey)).tolist()
                roots=[i for rows in selected.values() for i in rows]
                core_spans,solids_unused=records_to_spans(records[roots],c)
                spans=dict(zip(roots,core_spans))
                solids=[(ix,iy,lo,hi) for ix,iy,lo,hi,_,_,_ in records[core_xy].tolist()]
            else:
                spans=[s for s in spans if low[0]-border[0]<=s.ix*c.resolution<high[0]+border[0]
                       and low[1]-border[1]<=s.iy*c.resolution<high[1]+border[1]]
                for i,s in enumerate(spans):
                    zkey=s.iz//c.slab_cells
                    if (s.ix//c.tile_cells,s.iy//c.tile_cells)==xy and zkey in wanted:
                        selected[zkey].append(i)
            if any(selected.values()):
                classify_start=time.perf_counter()
                geometry,distances,reasons=(self.classifier.from_records(records,roots) if array_path else self.classifier(spans,
                    [i for rows in selected.values() for i in rows]) if self.classifier is not None
                    else classify(spans,c,self.masks))
                classify_ms=(time.perf_counter()-classify_start)*1000
                stage_ms['classify_ms']+=classify_ms
            else:
                geometry=np.zeros(len(records) if array_path else len(spans),dtype=np.uint64)
                distances=np.zeros(len(geometry));reasons=[]
            solid_groups=defaultdict(list)
            for solid in solids:
                ix,iy,lo,hi=solid
                if (ix//c.tile_cells,iy//c.tile_cells)!=xy:
                    continue
                for zkey in wanted:
                    if lo<(zkey+1)*c.slab_cells and hi>zkey*c.slab_cells:
                        solid_groups[zkey].append(solid)
            for key in populated:
                indices=selected[key[2]]
                core=[spans[i] for i in indices]
                masks=geometry[indices];dist=distances[indices]
                polygons=(build_polygons(core,masks,dist,c.resolution,key)
                          if indices and not defer_polygons else [])
                local_reasons=[reasons[i] for i in indices]
                local_solids=solid_groups[key[2]]
                if core or local_solids:
                    self.results[key]=SlabResult(core,masks,dist,polygons,local_solids,local_reasons)
                else:self.results.pop(key,None)
                self.pending.remove(key);done+=1
                self.completed_keys.append(key)
            stage_ms['assemble_ms']+=(time.perf_counter()-stage_start)*1000-classify_ms
            if self.cache_raster:self._span_signatures[xy]=span_signature
            groups_done+=1
            self._completed_groups.add(xy)
            self._group_order.pop(xy,None)
            # A wall-clock budget is used for the live local path.  One
            # populated XY tile is the atomic classification unit; deferring
            # the next tile guarantees that a cheap first tile cannot pull a
            # second expensive tile into the same callback and delay its
            # replacement event.
            if budget_ms is not None:
                break
        self.last_metrics=dict(processed_slabs=done,pending_slabs=len(self.pending),
                               processed_xy_groups=groups_done,empty_slabs=empty,
                               budget_overrun_ms=(max(0.,(time.perf_counter()-deadline)*1000)
                                                  if budget_ms is not None else 0.),
                               queried_triangles=triangles,process_ms=round((time.perf_counter()-started)*1000,1))
        self.last_metrics['unchanged_interval_groups']=reused_groups
        self.last_metrics.update({k:round(v,3) for k,v in stage_ms.items()})
        return self.last_metrics

    def snapshot(self,rebuild_polygons=False):
        from paper_polygons import build_graph, build_polygons
        if self.pending:
            raise RuntimeError('Cannot publish a graph while affected slabs are pending')
        polygons=[p for key,result in sorted(self.results.items())
                  for p in (build_polygons(result.spans,result.masks,result.distances,
                                            self.config.resolution,key)
                            if rebuild_polygons else result.polygons)]
        graph=build_graph(polygons,self.config.yaw_bins,self.config.max_step,self.config.required_height)
        return dict(schema_version=1,pipeline='paper_guided_se2_navmesh',
                    generation=self.generation,revision=self.revision,config=asdict(self.config),
                    polygons=polygons,graph=graph)
