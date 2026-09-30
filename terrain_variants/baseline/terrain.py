"""Conservative, multi-span geometry checks. No ROS or robot control dependency.

Mesh coordinates are world coordinates, z-up. Free-space observations are
separate from mesh occupancy: absence of a triangle never proves free space.
"""
from dataclasses import dataclass
from collections import defaultdict, deque
import math
import numpy as np
import shapely
from shapely.geometry import Polygon, box
from shapely.ops import unary_union


@dataclass(frozen=True)
class Config:
    resolution: float = 0.10
    vertical_resolution: float = 0.05
    length: float = 0.82
    width: float = 0.506
    height: float = 0.90
    side_margin: float = 0.08
    top_margin: float = 0.10
    max_slope_deg: float = 15.0
    candidate_slope_deg: float = 60.0
    max_plane_residual: float = 0.01
    max_step: float = 0.03
    ground_skin: float = 0.05
    support_gap_tolerance: float = 0.005
    min_support_fraction: float = 0.95
    yaw_bins: int = 40
    sweep_sample_deg: float = 1.0

    def __post_init__(self):
        values = vars(self)
        if not all(math.isfinite(float(x)) for x in values.values()):
            raise ValueError('Non-finite geometry parameter')
        if any(values[k] <= 0 for k in ('resolution', 'vertical_resolution',
                                       'length', 'width', 'height', 'sweep_sample_deg')):
            raise ValueError('Dimensions and resolutions must be positive')
        if any(values[k] < 0 for k in ('side_margin', 'top_margin', 'max_step', 'ground_skin')):
            raise ValueError('Margins must be nonnegative')
        if not 0 <= self.max_slope_deg < 90 or not 4 <= self.yaw_bins <= 64:
            raise ValueError('Invalid slope or yaw bins (4..64 supported)')
        if not 0 <= self.support_gap_tolerance <= .005 or not .95 <= self.min_support_fraction <= 1:
            raise ValueError('Support approximation limited to 5 mm and 5% area')
        if not self.max_slope_deg <= self.candidate_slope_deg < 90 or not 0 <= self.max_plane_residual <= .01:
            raise ValueError('Invalid support plane fitting bounds')


@dataclass
class Surface:
    ix: int
    iy: int
    z: float
    ceiling: float
    covered: bool


def clip_triangle(triangle, axis, boundary, keep_above):
    """Clip a 3D polygon against a vertical plane, preserving heights."""
    result = []
    if len(triangle) == 0:
        return result
    previous = triangle[-1]
    previous_inside = (previous[axis] >= boundary) if keep_above else (previous[axis] <= boundary)
    for current in triangle:
        inside = (current[axis] >= boundary) if keep_above else (current[axis] <= boundary)
        if inside != previous_inside:
            t = (boundary - previous[axis]) / (current[axis] - previous[axis])
            result.append(previous + t * (current - previous))
        if inside:
            result.append(current)
        previous, previous_inside = current, inside
    return result


def rasterize(vertices, triangles, config):
    """All triangles contribute collision intervals; only upward faces support.

    Exact XY clipping, not centroids. Polygon union later prevents duplicate or
    overlapping triangles from filling a genuinely missing piece of ground.
    """
    out = defaultdict(list)
    verts = np.asarray(vertices, dtype=float).reshape(-1, 3)
    ids = np.asarray(triangles, dtype=np.int64).reshape(-1, 3)
    if len(ids) and (ids.min() < 0 or ids.max() >= len(verts)):
        raise ValueError('Mesh triangle index outside vertex array')
    if not np.isfinite(verts).all():
        raise ValueError('Non-finite mesh vertex')
    r = config.resolution
    slope_cos = math.cos(math.radians(config.candidate_slope_deg))
    for indices in ids:
        tri = verts[indices]
        normal = np.cross(tri[1] - tri[0], tri[2] - tri[0])
        norm = np.linalg.norm(normal)
        if norm < 1e-12:
            continue
        support = normal[2] / norm >= slope_cos
        low = np.floor((tri[:, :2].min(axis=0) - 1e-9) / r).astype(int)
        high = np.floor((tri[:, :2].max(axis=0) + 1e-9) / r).astype(int)
        if np.prod(high - low + 1) > 100000:
            raise ValueError('Unbounded triangle raster extent')
        for ix in range(low[0], high[0] + 1):
            for iy in range(low[1], high[1] + 1):
                poly = list(tri)
                for axis, bound, above in ((0, ix*r, True), (0, (ix+1)*r, False),
                                           (1, iy*r, True), (1, (iy+1)*r, False)):
                    poly = clip_triangle(poly, axis, bound, above)
                if not poly:
                    continue
                xyz = np.asarray(poly)
                shape = Polygon(xyz) if len(poly) >= 3 and support else None
                if shape is not None and shape.area < 1e-12:
                    shape = None
                out[(ix, iy)].append((float(xyz[:, 2].min()),
                                     float(xyz[:, 2].max()), shape))
    return dict(out)


def classify_support_batch(patch_groups, cells, config):
    """Same support checks as the scalar reference, with batched GEOS/NumPy.

    Avoid thousands of Python/native/GIL transitions while ROS receives data.
    Batches are bounded by the caller to cap ragged polygon padding memory.
    """
    n = len(patch_groups)
    if not n:
        return np.zeros(0,dtype=bool)
    padded = np.empty((n,max(map(len,patch_groups))),dtype=object)
    padded[:] = None
    for i,patches in enumerate(patch_groups):
        padded[i,:len(patches)] = patches
    support = shapely.union_all(padded,axis=1)
    r = config.resolution
    cells = np.asarray(cells)
    area = shapely.area(support)
    complete = area >= r*r*(1-1e-5)
    partial = np.flatnonzero(~complete & (area >= r*r*config.min_support_fraction))
    if len(partial):
        xy = cells[partial]*r
        bounds = shapely.box(xy[:,0],xy[:,1],xy[:,0]+r,xy[:,1]+r)
        complete[partial] = shapely.covers(
            shapely.buffer(support[partial],config.support_gap_tolerance,quad_segs=16),bounds)
    ids = np.flatnonzero(complete)
    if not len(ids):
        return complete
    shapes, patch_cells = [], []
    for j,i in enumerate(ids):
        shapes.extend(patch_groups[i])
        patch_cells.extend([j]*len(patch_groups[i]))
    patch_cells = np.asarray(patch_cells)
    xyz, owners = shapely.get_coordinates(shapes,include_z=True,return_index=True)
    keep = np.r_[True,(owners[1:]!=owners[:-1]) | np.any(xyz[1:]!=xyz[:-1],axis=1)]
    keep &= np.r_[owners[1:]==owners[:-1],False]
    xyz, owners = xyz[keep],owners[keep]
    counts = np.bincount(owners,minlength=len(shapes))
    weights = (shapely.area(shapes)/counts)[owners]
    group = patch_cells[owners]
    # Normalized cell coordinates keep the 3x3 systems well-conditioned.
    xy = xyz[:,:2]/r-(cells[ids][group]+.5)
    design = np.c_[xy,np.ones(len(xy))]
    size = len(ids)
    gram = np.empty((size,3,3))
    rhs = np.empty((size,3))
    # Center z too, to preserve precision far from the world origin.
    total_weight = np.bincount(group,weights,minlength=size)
    mean_z = np.bincount(group,weights*xyz[:,2],minlength=size)/total_weight
    z = xyz[:,2]-mean_z[group]
    for a in range(3):
        rhs[:,a] = np.bincount(group,weights*design[:,a]*z,minlength=size)
        for b in range(3):
            gram[:,a,b] = np.bincount(group,weights*design[:,a]*design[:,b],minlength=size)
    good_rank = np.linalg.eigvalsh(gram)[:,0] > total_weight*1e-10
    coeff = np.zeros((size,3))
    coeff[good_rank] = np.linalg.solve(gram[good_rank],rhs[good_rank,:,None])[:,:,0]
    residual = np.zeros(size)
    np.maximum.at(residual,group,np.abs(np.sum(design*coeff[group],axis=1)-z))
    complete[ids] = (good_rank &
        (np.linalg.norm(coeff[:,:2],axis=1)/r <= math.tan(math.radians(config.max_slope_deg))+1e-9) &
        (residual <= config.max_plane_residual+1e-9))
    return complete


class Terrain:
    def __init__(self, config=Config()):
        self.config = config
        self.blocks = {}
        self.by_cell = defaultdict(dict)
        self.columns = {}
        self.dirty = set()

    def clear(self):
        self.blocks.clear()
        self.by_cell.clear()
        self.columns.clear()
        self.dirty.clear()

    def replace_block(self, key, records):
        for cell in self.blocks.pop(key, {}):
            self.by_cell[cell].pop(key, None)
            self.dirty.add(cell)
            if not self.by_cell[cell]:
                del self.by_cell[cell]
        if records:
            self.blocks[key] = records
            for cell, spans in records.items():
                self.by_cell[cell][key] = spans
                self.dirty.add(cell)

    def rebuild(self):
        patch_groups, candidates = [], []
        half = self.config.vertical_resolution/2
        for cell in self.dirty:
            records = sorted((rec for group in self.by_cell.get(cell,{}).values() for rec in group),key=lambda r:r[0])
            solids = []
            for lo,hi,poly in records:
                if not solids or lo-half > solids[-1][1]+1e-8:
                    solids.append([lo-half,hi+half,[(lo,hi,poly)]])
                else:
                    solids[-1][1] = max(solids[-1][1],hi+half)
                    solids[-1][2].append((lo,hi,poly))
            surfaces = []
            for index,(_,top,group) in enumerate(solids):
                patches = [(hi,p) for _,hi,p in group if p is not None and
                           top-half-hi <= self.config.vertical_resolution+1e-8]
                if not patches:
                    continue
                ceiling = solids[index+1][0] if index+1<len(solids) else math.inf
                surface = Surface(*cell,max(hi for hi,_ in patches),ceiling,False)
                surfaces.append(surface)
                candidates.append(surface)
                patch_groups.append([p for _,p in patches])
            if surfaces:
                self.columns[cell] = surfaces
            else:
                self.columns.pop(cell,None)
        for start in range(0,len(candidates),128):
            spans = candidates[start:start+128]
            complete = classify_support_batch(patch_groups[start:start+128],
                                               [(s.ix,s.iy) for s in spans],self.config)
            for surface,covered in zip(spans,complete):
                surface.covered = bool(covered)
        self.dirty.clear()

    def rebuild_reference(self):
        """Scalar oracle retained for batch implementation regression tests."""
        for cell in self.dirty:
            records = [rec for group in self.by_cell.get(cell, {}).values() for rec in group]
            records.sort(key=lambda rec: rec[0])
            solids = []
            # Surface thickness: one half vertical voxel on either side.
            # This is an uncertainty dilation, not an assumption about interiors.
            half = self.config.vertical_resolution / 2
            for lo, hi, poly in records:
                if not solids or lo - half > solids[-1][1] + 1e-8:
                    solids.append([lo-half, hi+half, [(lo, hi, poly)]])
                else:
                    solids[-1][1] = max(solids[-1][1], hi+half)
                    solids[-1][2].append((lo, hi, poly))
            surfaces = []
            for index, (_, top, group) in enumerate(solids):
                patches = [(hi, p) for _, hi, p in group if p is not None
                           and top - half - hi <= self.config.vertical_resolution + 1e-8]
                if not patches:
                    continue
                support = unary_union([p for _, p in patches])
                coverage = support.area
                r = self.config.resolution
                cell_box = box(cell[0]*r, cell[1]*r, (cell[0]+1)*r, (cell[1]+1)*r)
                complete = coverage >= r*r*(1-1e-5)
                # Only close sub-voxel cracks when actual support already
                # occupies at least 95% of this cell. Never invent a cell.
                if not complete and coverage >= r*r*self.config.min_support_fraction:
                    complete = support.buffer(self.config.support_gap_tolerance).covers(cell_box)
                # Fit the actual clipped XYZ patches at navigation resolution.
                # Individual noisy TSDF facets are not the terrain slope.
                # Area weighting avoids letting tiny triangles dominate. Every
                # vertex must remain within the explicit 1 cm roughness bound.
                if complete:
                    shapes = [patch for _,patch in patches]
                    xyz, owners = shapely.get_coordinates(shapes, include_z=True, return_index=True)
                    # Clip output contains consecutive padding vertices and
                    # a closing vertex. Remove both in bulk, keeping the exact
                    # same unique vertices and area weights as the reference.
                    keep = np.r_[True, (owners[1:] != owners[:-1]) |
                                 np.any(xyz[1:] != xyz[:-1],axis=1)]
                    keep &= np.r_[owners[1:] == owners[:-1], False]
                    xyz, owners = xyz[keep], owners[keep]
                    counts = np.bincount(owners,minlength=len(shapes))
                    weights = (shapely.area(shapes)/counts)[owners]
                    design = np.c_[xyz[:,:2]-[(cell[0]+.5)*r,(cell[1]+.5)*r], np.ones(len(xyz))]
                    weight = np.sqrt(weights)
                    coeff, _, rank, _ = np.linalg.lstsq(design*weight[:,None],xyz[:,2]*weight,rcond=None)
                    complete = (rank == 3 and
                                np.linalg.norm(coeff[:2]) <= math.tan(math.radians(self.config.max_slope_deg))+1e-9 and
                                np.max(np.abs(design@coeff-xyz[:,2])) <= self.config.max_plane_residual+1e-9)
                # z is a conservative upper support height for this cell.
                z = max(h for h, _ in patches)
                ceiling = solids[index+1][0] if index+1 < len(solids) else math.inf
                surfaces.append(Surface(*cell, z, ceiling,
                    bool(complete)))
            if surfaces:
                self.columns[cell] = surfaces
            else:
                self.columns.pop(cell, None)
        self.dirty.clear()

    def local_surfaces(self, x, y, radius):
        r = self.config.resolution
        return [s for cell, spans in sorted(self.columns.items())
                if abs((cell[0]+.5)*r-x) <= radius and abs((cell[1]+.5)*r-y) <= radius
                for s in spans]


def footprint_masks(config):
    """Rasterized swept rectangles including *all intersecting grid cells*.

    The sampled rotation union is inflated by the maximum intermediate vertex
    displacement R*delta_angle/2. Thus angular sampling cannot miss a corner
    between samples. Each channel covers yaw +/- half the channel interval.
    """
    a = config.length/2 + config.side_margin
    b = config.width/2 + config.side_margin
    interval = 2*math.pi/config.yaw_bins
    count = max(1, math.ceil(interval/math.radians(config.sweep_sample_deg)))
    inflation = math.hypot(a, b) * interval / count / 2
    a, b = a+inflation, b+inflation
    radius = math.hypot(a, b)
    n = math.ceil(radius/config.resolution + .5)
    offsets = np.array([(x, y) for x in range(-n, n+1) for y in range(-n, n+1)], dtype=int)
    centers = offsets*config.resolution
    half = config.resolution/2
    masks = []
    for yaw_i in range(config.yaw_bins):
        inside = np.zeros(len(offsets), dtype=bool)
        for angle in np.linspace(yaw_i*interval-interval/2, yaw_i*interval+interval/2, count+1):
            c, s = math.cos(angle), math.sin(angle)
            # Separating axis theorem: rectangle vs axis-aligned square.
            hit = (np.abs(centers[:, 0]) <= a*abs(c)+b*abs(s)+half+1e-10)
            hit &= np.abs(centers[:, 1]) <= a*abs(s)+b*abs(c)+half+1e-10
            hit &= np.abs(centers[:, 0]*c+centers[:, 1]*s) <= a+half*(abs(c)+abs(s))+1e-10
            hit &= np.abs(-centers[:, 0]*s+centers[:, 1]*c) <= b+half*(abs(c)+abs(s))+1e-10
            inside |= hit
        mask = {tuple(p) for p in offsets[inside]}
        # A spanning tree ensures every footprint cell is reached via adjacent
        # height-compatible surfaces. Extra adjacency checks reject ambiguity.
        order = [(0, 0)]
        parents = [-1]
        directions = [-1]
        lookup = {(0, 0): 0}
        for p in order:
            for direction, (dx, dy) in enumerate(((1, 0), (-1, 0), (0, 1), (0, -1))):
                q = (p[0]+dx, p[1]+dy)
                if q in mask and q not in lookup:
                    lookup[q] = len(order)
                    order.append(q)
                    parents.append(lookup[p])
                    directions.append(direction)
        if len(order) != len(mask):
            raise ValueError('Footprint mask is disconnected')
        checks = [(i, lookup[(x+dx, y+dy)], d)
                  for i, (x, y) in enumerate(order)
                  for d, (dx, dy) in enumerate(((1, 0), (-1, 0), (0, 1), (0, -1)))
                  if (x+dx, y+dy) in lookup and i < lookup[(x+dx, y+dy)]]
        masks.append((order, parents, directions, checks))
    return masks


def evaluate(surfaces, config, masks, observed_tops, diagnostics=None, *, native=True):
    """Return geometry-only and observed-free yaw bitmasks per span.

    Unknown support is blocked for both. Unobserved body volume may pass the
    geometry-only test, but can never pass the observed-free test.
    """
    n = len(surfaces)
    geometric = np.zeros(n, dtype=np.uint64)
    verified = np.zeros(n, dtype=np.uint64)
    unknown_support = np.zeros(n, dtype=np.uint64)
    if diagnostics is not None:
        diagnostics['unknown_support'] = unknown_support
    if n == 0:
        return geometric, verified
    cell_ids = defaultdict(list)
    for i, surface in enumerate(surfaces):
        cell_ids[(surface.ix, surface.iy)].append(i)
    z = np.array([s.z for s in surfaces] + [math.inf])
    ceiling = np.array([s.ceiling for s in surfaces] + [-math.inf])
    known_top = np.r_[np.asarray(observed_tops), -math.inf]
    covered = np.array([s.covered for s in surfaces] + [False])
    neighbors = np.full((n+1, 4), n, dtype=int)
    missing = np.ones((n+1, 4), dtype=bool)
    missing[n] = False
    for i, surface in enumerate(surfaces):
        for d, (dx, dy) in enumerate(((1, 0), (-1, 0), (0, 1), (0, -1))):
            adjacent = cell_ids.get((surface.ix+dx, surface.iy+dy), [])
            missing[i, d] = not adjacent or any(not covered[j] for j in adjacent)
            choices = [j for j in cell_ids.get((surface.ix+dx, surface.iy+dy), [])
                       if abs(z[j]-surface.z) <= config.max_step + 1e-9 and covered[j]]
            if len(choices) == 1:
                neighbors[i, d] = choices[0]
    required_height = config.height + config.top_margin
    # A rectangular footprint at yaw and yaw+pi has the same occupied cells.
    # Group identical masks, retaining every original channel bit exactly.
    unique_masks = {}
    for yaw_i,mask in enumerate(masks):
        key = tuple(mask[0])
        if key not in unique_masks:
            unique_masks[key] = [mask,np.uint64(0)]
        unique_masks[key][1] |= np.uint64(1) << np.uint64(yaw_i)
    if native:
        from fast_raster import evaluate_masks_native
        geometric,verified,unknown_support=evaluate_masks_native(
            neighbors,missing,z,ceiling,known_top,covered,unique_masks.values(),required_height)
        if diagnostics is not None:
            diagnostics['unknown_support']=unknown_support
        return geometric,verified
    for (order, parents, directions, checks),bit in unique_masks.values():
        mapped = np.full((len(order), n), n, dtype=int)
        mapped[0] = np.arange(n)
        uncertain = ~covered[:n].copy()
        for i in range(1, len(order)):
            uncertain |= missing[mapped[parents[i]], directions[i]]
            mapped[i] = neighbors[mapped[parents[i]], directions[i]]
        ok = covered[mapped].all(axis=0)
        # Avoid mixing different vertical branches at cycles in the footprint.
        for a, b, direction in checks:
            ok &= neighbors[mapped[a], direction] == mapped[b]
        maximum_floor = z[mapped].max(axis=0)
        minimum_ceiling = ceiling[mapped].min(axis=0)
        ok &= minimum_ceiling >= maximum_floor + required_height
        unknown_support[uncertain & ~ok] |= bit
        geometric[ok] |= bit
        free_ok = ok & (known_top[mapped].min(axis=0) >= maximum_floor + required_height)
        verified[free_ok] |= bit
    return geometric, verified


def geometry_display_state(geometry_mask, unknown_support_mask, yaw_bins, valid=True):
    """0=safe all yaw, 1=restricted yaw, 2=reject, 3=unknown, 4=stale.

    Depth/free-space observations deliberately do not participate in this
    Mesh-based classification. All channels include their continuous sweep.
    """
    if not valid:
        return 4
    if int(geometry_mask) == (1 << yaw_bins)-1:
        return 0
    if geometry_mask:
        return 1
    return 3 if unknown_support_mask else 2


def invalidate_occupied_evidence(evidence, terrain, dirty):
    """Keep observations across mesh refreshes except newly occupied voxels.

    Occupancy includes every face (not only floor patches), dilated by half
    a vertical voxel. Deleted support still fails the separate footprint test.
    """
    dz = terrain.config.vertical_resolution
    intervals = {}
    for cell in dirty:
        raw = sorted((lo-dz/2,hi+dz/2)
                     for group in terrain.by_cell.get(cell,{}).values() for lo,hi,_ in group)
        merged = []
        for lo,hi in raw:
            if not merged or lo>merged[-1][1]:
                merged.append([lo,hi])
            else:
                merged[-1][1] = max(hi,merged[-1][1])
        intervals[cell] = merged
    return {k:t for k,t in evidence.items()
            if not any(k[2]*dz <= hi and (k[2]+1)*dz >= lo
                       for lo,hi in intervals.get(k[:2],()))}


def required_free_keys(surfaces, config):
    """Query body-height columns plus an allowance for local sloping support."""
    dz = config.vertical_resolution
    result = set()
    top_extra = config.height + config.top_margin + config.length*math.tan(math.radians(config.max_slope_deg))
    for s in surfaces:
        if not s.covered:
            continue
        first = math.ceil((s.z+config.ground_skin)/dz - 1e-9)
        last = math.ceil((min(s.z+top_extra, s.ceiling))/dz)
        result.update((s.ix, s.iy, k) for k in range(first, last))
    return np.asarray(sorted(result), dtype=np.int32).reshape(-1, 3)


def certify_free(keys, depth, intrinsics, world_to_camera, config, depth_margin=.04, max_range=4.0):
    """Conservatively certify complete navigation voxels from one depth frame.

    All eight corners must project into the image and lie in front of every
    depth pixel in an enclosing image rectangle. Invalid depth and image
    borders reject evidence. This is deliberately stronger than ray carving
    a voxel touched by one ray. Distortion must be rectified by the caller.
    """
    if not len(keys):
        return np.zeros(0, dtype=bool)
    scale = np.array([config.resolution, config.resolution, config.vertical_resolution])
    corners = np.array([(x, y, z) for x in (0, 1) for y in (0, 1) for z in (0, 1)])
    world = (keys[:, None, :] + corners[None, :, :])*scale
    camera = world @ world_to_camera[:3, :3].T + world_to_camera[:3, 3]
    zs = camera[:, :, 2]
    fx, fy, cx, cy = intrinsics
    uv = camera[:, :, :2] / np.maximum(zs[:, :, None], 1e-8)
    uv = uv*np.array([fx, fy]) + np.array([cx, cy])
    lo = np.floor(uv.min(axis=1)).astype(np.int64)
    hi = np.ceil(uv.max(axis=1)).astype(np.int64)
    h, w = depth.shape
    valid = (zs.min(axis=1) > .1) & (zs.max(axis=1) <= max_range)
    valid &= (lo[:, 0] >= 0) & (lo[:, 1] >= 0) & (hi[:, 0] < w) & (hi[:, 1] < h)
    certified = np.zeros(len(keys), dtype=bool)
    ids = np.flatnonzero(valid)
    if not len(ids):
        return certified
    from fast_raster import rectangles_clear
    certified[ids] = rectangles_clear(depth,lo[ids],hi[ids],zs[ids].max(axis=1)+depth_margin)
    return certified


def observed_clearance_tops(surfaces, evidence, config, now, ttl):
    dz = config.vertical_resolution
    tops = []
    for s in surfaces:
        first = math.ceil((s.z+config.ground_skin)/dz - 1e-9)
        k = first
        while now - evidence.get((s.ix, s.iy, k), -math.inf) <= ttl:
            k += 1
        # If no voxel is proven free, do not claim even the contact band.
        tops.append(k*dz if k > first else s.z)
    return np.asarray(tops)
