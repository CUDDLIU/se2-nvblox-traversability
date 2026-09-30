"""Grid spans -> convex SE(2) polygons and a portal state graph.

This is a paper-inspired implementation, not the authors' unpublished code.
Distance maxima seed a four-neighbour watershed.  Exact grid-cell contours are
ear-clipped and adjacent triangles are merged only when their union is convex.
Contours with holes (or a failed ear clipping) use exact cell triangulation;
there is no contour simplification, hole filling, or minimum-region pruning.

Graph translation edges are centroid-to-portal spokes inside convex polygons.
They preserve connectivity but are not the paper's complete visibility graph.
All geometry is z-up; a missing upper surface is represented by ceiling=inf.
Each accepted discrete span is represented by its entire XY square.  This is a
voxel NavMesh abstraction: it does not independently certify every continuous
center location inside the square from a mask evaluated only at its center.
"""
from collections import defaultdict, deque
import hashlib
import heapq
import math

from shapely.geometry import Point, Polygon, box
from shapely.geometry.polygon import orient
from shapely.ops import unary_union
from shapely.strtree import STRtree


_DIRECTIONS = ((-1, 0), (0, -1), (0, 1), (1, 0))
_EPS = 1e-9


def _same_distance(a, b):
    """Equality for distance-field plateaus, including an unbounded (+inf) field."""
    return a == b or (math.isfinite(a) and math.isfinite(b) and abs(a - b) <= 1e-10)


def _neighbours(key):
    x, y = key
    return ((x + dx, y + dy) for dx, dy in _DIRECTIONS)


def _watershed(cells, distances):
    """Return connected basin cell sets, keeping maxima plateaus intact."""
    pending = set(cells)
    seeds = []
    while pending:
        start = min(pending)
        pending.remove(start)
        plateau = {start}
        queue = deque([start])
        maximum = True
        while queue:
            p = queue.popleft()
            for q in _neighbours(p):
                if q not in cells:
                    continue
                if distances[q] > distances[start] + 1e-10:
                    maximum = False
                if q in pending and _same_distance(distances[q], distances[start]):
                    pending.remove(q)
                    plateau.add(q)
                    queue.append(q)
        if maximum:
            seeds.append(plateau)
    seeds.sort(key=lambda points: min(points))
    labels, basins, frontier = {}, [], []
    for label, seed in enumerate(seeds):
        basins.append(set())
        for p in sorted(seed):
            labels[p] = label
            basins[label].add(p)
            heapq.heappush(frontier, (-distances[p], label, p))
    while frontier:
        _, label, p = heapq.heappop(frontier)
        for q in _neighbours(p):
            if q not in cells or q in labels:
                continue
            labels[q] = label
            basins[label].add(q)
            heapq.heappush(frontier, (-distances[q], label, q))
    if len(labels) != len(cells):
        raise RuntimeError('Watershed did not cover its input cells')
    return basins


def _cross(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _ring(poly):
    points = list(orient(poly, sign=1).exterior.coords)[:-1]
    # Remove collinear grid contour vertices before ear clipping.
    changed = True
    while changed and len(points) > 3:
        changed = False
        for i in range(len(points)):
            if abs(_cross(points[i - 1], points[i], points[(i + 1) % len(points)])) < 1e-12:
                del points[i]
                changed = True
                break
    return points


def _ear_clip(poly):
    """Triangulate a simple contour; return None for the safe grid fallback."""
    if poly.interiors:
        return None
    points = _ring(poly)
    output = []
    while len(points) > 3:
        found = False
        for i in range(len(points)):
            a, b, c = points[i - 1], points[i], points[(i + 1) % len(points)]
            if _cross(a, b, c) <= 1e-12:
                continue
            tri = Polygon([a, b, c])
            if any(tri.covers(Point(p)) for j, p in enumerate(points)
                   if j not in {(i - 1) % len(points), i, (i + 1) % len(points)}):
                continue
            output.append(tri)
            del points[i]
            found = True
            break
        if not found:
            return None
    if len(points) == 3:
        output.append(Polygon(points))
    if not output or unary_union(output).symmetric_difference(poly).area > _EPS:
        return None
    return output


def _convex_merge(pieces):
    """Greedy, deterministic batches of exactly area-preserving convex merges."""
    pieces = sorted(pieces, key=lambda p: (p.bounds, p.area))
    while len(pieces) > 1:
        tree = STRtree(pieces)
        consumed, merged = set(), []
        for i, a in enumerate(pieces):
            if i in consumed:
                continue
            partner = None
            for raw_j in sorted(tree.query(a)):
                j = int(raw_j)
                if j <= i or j in consumed:
                    continue
                b = pieces[j]
                if a.boundary.intersection(b.boundary).length <= 1e-10:
                    continue
                hull = unary_union([a, b]).convex_hull
                tolerance = _EPS * max(1.0, a.area + b.area)
                if abs(hull.area - a.area - b.area) > tolerance:
                    continue
                # Area equality alone is insufficient for overlapping inputs.
                if hull.symmetric_difference(unary_union([a, b])).area > tolerance:
                    continue
                partner = (j, hull)
                break
            consumed.add(i)
            if partner is None:
                merged.append(a)
            else:
                consumed.add(partner[0])
                merged.append(partner[1])
        if len(merged) == len(pieces):
            break
        pieces = sorted(merged, key=lambda p: (p.bounds, p.area))
    return pieces


def _canonical_vertices(poly, z):
    points = _ring(poly)
    first = min(range(len(points)), key=lambda i: points[i])
    points = points[first:] + points[:first]
    return [[float(x), float(y), float(z)] for x, y in points]


def build_polygons(spans, masks, distances, resolution, tile_key):
    """Build convex polygons for one tile/slab core, without any halo output.

    Polygons never cross a quantized floor level, yaw-mask boundary, hole, or
    watershed region.  ``cells`` lists every source cell overlapping a polygon;
    triangles can share a cell.  ``ceiling`` is the minimum ceiling in its cells.
    IDs encode geometry and yaw mask, so unchanged polygons keep their IDs.
    """
    if resolution <= 0 or not math.isfinite(resolution):
        raise ValueError('resolution must be finite and positive')
    if len(spans) != len(masks) or len(spans) != len(distances):
        raise ValueError('spans, masks, and distances must have equal lengths')
    groups, values = defaultdict(dict), defaultdict(dict)
    for span, mask, distance in zip(spans, masks, distances):
        mask = int(mask)
        if not span.walkable or not mask:
            continue
        if mask < 0:
            raise ValueError('yaw masks must be nonnegative')
        key, xy = (int(span.iz), mask), (int(span.ix), int(span.iy))
        if xy in groups[key]:
            raise ValueError('Duplicate span at one quantized floor cell')
        groups[key][xy] = span
        value = float(distance)
        values[key][xy] = value if math.isfinite(value) else 0.0
    result = []
    prefix = ','.join(map(str, tile_key)) if isinstance(tile_key, (tuple, list)) else str(tile_key)
    for (iz, mask), cells in sorted(groups.items()):
        for basin in _watershed(cells, values[(iz, mask)]):
            seed = min(basin)
            region = f'{prefix}/{iz}/{mask:x}/{seed[0]},{seed[1]}'
            cell_keys = sorted(basin)
            squares = [box(x * resolution, y * resolution,
                           (x + 1) * resolution, (y + 1) * resolution)
                       for x, y in cell_keys]
            contour = unary_union(squares)
            components = [contour] if contour.geom_type == 'Polygon' else list(contour.geoms)
            pieces = []
            for component in components:
                triangles = _ear_clip(component)
                if triangles is None:
                    triangles = []
                    for square in squares:
                        if component.intersection(square).area < square.area * .5:
                            continue
                        x0, y0, x1, y1 = square.bounds
                        triangles.extend([Polygon([(x0, y0), (x1, y0), (x1, y1)]),
                                          Polygon([(x0, y0), (x1, y1), (x0, y1)])])
                pieces.extend(_convex_merge(triangles))
            square_index = STRtree(squares)
            for polygon in pieces:
                owners = [int(i) for i in square_index.query(polygon)
                          if polygon.intersection(squares[int(i)]).area > 1e-12]
                source = [cells[cell_keys[i]] for i in sorted(owners)]
                floor = max(float(s.z) for s in source)
                ceiling = min(float(s.ceiling) for s in source)
                vertices = _canonical_vertices(polygon, floor)
                signature = ';'.join(f'{x:.12g},{y:.12g},{z:.12g}' for x, y, z in vertices)
                digest = hashlib.sha256(f'{iz}:{mask}:{signature}'.encode()).hexdigest()[:16]
                result.append({'id': f'{prefix}/{digest}', 'vertices': vertices,
                               'yaw_mask': mask, 'cells': [[int(s.ix), int(s.iy), int(s.iz)] for s in source],
                               'region': region, 'floor': floor, 'ceiling': ceiling})
    return sorted(result, key=lambda p: p['id'])


def _line_parts(geometry):
    if geometry.geom_type == 'LineString':
        return [geometry] if geometry.length > 1e-10 else []
    if geometry.geom_type in ('MultiLineString', 'GeometryCollection'):
        return [line for part in geometry.geoms for line in _line_parts(part)]
    return []


def build_graph(polygons, yaw_bins, max_step, height):
    """Build an undirected portal graph; each node represents one yaw channel.

    Both consecutive channel bits are required for a rotation edge.  The input
    channel masks must already validate the entire continuous yaw interval.
    Cross-region/tile/slab edges require a shared segment (point contacts do not
    count), compatible floors, and a common free interval of at least height.
    """
    if not 1 <= yaw_bins <= 64 or max_step < 0 or height <= 0:
        raise ValueError('Invalid graph robot parameters')
    if len({p['id'] for p in polygons}) != len(polygons):
        raise ValueError('Polygon IDs must be globally unique')
    full_mask = (1 << yaw_bins) - 1
    shapes = [Polygon([(v[0], v[1]) for v in p['vertices']]) for p in polygons]
    for shape in shapes:
        if not shape.is_valid or shape.area <= 0 or shape.interiors:
            raise ValueError('Graph input must contain valid simple polygons')
        if shape.convex_hull.area - shape.area > _EPS:
            raise ValueError('Graph translation requires convex input polygons')
    nodes, edges, states = {}, {}, {}

    def add_states(location_id, xyz, mask, owners, kind):
        mask &= full_mask
        by_yaw = {}
        for yaw in range(yaw_bins):
            if mask & (1 << yaw):
                node_id = f'{location_id}/y{yaw}'
                by_yaw[yaw] = node_id
                nodes[node_id] = {'id': node_id, 'position': list(xyz), 'yaw': yaw,
                                  'yaw_rad': yaw * 2 * math.pi / yaw_bins,
                                  'polygons': owners, 'kind': kind}
        states[location_id] = by_yaw
        for yaw, node_id in by_yaw.items():
            other = by_yaw.get((yaw + 1) % yaw_bins)
            if other is not None and other != node_id:
                add_edge(node_id, other, 'rotation', owners)
        return by_yaw

    def add_edge(a, b, kind, owners):
        key = tuple(sorted((a, b)))
        if key in edges:
            return
        pa, pb = nodes[a]['position'], nodes[b]['position']
        length = math.dist(pa, pb)
        edges[key] = {'source': key[0], 'target': key[1], 'kind': kind,
                      'polygons': owners, 'length': length,
                      'angle': 2 * math.pi / yaw_bins if kind == 'rotation' else 0.0}

    hubs = []
    for polygon, shape in zip(polygons, shapes):
        floor = float(polygon.get('floor', max(v[2] for v in polygon['vertices'])))
        ceiling = float(polygon.get('ceiling', math.inf))
        mask = int(polygon['yaw_mask']) if ceiling - floor >= height - _EPS else 0
        center = shape.centroid
        hubs.append(add_states(f"{polygon['id']}/center", [center.x, center.y, floor],
                               mask, [polygon['id']], 'center'))
    tree = STRtree(shapes)
    for i, a in enumerate(polygons):
        az = float(a.get('floor', max(v[2] for v in a['vertices'])))
        for raw_j in sorted(tree.query(shapes[i])):
            j = int(raw_j)
            if j <= i:
                continue
            b = polygons[j]
            bz = float(b.get('floor', max(v[2] for v in b['vertices'])))
            if abs(az - bz) > max_step + _EPS:
                continue
            top = min(float(a.get('ceiling', math.inf)), float(b.get('ceiling', math.inf)))
            if top - max(az, bz) < height - _EPS:
                continue
            mask = int(a['yaw_mask']) & int(b['yaw_mask']) & full_mask
            if not mask:
                continue
            shared = shapes[i].boundary.intersection(shapes[j].boundary)
            segments = sorted(_line_parts(shared), key=lambda segment: segment.bounds)
            for k, segment in enumerate(segments):
                center = segment.interpolate(.5, normalized=True)
                owners = sorted([a['id'], b['id']])
                location_id = f'portal/{owners[0]}|{owners[1]}/{k}'
                portal = add_states(location_id, [center.x, center.y, max(az, bz)],
                                    mask, owners, 'portal')
                for yaw, portal_id in portal.items():
                    for index, polygon in ((i, a), (j, b)):
                        if yaw in hubs[index]:
                            add_edge(hubs[index][yaw], portal_id, 'translation', [polygon['id']])
    return {'nodes': [nodes[k] for k in sorted(nodes)],
            'edges': [edges[k] for k in sorted(edges)],
            'directed': False, 'yaw_bins': yaw_bins,
            'translation_model': 'convex_centroid_portal_spokes'}
