"""ASA terrain annotations and atomic route envelope (project integration).

This is not an AOS terrain classifier. It reads tread geometry from the same
immutable mesh as ASA; it does not add body/footprint acceptance checks.
Positions/yaw remain unchanged. Runtime consumers only apply these annotations.
"""
import math
import numpy as np


def arc_lengths(poses):
    p = np.asarray(poses, dtype=float)
    return np.r_[0., np.cumsum(np.linalg.norm(np.diff(p[:, :2], axis=0), axis=1))]


def mode(start, end, terrain='flat'):
    return dict(start_m=float(start), end_m=float(end), terrain=terrain,
                Gait=0x3002 if terrain == 'flat' else 0x3003,
                NavMode=1, ObsMode=0, Speed=0, Manner=0)


def envelope(poses, modes, stamp, frame):
    return dict(schema=1, frame_id=frame, stamp=list(stamp), poses=poses, modes=modes)


def validate_route(doc):
    if doc.get('schema') != 1 or doc.get('frame_id') != 'se2_scan_map':
        raise ValueError('unsupported navigation route schema/frame')
    stamp = doc.get('stamp')
    if (not isinstance(stamp, list) or len(stamp) != 2 or
            any(type(v) is not int for v in stamp) or stamp[0] < 0 or not 0 <= stamp[1] < 10**9):
        raise ValueError('invalid route stamp')
    rows, modes = doc.get('poses'), doc.get('modes')
    if not isinstance(rows, list) or not isinstance(modes, list):
        raise ValueError('invalid route arrays')
    if not rows:
        if modes:raise ValueError('cancel route has modes')
        return doc
    p = np.asarray(rows, dtype=float)
    if p.ndim != 2 or p.shape[1] != 4 or not np.isfinite(p).all():
        raise ValueError('invalid route poses')
    length = arc_lengths(p)[-1]
    end = 0.
    for item in modes:
        for k in ('start_m', 'end_m'):
            if type(item.get(k)) not in (int, float) or not math.isfinite(item[k]):
                raise ValueError('invalid mode boundary')
        if abs(item['start_m']-end) > 1e-6 or item['end_m'] < end:
            raise ValueError('route modes have gaps/overlap')
        terrain = item.get('terrain')
        if terrain not in ('flat', 'stair_up', 'stair_down'):
            raise ValueError('invalid route terrain')
        expected = mode(item['start_m'], item['end_m'], terrain)
        if any(type(item.get(k)) is not int or item[k] != expected[k]
               for k in ('Gait', 'NavMode', 'ObsMode', 'Speed', 'Manner')):
            raise ValueError('ASA route requires autonomous obstacle avoidance')
        end = item['end_m']
    if not modes or abs(end-length) > 1e-6:
        raise ValueError('route modes do not cover path')
    return doc


class TerrainAnnotations:
    """Annotate mesh-supported climbs, including TSDF-smoothed stair flights.

    Horizontal mesh faces (within 12 degrees) supply actual sampled elevations.
    Two or more same-direction rises from 6 cm to max_step (+2.5 cm tolerance), with 12–80 cm treads,
    constitute a flight. These are explicit project classification settings,
    not recovered NOS constants. Smoothed flights also use sustained mesh rise
    (>=30 cm, >=60 cm run, >=0.30 grade); a steep ramp deliberately requests
    the same climbing mode. Gentle ramps, single low bumps and route-Z noise
    remain flat. No runtime body/footprint checks are added.
    """
    def __init__(self, blocks, max_step=.2):
        from shapely.geometry import Polygon
        from shapely.strtree import STRtree
        triangles = np.concatenate([np.asarray(v)[np.asarray(t, dtype=int).reshape(-1, 3)]
                                    for v, t in blocks.values()])
        normal = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
        norm = np.linalg.norm(normal, axis=1)
        # TSDF blends risers/treads into inclined faces. Keep these for the
        # longitudinal profile, while retaining the strict tread detector.
        keep = (norm > 1e-10) & (np.abs(normal[:, 2]) >= math.cos(math.radians(65))*norm)
        self.triangles = triangles[keep]
        self.normal = normal[keep]
        self.horizontal = np.abs(normal[keep, 2]) >= math.cos(math.radians(12))*norm[keep]
        self.tree = STRtree([Polygon(t[:, :2]) for t in self.triangles])
        self.max_step = max_step

    def profile_flights(self, s, heights):
        """Detect sustained supported slopes without bridging holes or floors."""
        flights = []
        ds = s[1]-s[0]
        # Fill at most 12 cm of missing mesh, never route elevations. Larger
        # holes split the evidence and cannot establish a flight.
        h = heights.copy()
        valid = np.flatnonzero(np.isfinite(h))
        for a, b in zip(valid, valid[1:]):
            if b-a > 1 and s[b]-s[a] <= .12+1e-9:
                h[a:b+1] = np.interp(s[a:b+1], s[[a,b]], h[[a,b]])
        radius = max(1, round(.24/ds))
        slopes = np.full(len(s), np.nan)
        for i in range(radius, len(s)-radius):
            window = h[i-radius:i+radius+1]
            if np.isfinite(window).all():
                slopes[i] = (window[-1]-window[0])/(2*radius*ds)
        for direction in (1, -1):
            selected = direction*slopes >= .30
            # A short tread can interrupt the slope estimate; join only short,
            # fully observed gaps, never a descent or a missing mesh region.
            ids = np.flatnonzero(selected)
            for a, b in zip(ids, ids[1:]):
                if 1 < b-a and s[b]-s[a] <= .16+1e-9 and np.isfinite(slopes[a:b+1]).all() and np.min(direction*slopes[a:b+1]) >= 0:
                    selected[a:b+1] = True
            changes = np.diff(np.r_[False, selected, False].astype(int))
            for a, stop in zip(np.flatnonzero(changes==1), np.flatnonzero(changes==-1)):
                b = stop-1
                if s[b]-s[a] < .6 or direction*(h[b]-h[a]) < .30:continue
                # Reject isolated tall discontinuities and oscillating surfaces.
                segment = h[a:b+1]
                delta = direction*np.diff(segment)
                if np.max(np.abs(delta)) > self.max_step+.025:continue
                if -np.minimum(delta, 0).sum() > .15*max(direction*(h[b]-h[a]), .30):continue
                if np.isfinite(heights[a:b+1]).mean() < .8:continue
                flights.append([max(0., s[a]-.8), min(s[-1], s[b]+.5), direction])
        return flights

    def annotate(self, poses):
        from shapely.geometry import Point
        poses = np.asarray(poses, dtype=float)
        arc = arc_lengths(poses);length = float(arc[-1])
        self.diagnostics = dict(method='mesh_treads_and_sustained_grade', mesh_coverage=0., tread_flights=0, profile_flights=0)
        if length < .2:return [mode(0, length)]
        # Keep only distinct arc positions for interpolation (in-place turns).
        keep = np.r_[np.diff(arc) > 1e-9, True]
        s = np.linspace(0., length, max(2, math.ceil(length/.04)+1))
        xyz = np.column_stack([np.interp(s, arc[keep], poses[keep, k]) for k in range(3)])
        heights = np.full(len(s), np.nan)
        tread_heights = heights.copy()
        for i, p in enumerate(xyz):
            ids = self.tree.query(Point(p[:2]), predicate='intersects')
            if not len(ids):continue
            t, n = self.triangles[ids, 0], self.normal[ids]
            z = t[:, 2] - np.sum(n[:, :2]*(p[:2]-t[:, :2]), axis=1)/n[:, 2]
            j = int(np.argmin(np.abs(z-p[2])))
            if abs(z[j]-p[2]) <= self.max_step+.08:
                heights[i] = z[j]
                if self.horizontal[ids[j]]:tread_heights[i] = z[j]
        profile_flights = self.profile_flights(s, heights)
        self.diagnostics.update(mesh_coverage=float(np.isfinite(heights).mean()),
                                profile_flights=len(profile_flights))
        heights = tread_heights
        # Runs of nearly level support, retaining gaps instead of filling them.
        treads = []
        start = 0;low=high=heights[0]
        for i in range(1, len(s)+1):
            if i < len(s) and np.isfinite(low) and np.isfinite(heights[i]):
                lo,hi=min(low,heights[i]),max(high,heights[i])
                if hi-lo<=.025:
                    low,high=lo,hi;continue
            if np.isfinite(low) and i-start >= 2:
                treads.append((s[start], s[i-1], float(np.median(heights[start:i]))))
            start = i
            if i<len(s):low=high=heights[i]
        edges = []
        for a, b in zip(treads, treads[1:]):
            rise = b[2]-a[2]
            if b[0]-a[1] <= .09 and .06 <= abs(rise) <= self.max_step+.025:
                edges.append(((a[1]+b[0])/2, 1 if rise > 0 else -1))
        flights = []
        first = 0
        for i in range(1, len(edges)+1):
            if i < len(edges) and edges[i][1] == edges[i-1][1] and .12 <= edges[i][0]-edges[i-1][0] <= .8:
                continue
            if i-first >= 2:
                flights.append([max(0., edges[first][0]-.6), min(length, edges[i-1][0]+.5), edges[first][1]])
            first = i
        self.diagnostics['tread_flights'] = len(flights)
        flights = sorted(flights+profile_flights)
        # Overlapping preparation/exit intervals stay in stair gait. Direction
        # labels may change at a landing without a flat->stair toggle.
        boundaries = sorted({0., length, *(float(v) for f in flights for v in f[:2])})
        result = []
        for a, b in zip(boundaries, boundaries[1:]):
            active = [f for f in flights if f[0] <= (a+b)/2 <= f[1]]
            terrain = ('stair_up' if active[-1][2] > 0 else 'stair_down') if active else 'flat'
            if result and result[-1]['terrain'] == terrain:result[-1]['end_m'] = b
            else:result.append(mode(a, b, terrain))
        return result or [mode(0, length)]
