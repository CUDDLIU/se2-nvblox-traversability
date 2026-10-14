"""Soft clearance from the boundary of the height-preserving feasible field.

Distances are to the *robot-center* region, not measured rail/body distances.
Adjacent treads are connected by max_step; stacked floors stay separate.
This changes route preference only, never replaces a motion certificate.
"""
from collections import defaultdict
import numpy as np
from scipy.spatial import cKDTree


class ClearancePreference:
    def __init__(self, polygons, config, preferred=.45, weight=6., field_path=None, region_weight=4.):
        self.preferred = float(preferred)
        self.weight = float(weight)
        if not np.isfinite([self.preferred, self.weight]).all() or self.preferred <= 0 or self.weight < 0:
            raise ValueError('Clearance preference requires positive distance and nonnegative weight')
        by_xy = defaultdict(set)
        for polygon in polygons:
            for x, y, z in polygon['cells']:
                by_xy[x, y].add(z)
        samples = []
        r, rz = config.resolution, config.vertical_resolution
        # Sample exposed cell sides every <=5 cm. Internal polygon/yaw/tile
        # seams and traversable stair risers must not become obstacles.
        t = np.linspace(0., 1., max(2, int(np.ceil(r/.05))+1))
        for (x, y), levels in by_xy.items():
            for z in levels:
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    if any(abs(other-z)*rz <= config.max_step+1e-8
                           for other in by_xy.get((x+dx, y+dy), ())):
                        continue
                    if dx:
                        side = np.column_stack((np.full(len(t), (x+(dx>0))*r), (y+t)*r, np.full(len(t), z*rz)))
                    else:
                        side = np.column_stack(((x+t)*r, np.full(len(t), (y+(dy>0))*r), np.full(len(t), z*rz)))
                    samples.extend(side)
        self.samples = np.unique(np.asarray(samples).reshape(-1, 3), axis=0)
        self.tree = cKDTree(self.samples) if len(self.samples) else None
        self.region_tree = None
        self.region_weight = float(region_weight) if weight else 0.
        if field_path is not None and field_path.exists():
            with np.load(field_path, allow_pickle=False) as field:
                cells = field['cells']
                self.region_points = cells.astype(float)*[r, r, rz]+[r/2, r/2, 0.]
                full = (1 << config.yaw_bins)-1
                fraction = np.minimum(1., np.nansum(np.maximum(0., field['states'][:,:,5]-field['states'][:,:,4]), axis=1)/(2*np.pi))
                # Exactly the green/orange classification shown in RViz.
                self.region_values = np.where(field['comfortable'] == full, 0., 1.-.5*fraction)
            if len(cells):self.region_tree = cKDTree(self.region_points)

    def distances(self, points):
        points = np.asarray(points)
        return self.tree.query(points)[0] if self.tree is not None else np.full(points.shape[:-1], np.inf)

    def penalty(self, points, gradient=False):
        points = np.asarray(points)
        if self.tree is None or self.weight == 0:
            return (np.zeros(len(points)), np.zeros_like(points)) if gradient else np.zeros(len(points))
        distance, index = self.tree.query(points)
        deficit = np.maximum(0., 1.-distance/self.preferred)
        value = deficit**2
        if not gradient:
            return value
        direction = (points-self.samples[index])/np.maximum(distance[:, None], 1e-12)
        return value, -2.*deficit[:, None]/self.preferred*direction

    def region_penalty(self, points, gradient=False):
        """Smooth preference for green cells, retaining narrow colored passages.

        Interpolation is local in 3D; overlapping floors are never flattened.
        It does not certify a heading or add a collision/footprint constraint.
        """
        points = np.asarray(points)
        if self.region_tree is None or self.region_weight == 0:
            return (np.zeros(len(points)), np.zeros_like(points)) if gradient else np.zeros(len(points))
        distance, index = self.region_tree.query(points, k=min(4, len(self.region_points)))
        if distance.ndim == 1:distance, index = distance[:, None], index[:, None]
        sigma2 = .12**2
        w = np.exp(-(distance**2-distance[:, :1]**2)/(2*sigma2))
        w /= w.sum(axis=1)[:, None]
        risk = self.region_values[index]
        value = np.sum(w*risk, axis=1)
        if not gradient:return value
        direction = -(points[:, None, :]-self.region_points[index])/sigma2
        return value, np.sum((w*(risk-value[:, None]))[:,:,None]*direction, axis=1)

    def edge_factors(self, a, b):
        """Length integral at <=10 cm spacing, including in-place rotations."""
        a, b = np.asarray(a), np.asarray(b)
        result = np.empty(len(a))
        count = np.maximum(1, np.ceil(np.linalg.norm(b-a, axis=1)/.1).astype(int))
        for n in np.unique(count):
            ids = np.flatnonzero(count == n)
            t = (np.arange(n)+.5)/n
            q = a[ids, None, :]*(1.-t[None, :, None])+b[ids, None, :]*t[None, :, None]
            samples = q.reshape(-1, 3)
            penalty = self.weight*self.penalty(samples)+self.region_weight*self.region_penalty(samples)
            result[ids] = 1.+penalty.reshape(len(ids), n).mean(axis=1)
        return result

    def objective(self, points):
        """Weighted string length and analytic gradient in XYZ."""
        points = np.asarray(points)
        a, b = points[:-1], points[1:]
        diff = b-a
        length = np.linalg.norm(diff, axis=1)
        count = np.maximum(1, np.ceil(length/.1).astype(int))
        edge = np.repeat(np.arange(len(length)), count)
        t = np.concatenate([(np.arange(n)+.5)/n for n in count])
        q = a[edge]*(1.-t[:, None])+b[edge]*t[:, None]
        value, gradient = self.penalty(q, gradient=True)
        region_value, region_gradient = self.region_penalty(q, gradient=True)
        value = self.weight*value+self.region_weight*region_value
        gradient = self.weight*gradient+self.region_weight*region_gradient
        average = np.bincount(edge, weights=value/count[edge], minlength=len(length))
        factor = 1.+average
        direct = diff/np.maximum(length[:, None], 1e-12)*factor[:, None]
        grad = np.zeros_like(points)
        grad[:-1] -= direct
        grad[1:] += direct
        scaled = gradient*(length[edge]/count[edge])[:, None]
        np.add.at(grad, edge, scaled*(1.-t[:, None]))
        np.add.at(grad, edge+1, scaled*t[:, None])
        return float(np.dot(length, factor)), grad

    def describe(self, poses):
        p = np.asarray(poses)[:, :3]
        distances = []
        for a, b in zip(p, p[1:]):
            n = max(1, int(np.ceil(np.linalg.norm(b-a)/.05)))
            distances.extend(self.distances(a+(b-a)*np.linspace(0, 1, n+1)[:, None]))
        return dict(method='height_connected_feasible_field_boundary', preferred_extra_m=self.preferred,
                    weight=self.weight, region_weight=self.region_weight,
                    mean_color_penalty=float(np.mean(self.region_penalty(p))),
                    minimum_m=float(min(distances)) if distances else None,
                    p10_m=float(np.percentile(distances, 10)) if distances else None,
                    meaning='distance within feasible robot-center region; not physical leg/rail clearance')
