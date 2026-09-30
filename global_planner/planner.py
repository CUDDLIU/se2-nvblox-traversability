"""A*--String Pulling--A* on a height-preserving, yaw-layered polygon mesh.

Independent implementation of SE(2) NavMesh, Section V-C. Polygon-edge
midpoints form the initial graph. Physical-pose witnesses supplement the
discrete layers in the existing classifier's sub-cell narrow passages.
"""
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
import time
import numpy as np
from scipy.optimize import minimize
from shapely.geometry import Point, Polygon, box
from shapely.strtree import STRtree

from geometry import MotionChecker, read_mesh, wrap
from paper_pipeline import Config


class NoPath(ValueError):
    pass


class Planner:
    def __init__(self, directory, longitudinal=.4, lateral=.2, yaw_rate=.6):
        self.directory = Path(directory)
        self.document = json.loads((self.directory / 'navmesh.json').read_text())
        self.config = Config(**self.document['config'])
        self.bins = self.config.yaw_bins
        self.polygons = self.document['polygons']
        self.shapes = [Polygon(np.asarray(p['vertices'])[:, :2]) for p in self.polygons]
        self.tree = STRtree(self.shapes)
        self.portals = self.document['portals']
        self.speeds = (float(longitudinal), float(lateral), float(yaw_rate))
        if min(self.speeds) <= 0:
            raise ValueError('Planning velocities must be positive')
        mesh_path = self.directory / self.document['source_mesh']
        if hashlib.sha256(mesh_path.read_bytes()).hexdigest() != self.document['mesh_sha256']:
            raise ValueError('Mesh 已改变，请重新构建通行图后规划。')
        blocks, self.trajectory = read_mesh(mesh_path)
        self.checker = MotionChecker(blocks, self.config)
        self.positions, self.angles, self.owners = [], [], []
        self.region_positions = defaultdict(list)
        self.extra = defaultdict(set)
        self.witness_cells = {}
        self.witness_proof = {}
        self.edge_cache = {}
        self.portal_lookup = defaultdict(list)
        for portal in self.portals:
            a, b = portal['polygons']
            pid = self.add_position(np.mean(portal['segment'], axis=0), [a, b], portal['yaw_mask'])
            self.portal_lookup[tuple(sorted((a, b)))].append(portal)
        # Isolated polygons still support a query entirely within themselves.
        for i, shape in enumerate(self.shapes):
            center = shape.centroid
            self.add_position([center.x, center.y, self.polygons[i]['floor']], [i], self.polygons[i]['yaw_mask'])
        self.add_refined_witnesses()
        self.base_positions = len(self.positions)

    def add_position(self, xyz, owners, mask, angles=None):
        pid = len(self.positions)
        self.positions.append(np.asarray(xyz, dtype=float))
        self.angles.append(angles if angles is not None else
                           {j: j * 2 * math.pi / self.bins for j in range(self.bins) if int(mask) & (1 << j)})
        self.owners.append(tuple(owners))
        for region in owners:
            self.region_positions[region].append(pid)
        return pid

    def add_refined_witnesses(self):
        path = self.directory / 'field.npz'
        if not path.exists():
            return
        cell_regions = defaultdict(list)
        for i, polygon in enumerate(self.polygons):
            for cell in polygon['cells']:
                cell_regions[tuple(cell)].append(i)
        by_cell = defaultdict(list)
        with np.load(path, allow_pickle=False) as field:
            for cell, states, proof in zip(field['cells'], field['states'], field['proved']):
                key = tuple(map(int, cell))
                if int(proof) == (1 << self.bins)-1:
                    continue
                grouped = defaultdict(dict)
                for yaw, state in enumerate(states):
                    if not np.isfinite(state).all():
                        continue
                    grouped[tuple(np.round(state[:3], 8))][yaw] = float(state[3])
                for xyz, angles in grouped.items():
                    owners = [r for r in cell_regions[key] if self.shapes[r].buffer(1e-8).covers(Point(xyz[:2]))]
                    if not owners:
                        continue
                    pid = self.add_position(xyz, owners, 0, angles)
                    self.witness_cells[pid] = key
                    center = [(key[0]+.5)*self.config.resolution, (key[1]+.5)*self.config.resolution,
                              key[2]*self.config.vertical_resolution]
                    nominal = sum(1 << yaw for yaw, angle in angles.items()
                                  if abs(wrap(angle-yaw*2*math.pi/self.bins)) < 1e-7)
                    self.witness_proof[pid] = (int(proof) & nominal) if np.linalg.norm(np.asarray(xyz)-center)<1e-7 else 0
                    by_cell[key].append(pid)
        by_xy = defaultdict(list)
        for key in by_cell:
            by_xy[key[:2]].append(key)
        for key, pids in by_cell.items():
            for dx, dy in ((1, 0), (0, 1), (1, 1), (1, -1)):
                for other in by_xy.get((key[0]+dx, key[1]+dy), []):
                    if abs(other[2]-key[2])*self.config.vertical_resolution > self.config.max_step+1e-8:
                        continue
                    for a in pids:
                        for b in by_cell[other]:
                            self.extra[a].add(b)
                            self.extra[b].add(a)

    def pose(self, node):
        pid, yaw = node
        return np.r_[self.positions[pid], self.angles[pid][yaw]]

    def candidates(self, xyz, tolerance=.25):
        p = np.asarray(xyz, dtype=float)
        found = []
        for raw_i in self.tree.query(box(p[0]-tolerance, p[1]-tolerance, p[0]+tolerance, p[1]+tolerance)):
            i = int(raw_i)
            floor = self.polygons[i]['floor']
            if abs(floor-p[2]) > tolerance:
                continue
            shape = self.shapes[i]
            if shape.distance(Point(p[:2])) > tolerance:
                continue
            found.append(i)
        return found

    def project(self, xyz, tolerance=.25):
        from shapely.ops import nearest_points
        p = np.asarray(xyz, dtype=float)
        if p.shape != (3,) or not np.isfinite(p).all():
            raise NoPath('选点必须为有限的三维坐标。')
        choices = []
        for i in self.candidates(p, tolerance):
            q = nearest_points(self.shapes[i], Point(p[:2]))[0]
            point = np.array([q.x, q.y, self.polygons[i]['floor']])
            choices.append((float(np.linalg.norm(point-p)), i, point))
        if not choices:
            raise NoPath('选点附近没有该高度的可通行表面；请点击地面或楼梯踏板，勿点击墙面。')
        choices.sort(key=lambda x: x[0])
        _, region, point = choices[0]
        return point, region

    def default_start(self):
        if not len(self.trajectory):
            raise NoPath('地图没有起始位姿，请先设置起点。')
        row = self.trajectory[-1]
        body = row[1:4]
        floors = []
        for raw_i in self.tree.query(box(body[0]-.3, body[1]-.3, body[0]+.3, body[1]+.3)):
            i = int(raw_i)
            dz = body[2]-self.polygons[i]['floor']
            if -.05 <= dz <= self.config.height + .25:
                floors.append((self.shapes[i].distance(Point(body[:2])), abs(dz-.56), i))
        if not floors:
            raise NoPath('录制终点下方没有可通行表面，请手动设置起点。')
        i = min(floors)[2]
        point, _ = self.project([body[0], body[1], self.polygons[i]['floor']], .35)
        x, y, z, w = row[4:8]
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        return np.r_[point, yaw]

    def cost(self, a, b):
        delta = b[:2]-a[:2]
        c, s = math.cos(a[3]), math.sin(a[3])
        vl, vt, omega = self.speeds
        return abs(c*delta[0]+s*delta[1])/vl + abs(-s*delta[0]+c*delta[1])/vt + abs(wrap(b[3]-a[3]))/omega

    def voxel_proof(self, a, b):
        """The paper's continuous-yaw voxel-mask guarantee within a polygon.

        A refined feasible bit alone never qualifies. Its complete swept channel
        must be independently certified by the footprint classifier in EVERY
        source cell. This preserves the representation's finite voxel semantics.
        """
        pa, pb = self.pose(a), self.pose(b)
        common = set(self.owners[a[0]]) & set(self.owners[b[0]])
        if abs(pa[2]-pb[2]) > self.config.max_step+1e-8:
            return False
        delta = wrap(pb[3]-pa[3])
        width = 2*math.pi/self.bins
        count = max(1, math.ceil(abs(delta)/width)*2+1)
        needed = {int(math.floor((pa[3]+delta*i/count)/width+.5)) % self.bins for i in range(count+1)}
        if any(all(int(self.polygons[r].get('proven_yaw_mask', 0)) & (1 << y) for y in needed)
               for r in common):
            return True
        # Cardinal neighbouring voxel centers with complete channel proofs are
        # the same translational connectivity used by the original mask graph.
        ca, cb = self.witness_cells.get(a[0]), self.witness_cells.get(b[0])
        if ca is not None and cb is not None and (ca == cb or abs(ca[0]-cb[0])+abs(ca[1]-cb[1]) == 1):
            proof = self.witness_proof[a[0]] & self.witness_proof[b[0]]
            return all(proof & (1 << y) for y in needed)
        return False

    def astar(self, start, goals, chain=None, timeout=60.):
        from search import Search
        if not hasattr(self, 'search'):
            self.search = Search(self)
        return self.search.run(start, goals, chain, timeout)

    def region_sequence(self, path):
        sequence = []
        for a, b in zip(path, path[1:]):
            common = set(self.owners[a[0]]) & set(self.owners[b[0]])
            if not common:
                pairs = [(ra, rb) for ra in self.owners[a[0]] for rb in self.owners[b[0]]
                         if tuple(sorted((ra, rb))) in self.portal_lookup]
                if not pairs:
                    return None  # A sub-cell diagonal has no polygon portal.
                ra, rb = min(pairs, key=lambda pair: (bool(sequence and pair[0]!=sequence[-1]), pair))
                for region in (ra, rb):
                    if not sequence or sequence[-1]!=region:sequence.append(region)
                continue
            region = sequence[-1] if sequence and sequence[-1] in common else min(common)
            if not sequence or sequence[-1] != region:
                sequence.append(region)
        return sequence

    def straighten(self, path):
        sequence = self.region_sequence(path)
        if not sequence:
            return None
        start, end = self.pose(path[0])[:3], self.pose(path[-1])[:3]
        segments = []
        for a, b in zip(sequence, sequence[1:]):
            choices = self.portal_lookup.get(tuple(sorted((a, b))), [])
            if not choices:
                return None
            portal = min(choices, key=lambda p: min(np.linalg.norm(np.mean(p['segment'], axis=0)-self.pose(n)[:3]) for n in path))
            segments.append(np.asarray(portal['segment']))
        if not segments:
            return np.asarray([start, end]), sequence
        edges = np.asarray(segments)
        base, delta = edges[:, 0], edges[:, 1]-edges[:, 0]

        def points(t):
            return np.vstack([start, base+t[:, None]*delta, end])

        def objective(t):
            diff = np.diff(points(t), axis=0)
            lengths = np.linalg.norm(diff, axis=1)
            unit = diff/np.maximum(lengths[:, None], 1e-10)
            grad = np.einsum('ij,ij->i', unit[:-1]-unit[1:], delta)
            return float(lengths.sum()), grad

        result = minimize(objective, np.full(len(edges), .5), jac=True, method='L-BFGS-B',
                          bounds=[(0., 1.)]*len(edges), options={'maxiter': 150, 'ftol': 1e-10})
        return points(result.x), sequence

    def straighten_sections(self, path):
        """String-pull polygon corridors, retaining exact sub-cell anchors.

        Partial-cell witnesses do not certify a whole polygon. Their metric
        transitions bound each string-pulling section instead of being silently
        promoted to full polygon/yaw feasibility.
        """
        output=[];sections=0

        def original(node):
            return dict(xyz=self.positions[node[0]].copy(), owners=list(self.owners[node[0]]), source=node[0])

        def append_section(nodes):
            nonlocal sections
            if not nodes:return
            pulled=self.straighten(nodes) if len(nodes)>1 else None
            if pulled is None:
                output.extend(original(n) for n in nodes)
                return
            points,regions=pulled
            sections+=1
            for i,xyz in enumerate(points):
                owners=([regions[0]] if i==0 else [regions[-1]] if i==len(points)-1 else [regions[i-1],regions[i]])
                source=nodes[0][0] if i==0 else nodes[-1][0] if i==len(points)-1 else None
                output.append(dict(xyz=xyz,owners=owners,source=source))

        first=0
        for i,(a,b) in enumerate(zip(path,path[1:])):
            common=set(self.owners[a[0]]) & set(self.owners[b[0]])
            portal=any(tuple(sorted((ra,rb))) in self.portal_lookup for ra in self.owners[a[0]] for rb in self.owners[b[0]])
            if (not common and not portal) or not self.voxel_proof(a,b):
                append_section(path[first:i+1])
                output.extend([original(a),original(b)])
                first=i+1
        append_section(path[first:])
        compact=[]
        for item in output:
            if compact and np.linalg.norm(item['xyz']-compact[-1]['xyz'])<1e-9:
                compact[-1]['owners']=sorted(set(compact[-1]['owners'])|set(item['owners']))
                if item['source'] is not None:compact[-1]['source']=item['source']
            else:compact.append(item)
        return compact,sections

    def endpoint_angles(self, xyz, region):
        """Keep actual non-bin headings when querying a refined narrow cell."""
        mask=int(self.polygons[region]['yaw_mask'])
        angles={y:y*2*math.pi/self.bins for y in range(self.bins) if mask & (1<<y)}
        choices={}
        for pid in self.region_positions[region]:
            if pid not in self.witness_cells:continue
            distance=float(np.linalg.norm(self.positions[pid]-xyz))
            for yaw,angle in self.angles[pid].items():
                if yaw not in angles or abs(wrap(angle-angles[yaw]))<1e-8:continue
                if yaw not in choices or distance<choices[yaw][0]:choices[yaw]=(distance,angle)
        candidates=list(choices)
        poses=[np.r_[xyz,choices[y][1]] for y in candidates]
        for yaw,ok in zip(candidates,self.checker.motions([(p,p) for p in poses])):
            if ok:angles[yaw]=choices[yaw][1]
        return angles

    def query(self, start, goal, goal_yaw=None, timeout=60.):
        started = time.monotonic()
        if len(start) != 4 or not np.isfinite(start).all() or (goal_yaw is not None and not math.isfinite(goal_yaw)):
            raise NoPath('起点位置与朝向必须为有限值。')
        base = len(self.positions)
        try:
            sp, sr = self.project(start[:3])
            gp, gr = self.project(goal[:3])
            sid = self.add_position(sp, [sr], 0, self.endpoint_angles(sp,sr))
            gid = self.add_position(gp, [gr], 0, self.endpoint_angles(gp,gr))
            self.angles[sid][-1] = float(start[3])
            if goal_yaw is not None:
                self.angles[gid][-1] = float(goal_yaw)
            goals = {(gid, y) for y in self.angles[gid]} if goal_yaw is None else {(gid, -1)}
            initial, initial_metrics = self.astar((sid, -1), goals, timeout=timeout)
            final = initial
            report = dict(initial=initial_metrics, straightening='unchanged', yaw_refinement='unchanged')
            pulled,sections = self.straighten_sections(initial)
            if len(pulled)>1:
                chain, pids = defaultdict(set), []
                for item in pulled:
                    xyz,owners,source=item['xyz'],item['owners'],item['source']
                    mask = (1 << self.bins)-1
                    for owner in owners:
                        mask &= self.polygons[owner]['yaw_mask']
                    angles=dict(self.angles[source]) if source is not None else None
                    pid=self.add_position(xyz,owners,mask,angles)
                    pids.append(pid)
                    if source in self.witness_cells:
                        self.witness_cells[pid]=self.witness_cells[source]
                        self.witness_proof[pid]=self.witness_proof[source]
                self.angles[pids[0]][-1] = float(start[3])
                if goal_yaw is not None:
                    self.angles[pids[-1]][-1] = float(goal_yaw)
                for a, b in zip(pids, pids[1:]):
                    chain[a].add(b)
                    chain[b].add(a)
                goal_states = {(pids[-1], y) for y in self.angles[pids[-1]]} if goal_yaw is None else {(pids[-1], -1)}
                try:
                    candidate, refined = self.astar((pids[0], -1), goal_states, chain=chain, timeout=min(timeout, 30.))
                    report.update(straightening='portal-constrained 3D string pulling with retained metric anchors',
                                  string_pulling_sections=sections, yaw_refinement=refined)
                    if refined['cost'] <= initial_metrics['cost']+1e-8:
                        final = candidate
                    else:
                        report['selection'] = 'initial checked path has lower cost'
                except NoPath as exc:
                    report['straightening'] = 'refined route rejected by physical motion checks'
                    report['refinement_detail'] = str(exc)
            poses = np.asarray([self.pose(node) for node in final])
            checks = np.asarray([self.voxel_proof(a, b) or self.checker.motion(self.pose(a), self.pose(b))
                                 for a, b in zip(final, final[1:])])
            if not checks.all():
                raise NoPath('最终路径复核失败，拒绝发布。')
            report.update(success=True, method='ASA', seconds=time.monotonic()-started,
                          start=poses[0].tolist(), goal=poses[-1].tolist(),
                          length_m=float(np.linalg.norm(np.diff(poses[:, :3], axis=0), axis=1).sum()),
                          height_range_m=[float(poses[:, 2].min()), float(poses[:, 2].max())],
                          motion_segments_verified=len(checks), poses=poses.tolist())
            report['voxel_certified_segments'] = sum(self.voxel_proof(a, b) for a, b in zip(final, final[1:]))
            report['metric_certified_segments'] = len(checks)-report['voxel_certified_segments']
            return report
        finally:
            for region in self.region_positions:
                self.region_positions[region] = [p for p in self.region_positions[region] if p < base]
            self.positions[base:] = []
            self.angles[base:] = []
            self.owners[base:] = []
            self.edge_cache = {k: v for k, v in self.edge_cache.items() if max(k[0][0], k[1][0]) < base}
            if hasattr(self, 'search'):
                self.search.pose_cache={k:v for k,v in self.search.pose_cache.items() if k[0]<base}
            self.witness_cells={k:v for k,v in self.witness_cells.items() if k<base}
            self.witness_proof={k:v for k,v in self.witness_proof.items() if k<base}
