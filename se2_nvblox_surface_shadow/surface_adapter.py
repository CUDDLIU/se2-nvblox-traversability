import math
import time
import os
import resource
import numpy as np
from collections import defaultdict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2, Image
from visualization_msgs.msg import Marker, MarkerArray
from nvblox_msgs.msg import Mesh, DistanceMapSlice


class SurfaceAdapter(Node):
    """Shadow-only Mesh/ESDF to local SE(2) diagnostics.

    This node never publishes cmd_vel, map->odom, or navigation unlock signals.
    Unknown cells are conservatively marked occupied.
    """
    def __init__(self):
        super().__init__('se2_nvblox_surface_adapter')
        self.declare_parameter('cloud_topic', '/LIDAR/POINTS_NX')
        self.declare_parameter('depth_topic', '')
        self.declare_parameter('mesh_topic', '/nvblox_node/mesh')
        self.declare_parameter('esdf_topic', '/nvblox_node/static_map_slice')
        self.declare_parameter('odom_topic', '/lio/odom')
        self.declare_parameter('output_frame', 'map')
        self.declare_parameter('window_radius_m', 10.0)
        self.declare_parameter('surface_resolution_m', 0.15)
        self.declare_parameter('yaw_bins', 24)
        self.declare_parameter('max_slope_deg', 30.0)
        self.declare_parameter('support_ratio_min', 0.70)
        self.declare_parameter('max_footprint_height_range_m', 0.15)
        self.declare_parameter('footprint_length_m', 0.82)
        self.declare_parameter('footprint_width_m', 0.506)
        self.declare_parameter('extra_clearance_m', 0.10)
        self.declare_parameter('mesh_vertices_are_local', False)
        self.declare_parameter('log_period_s', 1.0)
        self.declare_parameter('yaw_eval_period_s', 1.0)
        self.declare_parameter('visualization_period_s', 1.0)
        self.declare_parameter('max_yaw_markers', 200)
        self.frame = self.get_parameter('output_frame').value
        self.res = float(self.get_parameter('surface_resolution_m').value)
        self.radius = float(self.get_parameter('window_radius_m').value)
        self.yaw_bins = int(self.get_parameter('yaw_bins').value)
        self.max_slope = float(self.get_parameter('max_slope_deg').value)
        self.min_support = float(self.get_parameter('support_ratio_min').value)
        self.max_height_range = float(self.get_parameter('max_footprint_height_range_m').value)
        self.fp_l = float(self.get_parameter('footprint_length_m').value)
        self.fp_w = float(self.get_parameter('footprint_width_m').value)
        self.extra_clearance = float(self.get_parameter('extra_clearance_m').value)
        self.yaw_eval_period = float(self.get_parameter('yaw_eval_period_s').value)
        self.visualization_period = float(self.get_parameter('visualization_period_s').value)
        self.max_yaw_markers = int(self.get_parameter('max_yaw_markers').value)
        self.mesh_blocks = {}
        # Per-block support cache.  Mesh messages usually contain only changed
        # blocks; keeping the raster per block avoids re-walking every cached
        # triangle on every message.
        self.block_support_cells = {}
        self.block_support_counts = {}
        self.block_rejected_counts = {}
        self.surface_cells = {}
        self.esdf = None
        self.odom = None
        self.stats = defaultdict(int)
        self.last_mesh_wall = None
        self.last_esdf_wall = None
        self.last_yaw_eval_wall = 0.0
        self.last_visualization_wall = 0.0
        self.pub_grid = self.create_publisher(OccupancyGrid, '~/surface_grid', 1)
        self.pub_markers = self.create_publisher(MarkerArray, '~/surface_markers', 1)
        self.pub_yaw = self.create_publisher(MarkerArray, '~/yaw_footprint_shadow', 1)
        self.create_subscription(Mesh, self.get_parameter('mesh_topic').value, self.mesh_cb, 5)
        self.create_subscription(DistanceMapSlice, self.get_parameter('esdf_topic').value, self.esdf_cb, 5)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.odom_cb, 10)
        self.cloud_topic = self.get_parameter('cloud_topic').value
        # This is diagnostic liveness only; nvblox itself owns the live input.
        # BEST_EFFORT depth=1 prevents a slow Python callback from retaining a
        # backlog or causing reliable retransmissions on the shadow branch.
        cloud_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                               history=HistoryPolicy.KEEP_LAST)
        depth_topic = self.get_parameter('depth_topic').value
        if depth_topic:
            self.create_subscription(Image, depth_topic, self.depth_cb, cloud_qos)
        else:
            self.create_subscription(PointCloud2, self.cloud_topic, self.cloud_cb, cloud_qos)
        self.timer = self.create_timer(1.0, self.publish_diagnostics)
        self.get_logger().info(
            f'START shadow_only=true cloud={self.cloud_topic} '
            f'mesh={self.get_parameter("mesh_topic").value} '
            f'esdf={self.get_parameter("esdf_topic").value} '
            f'odom={self.get_parameter("odom_topic").value} radius={self.radius:.1f} '
            f'res={self.res:.2f} yaw_bins={self.yaw_bins} '
            f'footprint={self.fp_l:.3f}x{self.fp_w:.3f}')

    def depth_cb(self, msg):
        self.stats['depth_frames'] += 1
        if msg.header.stamp.sec <= 0:
            self.stats['invalid_cloud_stamp'] += 1

    def cloud_cb(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.stats['cloud_frames'] += 1
        self.stats['cloud_points_est'] += int(msg.width * max(1, msg.height))
        if msg.header.frame_id != 'lidar_link':
            self.stats['cloud_frame_mismatch'] += 1
        if stamp <= 0.0:
            self.stats['invalid_cloud_stamp'] += 1

    def odom_cb(self, msg):
        if msg.header.frame_id != self.frame:
            self.stats['odom_frame_mismatch'] += 1
            return
        self.odom = msg
        self.stats['odom_frames'] += 1

    def mesh_cb(self, msg):
        if msg.header.frame_id != self.frame:
            self.stats['mesh_frame_mismatch'] += 1
            return
        t0 = time.perf_counter()
        if msg.clear:
            self.mesh_blocks.clear()
            self.block_support_cells.clear()
            self.block_support_counts.clear()
            self.block_rejected_counts.clear()
            self.stats['mesh_clears'] += 1
        changed = 0
        if self.stats['mesh_msgs'] == 0:
            self.get_logger().info(
                'MESH_SCHEMA clear=%s block_indices=%d blocks=%d header_frame=%s '
                'msg_attrs=%s' % (
                    bool(msg.clear), len(msg.block_indices), len(msg.blocks),
                    msg.header.frame_id, ','.join(sorted(a for a in dir(msg) if not a.startswith('_')))))
            if msg.blocks:
                b0 = msg.blocks[0]
                self.get_logger().info(
                    'MESH_BLOCK_SCHEMA attrs=%s vertices=%d normals=%d triangles=%d colors=%d'
                    % (','.join(sorted(a for a in dir(b0) if not a.startswith('_'))),
                       len(getattr(b0, 'vertices', [])), len(getattr(b0, 'normals', [])),
                       len(getattr(b0, 'triangles', [])), len(getattr(b0, 'colors', []))))
        for idx, block in zip(msg.block_indices, msg.blocks):
            key = (idx.x, idx.y, idx.z)
            self.mesh_blocks[key] = block
            changed += 1
        self.stats['mesh_msgs'] += 1
        self.stats['mesh_blocks_changed'] += changed
        self.stats['mesh_vertices'] = sum(len(b.vertices) for b in self.mesh_blocks.values())
        self.stats['mesh_triangles'] = sum(len(b.triangles) // 3 for b in self.mesh_blocks.values())
        changed_keys = [
            (idx.x, idx.y, idx.z) for idx in msg.block_indices[:len(msg.blocks)]
        ]
        self.extract_support_cells(msg, changed_keys)
        self.last_mesh_wall = time.monotonic()
        now = time.monotonic()
        if now - self.last_visualization_wall >= self.visualization_period:
            self.last_visualization_wall = now
            self.publish_shadow_surface(msg.header)
        self.stats['mesh_update_ms_last'] = round((time.perf_counter() - t0) * 1000.0, 3)

    def extract_support_cells(self, msg, changed_keys):
        """Extract conservative upward-facing support samples from changed mesh blocks.

        This is intentionally a diagnostic raster, not yet the navigation graph.
        Unknown remains blocked until support is observed and validated.
        """
        cos_limit = math.cos(math.radians(self.max_slope))
        support = 0
        rejected = 0
        local_vertices = bool(self.get_parameter('mesh_vertices_are_local').value)
        # Rebuild only blocks carried in this Mesh message.  The global raster
        # is then merged from the small per-block rasters (cells, not triangles).
        for key in changed_keys:
            block = self.mesh_blocks.get(key)
            if block is None:
                continue
            block_cells = {}
            block_support = 0
            block_rejected = 0
            ox, oy, oz = (key[i] * float(msg.block_size_m) for i in range(3))
            verts = []
            for p in block.vertices:
                verts.append((p.x + ox, p.y + oy, p.z + oz) if local_vertices else (p.x, p.y, p.z))
            tri = block.triangles
            for i in range(0, len(tri) - 2, 3):
                try:
                    a, b, c = verts[tri[i]], verts[tri[i + 1]], verts[tri[i + 2]]
                except (IndexError, TypeError):
                    self.stats['mesh_bad_triangle_indices'] += 1
                    continue
                ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
                vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
                nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
                norm = math.sqrt(nx * nx + ny * ny + nz * nz)
                if norm < 1e-6:
                    block_rejected += 1
                    continue
                nz = nz / norm
                cx, cy, cz = tuple((a[j] + b[j] + c[j]) / 3.0 for j in range(3))
                if nz < cos_limit:
                    block_rejected += 1
                    continue
                key2 = (math.floor(cx / self.res), math.floor(cy / self.res))
                old = block_cells.get(key2)
                block_cells[key2] = cz if old is None else min(old, cz)
                block_support += 1
            self.block_support_cells[key] = block_cells
            self.block_support_counts[key] = block_support
            self.block_rejected_counts[key] = block_rejected

        # Merge per-block cells.  This is bounded by the raster size and keeps
        # a single global height for cells straddling block boundaries.
        merged = {}
        for block_cells in self.block_support_cells.values():
            for key2, cz in block_cells.items():
                old = merged.get(key2)
                merged[key2] = cz if old is None else min(old, cz)
        self.surface_cells = merged
        support = sum(self.block_support_counts.values())
        rejected = sum(self.block_rejected_counts.values())
        self.stats['support_triangles'] = support
        self.stats['rejected_triangles'] = rejected
        self.stats['support_cells'] = len(self.surface_cells)
        now = time.monotonic()
        if now - self.last_yaw_eval_wall >= self.yaw_eval_period:
            self.last_yaw_eval_wall = now
            self.evaluate_yaw_layers()

    def evaluate_yaw_layers(self):
        """Evaluate footprint feasibility without sending a navigation command."""
        if not self.surface_cells:
            self.stats['yaw_free_states'] = 0
            self.stats['yaw_test_states'] = 0
            return
        half_l = self.fp_l * 0.5
        half_w = self.fp_w * 0.5
        sample = [(sx * half_l, sy * half_w)
                  for sx in (-1.0, 0.0, 1.0) for sy in (-1.0, 0.0, 1.0)]
        tested = free = 0
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        marker_id = 0
        # Integer cell offsets are translation invariant: vectorize the nine
        # footprint probes across cells rather than repeating trigonometry.
        # Footprint feasibility is a local diagnostic.  Evaluating every cell
        # accumulated during a long walk creates tens of thousands of states
        # and stalls RViz; keep the full support raster but test only the
        # robot's current local window.
        if self.odom is not None:
            ox = self.odom.pose.pose.position.x
            oy = self.odom.pose.pose.position.y
            r2 = self.radius * self.radius
            cells = [k for k in self.surface_cells
                     if ((k[0] + 0.5) * self.res - ox) ** 2 +
                        ((k[1] + 0.5) * self.res - oy) ** 2 <= r2]
        else:
            cells = list(self.surface_cells)
        if not cells:
            self.stats['yaw_free_states'] = 0
            self.stats['yaw_test_states'] = 0
            return
        coords = np.asarray(cells, dtype=np.int64)
        low = coords.min(axis=0)
        shape = coords.max(axis=0) - low + 1
        feasible = None
        if int(shape[0]) * int(shape[1]) <= 4_000_000:
            occupied = np.zeros(tuple(shape), dtype=bool)
            heights = np.full(tuple(shape), np.nan)
            rel = coords - low
            occupied[rel[:, 0], rel[:, 1]] = True
            heights[rel[:, 0], rel[:, 1]] = [self.surface_cells[k] for k in cells]
            feasible = np.ones((len(cells), self.yaw_bins), dtype=bool)
            for yaw_i in range(self.yaw_bins):
                yaw = 2.0 * math.pi * yaw_i / self.yaw_bins
                c, s = math.cos(yaw), math.sin(yaw)
                z_min = np.full(len(cells), np.inf)
                z_max = np.full(len(cells), -np.inf)
                for sx, sy in sample:
                    delta = np.array([math.floor(0.5 + (c*sx-s*sy)/self.res),
                                      math.floor(0.5 + (s*sx+c*sy)/self.res)])
                    query = rel + delta
                    valid = ((query >= 0) & (query < shape)).all(axis=1)
                    hit = np.zeros(len(cells), dtype=bool)
                    hit[valid] = occupied[query[valid, 0], query[valid, 1]]
                    feasible[:, yaw_i] &= hit
                    zs = np.full(len(cells), np.nan)
                    zs[valid] = heights[query[valid, 0], query[valid, 1]]
                    z_min = np.minimum(z_min, zs)
                    z_max = np.maximum(z_max, zs)
                feasible[:, yaw_i] &= (z_max - z_min <= self.max_height_range)
            tested = len(cells) * self.yaw_bins
            free = int(feasible.sum())
        for cell_i, (ix, iy) in enumerate(cells):
            z = self.surface_cells[(ix, iy)]
            cx, cy = (ix + 0.5) * self.res, (iy + 0.5) * self.res
            for yaw_i in range(self.yaw_bins):
                yaw = 2.0 * math.pi * yaw_i / self.yaw_bins
                if feasible is not None and marker_id >= self.max_yaw_markers:
                    break
                c, s = math.cos(yaw), math.sin(yaw)
                ok = bool(feasible[cell_i, yaw_i]) if feasible is not None else True
                if feasible is None:
                    tested += 1
                    sample_heights = []
                    for sx, sy in sample:
                        px = ix + math.floor(0.5 + (c*sx-s*sy)/self.res)
                        py = iy + math.floor(0.5 + (s*sx+c*sy)/self.res)
                        if (px, py) not in self.surface_cells:
                            ok = False
                            break
                        sample_heights.append(self.surface_cells[(px, py)])
                    if ok and max(sample_heights) - min(sample_heights) > self.max_height_range:
                        ok = False
                if ok:
                    if feasible is None:
                        free += 1
                    if marker_id < self.max_yaw_markers:
                        m = Marker()
                        m.header.frame_id = self.frame
                        m.header.stamp = self.get_clock().now().to_msg()
                        m.ns = 'yaw_footprint_shadow'
                        m.id = marker_id
                        m.type = Marker.CUBE
                        m.action = Marker.ADD
                        m.pose.position.x = cx
                        m.pose.position.y = cy
                        m.pose.position.z = z + 0.03
                        m.pose.orientation.z = math.sin(yaw * 0.5)
                        m.pose.orientation.w = math.cos(yaw * 0.5)
                        m.scale.x = self.fp_l
                        m.scale.y = self.fp_w
                        m.scale.z = 0.04
                        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.9, 0.2, 0.18
                        markers.markers.append(m)
                        marker_id += 1
        self.stats['yaw_free_states'] = free
        self.stats['yaw_test_states'] = tested
        self.pub_yaw.publish(markers)

    def esdf_cb(self, msg):
        self.esdf = msg
        self.stats['esdf_msgs'] += 1
        self.stats['esdf_cells'] = int(msg.width * msg.height)
        self.last_esdf_wall = time.monotonic()

    def publish_shadow_surface(self, header):
        # Conservative local visualization grid. Mesh triangles are counted and
        # normals are checked; detailed footprint rasterization follows after bag validation.
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.frame
        n = max(1, int(2 * self.radius / self.res))
        grid.info.resolution = self.res
        grid.info.width = n
        grid.info.height = n
        ox = oy = -self.radius
        if self.odom is not None:
            ox = self.odom.pose.pose.position.x - self.radius
            oy = self.odom.pose.pose.position.y - self.radius
        grid.info.origin.position.x = ox
        grid.info.origin.position.y = oy
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [-1] * (n * n)
        for (ix, iy), _z in self.surface_cells.items():
            wx = (ix + 0.5) * self.res
            wy = (iy + 0.5) * self.res
            gx = int(math.floor((wx - ox) / self.res))
            gy = int(math.floor((wy - oy) / self.res))
            if 0 <= gx < n and 0 <= gy < n:
                grid.data[gy * n + gx] = 100
        self.pub_grid.publish(grid)

        # One POINTS marker keeps RViz responsive when a long walk has built
        # thousands of support cells.  One CUBE marker per cell made RViz
        # spend most of a frame updating marker bookkeeping.
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        m = Marker()
        m.header = grid.header
        m.ns = 'support_surface'
        m.id = 0
        m.type = Marker.POINTS
        m.action = Marker.ADD
        m.scale.x = self.res
        m.scale.y = self.res
        m.color.r, m.color.g, m.color.b, m.color.a = 0.05, 0.75, 0.95, 0.75
        for (ix, iy), z in list(self.surface_cells.items())[:5000]:
            m.points.append(Point(x=(ix + 0.5) * self.res,
                                  y=(iy + 0.5) * self.res, z=z))
        markers.markers.append(m)
        self.pub_markers.publish(markers)

    def publish_diagnostics(self):
        age_mesh = -1.0 if self.last_mesh_wall is None else time.monotonic() - self.last_mesh_wall
        age_esdf = -1.0 if self.last_esdf_wall is None else time.monotonic() - self.last_esdf_wall
        try:
            with open('/proc/meminfo', 'r', encoding='utf-8') as f:
                mem = {line.split(':', 1)[0]: int(line.split()[1]) for line in f if ':' in line}
            mem_avail_mb = mem.get('MemAvailable', 0) / 1024.0
            mem_total_mb = mem.get('MemTotal', 0) / 1024.0
            load1 = os.getloadavg()[0]
            rss_mb = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0
        except Exception as exc:
            mem_avail_mb = mem_total_mb = load1 = rss_mb = -1.0
            self.stats['system_metric_errors'] += 1
            self.get_logger().warning(f'SYSTEM_METRIC_ERROR error={exc!r}')
        self.get_logger().info(
            f'HEALTH depth_frames={self.stats["depth_frames"]} cloud_frames={self.stats["cloud_frames"]} '
            f'cloud_points={self.stats["cloud_points_est"]} '
            f'cloud_frame_mismatch={self.stats["cloud_frame_mismatch"]} '
            f'invalid_stamp={self.stats["invalid_cloud_stamp"]} '
            f'odom_frames={self.stats["odom_frames"]} mesh_msgs={self.stats["mesh_msgs"]} '
            f'blocks={len(self.mesh_blocks)} vertices={self.stats["mesh_vertices"]} '
            f'triangles={self.stats["mesh_triangles"]} esdf_msgs={self.stats["esdf_msgs"]} '
            f'esdf_cells={self.stats["esdf_cells"]} mesh_age_s={age_mesh:.3f} '
            f'esdf_age_s={age_esdf:.3f} '
            f'support_triangles={self.stats["support_triangles"]} '
            f'rejected_triangles={self.stats["rejected_triangles"]} '
            f'support_cells={self.stats["support_cells"]} '
            f'yaw_free_states={self.stats.get("yaw_free_states", 0)} '
            f'yaw_test_states={self.stats.get("yaw_test_states", 0)} '
            f'mesh_update_ms={self.stats.get("mesh_update_ms_last", "none")} '
            f'mem_available_mb={mem_avail_mb:.1f} mem_total_mb={mem_total_mb:.1f} '
            f'load1={load1:.2f} adapter_rss_mb={rss_mb:.1f}')


def main(args=None):
    rclpy.init(args=args)
    node = SurfaceAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
