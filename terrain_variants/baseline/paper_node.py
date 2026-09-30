#!/usr/bin/env python3
"""Read-only ROS adapter for the incremental SE(2) mesh pipeline.

Mesh reception has a separate executor. Geometry is mutated only by the main
executor; an inbox coalesces per-block replacements without dropping deltas.
The last complete generation stays visible while the next one is built.
"""

import json
import copy
import math
import os
from pathlib import Path
import tempfile
import threading
import time
from dataclasses import asdict, fields

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from nvblox_msgs.msg import Mesh
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import SingleThreadedExecutor
try:
    from rclpy.executors import ExternalShutdownException
except ImportError:
    class ExternalShutdownException(Exception):
        pass
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from paper_pipeline import Config, Engine


def json_value(value):
    """Convert NumPy and unbounded interval values into portable JSON."""
    if isinstance(value, dict):
        return {str(key): json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_value(item) for item in value]
    if isinstance(value, np.ndarray):
        return json_value(value.tolist())
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(value) else None
    return value


def polygon_triangles(vertices):
    """Fan-triangulate one convex polygon, preserving its vertex coordinates."""
    if len(vertices) < 3:
        return []
    return [vertex for i in range(1, len(vertices) - 1)
            for vertex in (vertices[0], vertices[i], vertices[i + 1])]


class PaperNode(Node):
    def __init__(self):
        super().__init__('se2_terrain_check')
        defaults = Config()
        for field in fields(Config):
            self.declare_parameter(field.name, getattr(defaults, field.name))
        self.config = Config(**{field.name: self.get_parameter(field.name).value
                                for field in fields(Config)})
        options = dict(output_frame='nvblox_odom', mesh_topic='/nvblox_node/mesh',
                       odom_topic='/nvblox_lio/odom', update_period_s=0.1,native_update_period_s=.005,
                       max_slabs_per_tick=4, max_inbox_blocks=20000,
                       input_timeout_s=5.0, publish_debug_intervals=False,
                       publish_rejected_cells=True, debug_interval_height_m=3.0,
                       output_json='/home/nvidia/scanplanner_test/se2_terrain_check/output/latest_navmesh.json')
        for name, value in options.items():
            self.declare_parameter(name, value)
        self.options = {name: self.get_parameter(name).value for name in options}
        self.frame = self.options['output_frame']
        self.engine = self.create_engine()
        self.lock = threading.RLock()
        self.inbox = {}
        self.reset_pending = False
        self.epoch = 0
        self.input_sequence = 0
        self.mesh_received = self.odom_received = -math.inf
        self.odom_position = None
        self.mesh_size = None
        self.input_error = ''
        self.compute_error = ''
        self.export_error = ''
        self.frame_rejects = 0
        self.mesh_messages = 0
        self.published_revision = None
        self.published_epoch = None
        self.last_document = None
        self.last_rendered = None
        self.last_visual_history = None
        self.debug_marker_counts = {'occupied': 0, 'free_interval': 0}
        self.last_validity = None
        self.last_metrics = {}
        self.last_log = 0.0
        retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers_pub = self.create_publisher(MarkerArray, '/se2_terrain/markers', retained)
        self.completed_pub = self.create_publisher(MarkerArray, '/se2_navmesh/completed_markers', retained)
        self.solids_pub = self.create_publisher(MarkerArray, '/se2_navmesh/solid_intervals', retained)
        self.free_pub = self.create_publisher(MarkerArray, '/se2_navmesh/free_intervals', retained)
        self.map_pub = self.create_publisher(String, '/se2_navmesh/map', retained)
        self.status_pub = self.create_publisher(String, '/se2_terrain/status', retained)
        self.io_node = Node('se2_navmesh_input', use_global_arguments=False)
        self.create_mesh_subscription()
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.io_node.create_subscription(Odometry, self.options['odom_topic'], self.receive_odom,
                                         sensor_qos)
        self.io_executor = SingleThreadedExecutor()
        self.io_executor.add_node(self.io_node)
        def spin_io():
            try:
                self.io_executor.spin()
            except ExternalShutdownException:
                pass
        self.io_thread = threading.Thread(target=spin_io, daemon=True)
        self.io_thread.start()
        self.add_on_set_parameters_callback(self.parameters_changed)
        period=self.options['native_update_period_s'] if getattr(self.engine,'native_mesh',False) else self.options['update_period_s']
        self.create_timer(float(period), self.tick)
        self.get_logger().info('START paper mesh pipeline; robot command outputs=none; config='
                               + str(self.config))

    def create_mesh_subscription(self):
        self.io_node.create_subscription(Mesh, self.options['mesh_topic'], self.receive_mesh, 100)

    def create_engine(self):
        return Engine(self.config)

    def parameters_changed(self, parameters):
        # RealtimeNode adds scheduling controls after the base node has
        # installed this callback. They are safe scalar runtime parameters;
        # geometry parameters still require a restart.
        for parameter in parameters:
            if parameter.name in ('compute_budget_ms', 'global_period_s'):
                if (not isinstance(parameter.value, (int, float))
                        or not math.isfinite(float(parameter.value))
                        or float(parameter.value) < 0.):
                    return SetParametersResult(successful=False,
                                               reason='Scheduling values must be finite and nonnegative')
                continue
            if parameter.name == 'selected_yaw_deg':
                continue
            return SetParametersResult(successful=False,
                                       reason='Restart node to rebuild mesh geometry/configuration')
        return SetParametersResult(successful=True)

    def receive_odom(self, msg):
        with self.lock:
            if msg.header.frame_id != self.frame:
                self.frame_rejects += 1
                return
            position = msg.pose.pose.position
            self.odom_position = [position.x, position.y, position.z]
            self.odom_received = time.monotonic()

    def receive_mesh(self, msg):
        with self.lock:
            if msg.header.frame_id != self.frame:
                self.frame_rejects += 1
                self.input_error = 'Rejected mesh in frame ' + msg.header.frame_id
                return
            if len(msg.block_indices) != len(msg.blocks) or not math.isfinite(msg.block_size_m) or msg.block_size_m <= 0:
                self.input_error = 'Invalid mesh schema; waiting for a full mesh reset'
                return
            if msg.clear:
                self.epoch += 1
                self.inbox.clear()
                self.reset_pending = True
                self.input_error = ''
                self.mesh_size = msg.block_size_m
                # Reset cannot wait for expensive geometry work. Publication is
                # epoch-guarded, so a concurrent older computation stays hidden.
                self.clear_outputs_locked('Mesh reset; rebuilding')
            elif self.mesh_size is not None and abs(self.mesh_size - msg.block_size_m) > 1e-6:
                self.input_error = 'Mesh block size changed without reset'
                return
            if self.input_error:
                return
            self.mesh_size = msg.block_size_m
            for key, block in zip(msg.block_indices, msg.blocks):
                index = (int(key.x), int(key.y), int(key.z))
                if index not in self.inbox and len(self.inbox) >= self.options['max_inbox_blocks']:
                    self.input_error = 'Mesh inbox capacity exceeded; waiting for a full mesh reset'
                    break
                self.inbox[index] = block
            self.mesh_received = time.monotonic()
            self.mesh_messages += 1
            self.input_sequence += 1

    def empty_markers(self):
        marker = Marker()
        marker.header.frame_id = self.frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.action = Marker.DELETEALL
        return MarkerArray(markers=[marker])

    def clear_outputs_locked(self, reason):
        markers = self.empty_markers()
        self.markers_pub.publish(markers)
        self.completed_pub.publish(markers)
        self.solids_pub.publish(markers)
        self.free_pub.publish(markers)
        self.last_document = None
        self.last_rendered = None
        self.last_visual_history = None
        self.debug_marker_counts = {'occupied': 0, 'free_interval': 0}
        self.published_epoch = None
        self.published_revision = None
        empty = dict(frame=self.frame, input_epoch=self.epoch, valid=False,
                     updating=True, reason=reason, polygons=[], graph={},
                     control_outputs='none')
        self.map_pub.publish(String(data=json.dumps(empty)))
        self.status_pub.publish(String(data=json.dumps(empty)))
        destination = self.options['output_json']
        if destination:
            try:
                os.unlink(destination)
            except FileNotFoundError:
                pass
            except OSError as exc:
                self.export_error = str(exc)

    def tick(self):
        started = time.monotonic()
        with self.lock:
            # Finish a frozen revision before admitting more updates. The I/O
            # side keeps coalescing latest replacements for the next revision.
            if self.reset_pending or not self.engine.pending:
                batch, self.inbox = self.inbox, {}
                clear, self.reset_pending = self.reset_pending, False
            else:
                batch, clear = {}, False
            epoch = self.epoch
        try:
            # A malformed delta loses synchronization with the publisher. Latch
            # that failure until a full reset; later valid deltas cannot repair it.
            if clear:
                self.compute_error = ''
            if not self.compute_error:
                decoded = {}
                for key, block in batch.items():
                    vertices = np.array([(point.x, point.y, point.z) for point in block.vertices],
                                        dtype=np.float64).reshape((-1, 3))
                    triangles = np.asarray(block.triangles, dtype=np.int64).reshape((-1, 3))
                    decoded[key] = vertices, triangles
                update_metrics = self.engine.update(decoded, clear=clear)
                metrics = self.engine.process(max_slabs=int(self.options['max_slabs_per_tick']))
                self.last_metrics = {**(update_metrics or {}), **(metrics or {})}
        except Exception as exc:
            self.compute_error = f'{type(exc).__name__}: {exc}; waiting for a full mesh reset'
            self.get_logger().error('Mesh pipeline failed: ' + self.compute_error)
        # Building graph/markers is also compute work: do it outside the input
        # lock so mesh reception does not wait on global polygon serialization.
        revision = (self.engine.generation, self.engine.revision)
        document = rendered = None
        # A new delta may arrive while this batch is being processed. Publish
        # the completed revision immediately and label it ``updating`` below;
        # waiting for a permanently quiet sensor would starve the first map.
        try:
            if (not self.engine.pending and not self.compute_error
                    and (revision != self.published_revision or self.published_epoch != epoch)):
                document = self.engine.snapshot()
                document.update(frame=self.frame, config=asdict(self.config), input_epoch=epoch,
                                complete_revision=True,
                                control_outputs='none',
                                stamp=self.get_clock().now().nanoseconds * 1e-9)
                document = json_value(document)
                rendered = self.render_geometry(document)
        except Exception as exc:
            document = rendered = None
            self.compute_error = f'{type(exc).__name__}: {exc}; waiting for a full mesh reset'
            self.get_logger().error('Mesh snapshot failed: '+self.compute_error)
        data = None
        now = time.monotonic()
        with self.lock:
            stale_mesh = now - self.mesh_received > self.options['input_timeout_s']
            stale_odom = now - self.odom_received > self.options['input_timeout_s']
            updating = bool(self.engine.pending or self.inbox or self.reset_pending or epoch != self.epoch)
            valid = not (stale_mesh or self.input_error or self.compute_error or updating)
            if (document is not None and epoch == self.epoch and not self.reset_pending
                    and not self.input_error and not self.compute_error):
                # Completed revision geometry is independently inspectable in
                # its semantic colors. Current-input validity remains on the
                # original markers/status, which turn blue while rebuilding.
                self.completed_pub.publish(copy.deepcopy(rendered))
                self.last_rendered = rendered
                self.last_visual_history = None
                self.last_document = document
                self.published_revision = revision
                self.published_epoch = epoch
                self.last_validity = None
            if self.last_rendered is not None and self.last_visual_history != (not valid):
                self.recolor_history(self.last_rendered, not valid)
                self.markers_pub.publish(self.last_rendered)
                self.last_visual_history = not valid
            if self.last_document is not None:
                validity = (valid, updating, stale_mesh, self.input_error, self.compute_error)
                if validity != self.last_validity:
                    self.last_document.update(valid=valid, updating=updating,
                                              stale_mesh=stale_mesh,
                                              input_error=self.input_error,
                                              compute_error=self.compute_error)
                    data = json.dumps(self.last_document, separators=(',', ':'), allow_nan=False)
                    self.map_pub.publish(String(data=data))
                    self.last_validity = validity
            status = dict(pipeline='mesh_bvh_voxel_spans_se2', valid=valid, updating=updating,
                          historical_geometry_retained=True, control_outputs='none',
                          input_epoch=self.epoch, generation=self.engine.generation,
                          revision=self.engine.revision, pending_slabs=len(self.engine.pending),
                          inbox_blocks=len(self.inbox), mesh_blocks=len(self.engine.index.blocks),
                          mesh_messages=self.mesh_messages, frame_rejects=self.frame_rejects,
                          mesh_age_s=now-self.mesh_received, odom_age_s=now-self.odom_received,
                          stale_mesh=stale_mesh, stale_odom=stale_odom,
                          robot_position=self.odom_position, input_error=self.input_error,
                          compute_error=self.compute_error, export_error=self.export_error,
                          tick_ms=(time.monotonic()-started)*1000,
                          published_revision=self.published_revision, metrics=self.last_metrics)
            if self.last_document:
                polygons = self.last_document.get('polygons', [])
                full = (1 << self.config.yaw_bins) - 1
                status.update(polygons=len(polygons),
                              all_yaw_polygons=sum(int(p['yaw_mask']) == full for p in polygons),
                              restricted_polygons=sum(0 < int(p['yaw_mask']) < full for p in polygons))
            self.status_pub.publish(String(data=json.dumps(json_value(status), allow_nan=False)))
        # Export complete revisions, with explicit historical/updating metadata
        # when a newer batch exists. Never export a partially processed graph.
        if data is not None:
            self.export_snapshot(data, epoch)
        if document is not None and self.options['publish_debug_intervals']:
            self.publish_intervals(epoch)
        if now - self.last_log > 10:
            self.get_logger().info(json.dumps(json_value(status), separators=(',', ':')))
            self.last_log = now

    def export_snapshot(self, data, epoch):
        destination = self.options['output_json']
        if not destination:
            return
        temporary = None
        try:
            path = Path(destination)
            path.parent.mkdir(parents=True, exist_ok=True)
            with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent,
                                             prefix=path.name+'.', suffix='.tmp', delete=False) as stream:
                temporary = stream.name
                stream.write(data)
                stream.write('\n')
            with self.lock:
                if epoch == self.epoch:
                    os.replace(temporary, path)
                else:
                    os.unlink(temporary)
            self.export_error = ''
        except OSError as exc:
            self.export_error = str(exc)
            if temporary is not None:
                try:
                    os.unlink(temporary)
                except OSError:
                    pass

    def marker(self, namespace, kind, color, scale):
        result = Marker()
        result.header.frame_id = self.frame
        result.header.stamp = self.get_clock().now().to_msg()
        result.ns = namespace
        result.id = 0
        result.type = kind
        result.action = Marker.ADD
        result.pose.orientation.w = 1.0
        result.scale.x, result.scale.y, result.scale.z = scale
        result.color.r, result.color.g, result.color.b, result.color.a = color
        # Zero lifetime: retain historical results and survive computation gaps.
        return result

    @staticmethod
    def point(vertex, z_offset=0.025):
        return Point(x=float(vertex[0]), y=float(vertex[1]), z=float(vertex[2])+z_offset)

    def recolor_history(self, markers, historical):
        colors = {'all_yaw': (.08, .85, .25, .90),
                  'restricted_yaw': (1., .62, .04, .90),
                  'unknown_or_footprint': (.48, .50, .54, .65),
                  'slope_or_headroom': (.90, .16, .12, .70)}
        for marker in markers.markers:
            if marker.ns in colors:
                color = (.23, .48, .78, .55) if historical else colors[marker.ns]
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = color

    def render_geometry(self, document):
        markers = MarkerArray()
        green = self.marker('all_yaw', Marker.TRIANGLE_LIST, (.08, .85, .25, .90), (1., 1., 1.))
        amber = self.marker('restricted_yaw', Marker.TRIANGLE_LIST, (1., .62, .04, .90), (1., 1., 1.))
        edges = self.marker('polygon_edges', Marker.LINE_LIST, (.05, .08, .12, .95), (.012, 0., 0.))
        # A fixed namespace/id is updated in place by RViz.  We use one marker
        # per semantic class, avoiding DELETEALL and the resulting flicker.
        green.id, amber.id, edges.id = 0, 0, 0
        full = (1 << self.config.yaw_bins) - 1
        for polygon in document.get('polygons', []):
            vertices = polygon['vertices']
            target = green if int(polygon['yaw_mask']) == full else amber
            target.points.extend(self.point(vertex) for vertex in polygon_triangles(vertices))
            for a, b in zip(vertices, vertices[1:]+vertices[:1]):
                edges.points.extend((self.point(a, .03), self.point(b, .03)))
        markers.markers.extend((green, amber, edges))
        if self.options['publish_rejected_cells']:
            gray = self.marker('unknown_or_footprint', Marker.POINTS, (.48, .50, .54, .65),
                               (self.config.resolution*.82, self.config.resolution*.82, 0.))
            red = self.marker('slope_or_headroom', Marker.POINTS, (.90, .16, .12, .70),
                              (self.config.resolution*.82, self.config.resolution*.82, 0.))
            for key, result in sorted(self.engine.results.items()):
                if key in self.engine.pending:
                    continue
                for span, mask, reason in zip(result.spans, result.masks, result.reasons):
                    if int(mask):
                        continue
                    target = red if reason in ('slope', 'headroom') else gray
                    target.points.append(self.point(((span.ix+.5)*self.config.resolution,
                                                     (span.iy+.5)*self.config.resolution, span.z)))
            gray.id, red.id = 0, 0
            markers.markers.extend((gray, red))
        # RViz rejects an empty TRIANGLE_LIST with ADD. Explicit deletion also
        # removes a class that existed in the preceding snapshot.
        for marker in markers.markers:
            if not marker.points:
                marker.action = Marker.DELETE
        return markers

    def publish_intervals(self, epoch):
        solids = MarkerArray()
        free = MarkerArray()
        for key, result in sorted(self.engine.results.items()):
            if key in self.engine.pending:
                continue
            for ix, iy, lo, hi in result.solids:
                # A vertical solid can touch several slabs; display only the
                # slab-owned intersection so it is not drawn repeatedly.
                lo = max(lo, key[2]*self.config.slab_cells)
                hi = min(hi, (key[2]+1)*self.config.slab_cells)
                marker = self.marker('occupied', Marker.CUBE, (.85, .22, .22, .22),
                                     (self.config.resolution*.95, self.config.resolution*.95,
                                      max(self.config.vertical_resolution,
                                          (hi-lo)*self.config.vertical_resolution)))
                marker.id = len(solids.markers)
                marker.pose.position.x = (ix+.5)*self.config.resolution
                marker.pose.position.y = (iy+.5)*self.config.resolution
                marker.pose.position.z = (lo+hi)*.5*self.config.vertical_resolution
                solids.markers.append(marker)
            for span in result.spans:
                ceiling = span.ceiling
                top = min(ceiling, span.z+self.options['debug_interval_height_m'])
                if not math.isfinite(top) or top <= span.z:
                    continue
                marker = self.marker('free_interval', Marker.CUBE, (.1, .5, 1., .07),
                                     (self.config.resolution*.9, self.config.resolution*.9, top-span.z))
                marker.id = len(free.markers)
                marker.pose.position.x = (span.ix+.5)*self.config.resolution
                marker.pose.position.y = (span.iy+.5)*self.config.resolution
                marker.pose.position.z = (span.z+top)*.5
                free.markers.append(marker)
        with self.lock:
            if epoch == self.epoch:
                for namespace, markers in (('occupied', solids), ('free_interval', free)):
                    count = len(markers.markers)
                    for identifier in range(count, self.debug_marker_counts[namespace]):
                        removed = self.marker(namespace, Marker.CUBE, (0., 0., 0., 0.), (1., 1., 1.))
                        removed.id = identifier
                        removed.action = Marker.DELETE
                        markers.markers.append(removed)
                    self.debug_marker_counts[namespace] = count
                self.solids_pub.publish(solids)
                self.free_pub.publish(free)

    def shutdown(self):
        self.io_executor.shutdown(timeout_sec=3.0)
        self.io_thread.join(timeout=3.0)
        self.io_node.destroy_node()


def main():
    rclpy.init()
    node = PaperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
