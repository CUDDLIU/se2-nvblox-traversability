#!/usr/bin/env python3
"""Fast local mask deltas; historical global NavMesh is a separate worker.

No robot commands. Local consumers must handle reset/invalidate/replace events;
completed tiles have the same full-yaw checks as the paper pipeline. Global
output is explicitly historical and must not be substituted for local validity.
"""
import copy
import json
import math
import queue
import threading
import time
import os
import hashlib
from pathlib import Path
from dataclasses import asdict, replace
from collections import Counter
from rcl_interfaces.msg import SetParametersResult

import numpy as np
import rclpy
try:
    from rclpy.executors import ExternalShutdownException
except ImportError:  # lightweight test stubs and older rclpy releases
    class ExternalShutdownException(Exception):
        pass
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from local_updates import LocalUpdates
from local_display import LocalDisplay
from paper_node import PaperNode, json_value
from mesh_wire import block_arrays, check_schema, decode_mesh
from global_export import export_document, lower_priority


class RealtimeNode(PaperNode):
    TUNABLE = {'max_slope_deg','max_step','height','top_margin','side_margin','length','width'}

    def __init__(self):
        self.live_ready = False
        self.config_version = 0
        self.history_document = None
        self.last_history_save = -math.inf
        self.admit_turn = 0
        self.wire_metrics = {}
        self.provenance = LocalUpdates()
        self.receipts = {}
        self.global_queue = queue.Queue(maxsize=1)
        self.global_stop = threading.Event()
        self.last_global_submit = -math.inf
        self.last_status = -math.inf
        self.last_local = None
        self.local_events = 0
        self.local_pub = self.local_markers_pub = None
        self.display = LocalDisplay()
        self.last_display = -math.inf
        self.display_pub = self.validity_display_pub = None
        super().__init__()
        for name, value in dict(compute_budget_ms=35., global_period_s=1.,
                               max_update_blocks=128,native_update_blocks=4096,
                               history_path='/home/nvidia/scanplanner_test/se2_terrain_check/output/local_history.json').items():
            self.declare_parameter(name, value)
            self.options[name] = self.get_parameter(name).value
        self.map_session = os.environ.get('SE2_MAP_SESSION','')
        try:
            path=Path(self.options['history_path'])
            history=json.loads(path.read_text())
            if not self.display.load_document(history,self.frame,self.map_session):
                # Keep a previous map session on disk, but never overlay its
                # unrelated odom origin onto the new map automatically.
                session_id=hashlib.sha256(str(history.get('map_session','')).encode()).hexdigest()[:12]
                saved=path.with_name('local_history_'+session_id+'.json')
                if not saved.exists():saved.write_text(json.dumps(history))
        except (OSError, ValueError, KeyError):
            pass
        self.restored_history_cells=sum(len(rows) for rows in self.display.archive_cells.values())
        # Deltas are not a self-contained retained snapshot. Consumers must
        # subscribe before scanning, or bootstrap from a same-epoch global map.
        self.local_pub = self.create_publisher(String, '/se2_navmesh/local_result', 100)
        self.local_markers_pub = self.create_publisher(MarkerArray, '/se2_navmesh/local_markers', 100)
        retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.display_pub = self.create_publisher(MarkerArray, '/se2_navmesh/local_display', retained)
        self.validity_display_pub = self.create_publisher(
            MarkerArray, '/se2_navmesh/local_validity_display', retained)
        from concurrent.futures import ProcessPoolExecutor
        import multiprocessing
        self.export_pool=ProcessPoolExecutor(max_workers=1,mp_context=multiprocessing.get_context('spawn'),
                                             initializer=lower_priority)
        self.global_thread = threading.Thread(target=self.global_loop, daemon=True)
        self.global_thread.start()
        self.live_ready = True

    def create_mesh_subscription(self):
        from nvblox_msgs.msg import Mesh, MeshBlock, Index3D
        check_schema(Mesh, MeshBlock, Index3D)
        self.io_node.create_subscription(Mesh, self.options['mesh_topic'],
                                         self.receive_serialized_mesh, 100, raw=True)

    def create_engine(self):
        from paper_pipeline import Engine
        return Engine(self.config,native_mesh=True)

    def receive_serialized_mesh(self, raw):
        started = time.monotonic()
        try:
            msg = decode_mesh(raw)
        except (ValueError, UnicodeError) as exc:
            with self.lock:
                self.input_error = f'Invalid Mesh wire data: {exc}; waiting for a full mesh reset'
            return
        decoded = time.monotonic()
        self.receive_mesh(msg, received=started)
        with self.lock:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
            self.wire_metrics = dict(bytes=len(raw), blocks=len(msg.blocks),
                decode_ms=(decoded-started)*1000,
                receive_ms=(time.monotonic()-started)*1000,
                source_age_ms=(self.get_clock().now().nanoseconds*1e-9-stamp)*1000)

    def parameters_changed(self, parameters):
        if not self.live_ready:
            return SetParametersResult(successful=True)
        try:
            values = {p.name:p.value for p in parameters}
            if set(values)-self.TUNABLE:
                raise ValueError('Only physical traversal thresholds can be changed live')
            replace(self.config, **values)
            return SetParametersResult(successful=True)
        except (TypeError, ValueError) as exc:
            return SetParametersResult(successful=False,reason=str(exc))

    def receive_mesh(self, msg, received=None):
        received = time.monotonic() if received is None else received
        with self.lock:
            before = self.input_sequence
            super().receive_mesh(msg)
            if self.input_sequence == before:
                return
            if msg.clear:
                self.receipts.clear()
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
            for index in msg.block_indices:
                key = (int(index.x), int(index.y), int(index.z))
                old = self.receipts.get(key)
                if key in self.inbox:
                    self.receipts[key] = (old[0] if old else received, received,
                                          self.input_sequence, stamp)

    def clear_outputs_locked(self, reason):
        super().clear_outputs_locked(reason)
        self.display.archive()
        self.last_display = -math.inf
        if self.display_pub is not None:
            self.publish_display(stale=True)
        if self.local_pub is not None:
            self.local_pub.publish(String(data=json.dumps(dict(
                event='reset', input_epoch=self.epoch, valid=False, reason=reason))))
            self.local_markers_pub.publish(self.empty_markers())

    def local_markers(self, keys, invalidate=False):
        markers = MarkerArray()
        # RViz uses the retained aggregate display by default. Building and
        # serializing hundreds of unused per-slab markers consumed main-loop
        # time even with no subscriber; local_result remains authoritative.
        if (hasattr(self.local_markers_pub,'get_subscription_count') and
                self.local_markers_pub.get_subscription_count()==0):
            return markers
        full = (1 << self.config.yaw_bins)-1
        res = self.config.resolution
        for key in keys:
            namespace = 'tile_'+'_'.join(map(str, key))
            groups = [self.marker(namespace, Marker.CUBE_LIST, color, (res, res, .012))
                      for color in ((.08, .85, .25, .90), (1., .62, .04, .90))]
            for identifier, marker in enumerate(groups):
                marker.id = identifier
                marker.action = Marker.DELETE if invalidate else Marker.ADD
            if not invalidate:
                result = self.engine.results.get(key)
                if result:
                    for span, mask in zip(result.spans, result.masks):
                        if mask:
                            groups[0 if int(mask) == full else 1].points.append(self.point(
                                ((span.ix+.5)*res, (span.iy+.5)*res, span.z)))
            markers.markers.extend(groups)
        for marker in markers.markers:
            if not marker.points:
                marker.action = Marker.DELETE
        return markers

    def publish_display(self, stale=False):
        """Publish a self-contained retained view for late RViz subscribers.

        Called under the epoch lock. Repeating this view is visualization only
        and does not increment local_events or publish a local_result event.
        """
        snapshot = self.display.snapshot(self.config.yaw_bins, stale=stale)
        colors = dict(green=(.08, .85, .25, .90), amber=(1., .62, .04, .90),
                      current_green=(.08, .85, .25, .90), current_amber=(1., .62, .04, .90),
                      historical=(.23, .48, .78, .55))
        colors['archived']=(.23,.48,.78,.65)
        def render(names):
            result = MarkerArray()
            for name in names:
                marker = self.marker(name, Marker.CUBE_LIST, colors[name],
                                     (self.config.resolution, self.config.resolution, .012))
                marker.points = [self.point(((ix+.5)*self.config.resolution,
                                             (iy+.5)*self.config.resolution,
                                             iz*self.config.vertical_resolution))
                                 for ix, iy, iz in snapshot[name]]
                if not marker.points:
                    marker.action = Marker.DELETE
                result.markers.append(marker)
            return result
        self.display_pub.publish(render(('green', 'amber','archived')))
        self.validity_display_pub.publish(render(('current_green', 'current_amber', 'historical','archived')))
        return {name: len(rows) for name, rows in snapshot.items()}

    def tick(self):
        started = time.monotonic()
        with self.lock:
            self.admit_turn += 1
            keys=list(self.inbox)
            if self.odom_position is not None and self.admit_turn%4:
                x,y=self.odom_position[:2]
                size=self.mesh_size or .4
                # Stable partition prioritizes new nearby blocks without
                # starving old blocks: every fourth batch remains FIFO.
                keys.sort(key=lambda k: ((k[0]+.5)*size-x)**2+((k[1]+.5)*size-y)**2>25)
            limit=(self.options.get('native_update_blocks',4096) if self.engine.native_mesh
                   else self.options.get('max_update_blocks',128))
            keys=keys[:int(limit)]
            batch={k:self.inbox.pop(k) for k in keys}
            receipts={k:self.receipts.pop(k) for k in keys}
            clear, self.reset_pending = self.reset_pending, False
            epoch, input_sequence = self.epoch, self.input_sequence
        if clear:
            self.compute_error = ''
            self.provenance.clear()
        try:
            values={name:self.get_parameter(name).value for name in self.TUNABLE}
            candidate=replace(self.config,**values)
            config_dirty=set()
            if candidate!=self.config:
                self.config=candidate
                config_dirty=self.engine.reconfigure(candidate)
                self.config_version+=1
                with self.lock:
                    self.display.parameters_changed()
                    self.display.invalidate(config_dirty)
                    self.local_pub.publish(String(data=json.dumps(dict(event='parameters',
                        input_epoch=epoch,config_version=self.config_version,config=asdict(candidate),
                        valid=False))))
                self.provenance.invalidate(config_dirty,started,input_sequence,None)
            if not self.compute_error and not self.input_error:
                stage_start = time.monotonic()
                decoded = {key: block_arrays(block) for key, block in batch.items()}
                decode_ms = (time.monotonic()-stage_start)*1000
                stage_start = time.monotonic()
                metrics = self.engine.update(decoded, clear=clear)
                metrics.update(array_decode_ms=decode_ms, bvh_update_ms=(time.monotonic()-stage_start)*1000)
                stage_start=time.monotonic()
                dirty = sorted(self.engine.dirty_keys | (config_dirty if not clear else set()))
                if dirty:
                    oldest = min((r[0] for r in receipts.values()),default=started)
                    newest = max((r[1] for r in receipts.values()),default=started)
                    latest = max(receipts.values(), key=lambda r: r[2],default=(started,started,input_sequence,None))
                    self.provenance.invalidate(dirty, oldest, latest[2], latest[3], newest)
                    invalid = dict(event='invalidate', input_epoch=epoch,
                                   input_sequence=input_sequence, revision=self.engine.revision,
                                   invalidated_slabs=dirty, valid=False)
                    markers = self.local_markers(dirty, invalidate=True)
                    with self.lock:
                        if epoch == self.epoch and not self.reset_pending:
                            self.display.invalidate(dirty)
                            self.local_pub.publish(String(data=json.dumps(invalid)))
                            self.local_markers_pub.publish(markers)
                metrics['invalidation_ms']=(time.monotonic()-stage_start)*1000
                # Admission has its own bounded block budget. Always perform
                # classification even when admission exceeded the time target.
                processed = self.engine.process(budget_ms=self.options['compute_budget_ms'],
                    defer_polygons=True,priority_position=self.odom_position)
                self.last_metrics = {**metrics, **processed}
                keys = self.engine.completed_keys
                output = self.provenance.finish(self.engine, keys)
                stage_start=time.monotonic()
                if output:
                    markers = self.local_markers(keys)
                    output.update(event='replace', input_epoch=epoch, frame=self.frame,
                                  input_sequence=input_sequence,
                                  config_version=self.config_version,
                                  resolution=self.config.resolution,
                                  vertical_resolution=self.config.vertical_resolution,
                                  yaw_bins=self.config.yaw_bins, control_outputs='none')
                    with self.lock:
                        if (epoch == self.epoch and not self.reset_pending
                                and not self.input_error):
                            output.update(current_input=input_sequence == self.input_sequence,
                                          global_updating=bool(self.engine.pending or self.inbox),
                                          output_stamp=self.get_clock().now().nanoseconds*1e-9)
                            output['complete_input'] &= not bool(self.inbox)
                            # Includes decode, queue wait, mask calculation and
                            # marker preparation, not merely Engine.process().
                            emitted = time.monotonic()
                            output['callback_ms'] = (emitted-started)*1000
                            output['output_monotonic'] = emitted
                            output['latency_oldest_ms'] = (emitted-output['input_received_monotonic'])*1000
                            output['latency_newest_ms'] = (emitted-output['newest_input_received_monotonic'])*1000
                            for slab in output['updated_slabs']:
                                slab['latency_ms'] = (emitted-slab['input_received_monotonic'])*1000
                            self.local_pub.publish(String(data=json.dumps(output, separators=(',', ':'))))
                            self.display.replace(output['updated_slabs'])
                            self.local_markers_pub.publish(markers)
                            self.last_local = output
                            self.local_events += 1
                self.last_metrics['result_publish_ms']=(time.monotonic()-stage_start)*1000
                now = time.monotonic()
                if (not self.engine.pending and now-self.last_global_submit >= self.options['global_period_s']
                        and (self.engine.generation, self.engine.revision) != self.published_revision):
                    view = copy.copy(self.engine)
                    view.results, view.pending = self.engine.results.copy(), set()
                    if not self.global_queue.full():
                        self.global_queue.put_nowait((epoch, input_sequence, self.config_version, view))
                        self.last_global_submit = now
        except Exception as exc:
            self.compute_error = f'{type(exc).__name__}: {exc}; waiting for a full mesh reset'
            self.get_logger().error(self.compute_error)
        now = time.monotonic()
        if now-self.last_status >= .1:
            with self.lock:
                stale = bool(self.input_error or self.compute_error or self.inbox or self.reset_pending
                             or now-self.mesh_received > self.options['input_timeout_s'])
                counts = {}
                if now-self.last_display >= .1:
                    counts = self.publish_display(stale=stale)
                    self.last_display = now
                if now-self.last_history_save >= 2.:
                    self.history_document=self.display.to_document(self.frame,self.map_session,asdict(self.config))
                    self.last_history_save=now
                reasons=Counter(reason for key,result in self.engine.results.items()
                                if key not in self.engine.pending for reason in result.reasons)
                status = dict(pipeline='incremental_local_se2', input_epoch=self.epoch,
                              revision=self.engine.revision, pending_slabs=len(self.engine.pending),
                              inbox_blocks=len(self.inbox), mesh_messages=self.mesh_messages,
                              local_events=self.local_events, compute_error=self.compute_error,
                              input_error=self.input_error, export_error=self.export_error,
                              mesh_age_s=now-self.mesh_received, metrics=self.last_metrics,
                              tick_ms=(now-started)*1000, control_outputs='none',
                              display_counts=counts, display_historical_only=True)
                status.update(config=asdict(self.config),config_version=self.config_version,
                              robot_position=self.odom_position,reasons=dict(reasons),
                              latency_ms=self.last_local.get('latency_oldest_ms') if self.last_local else None,
                              history_slabs=len(self.display.archive_cells))
                status['restored_history_cells']=getattr(self,'restored_history_cells',0)
                status['wire']=getattr(self,'wire_metrics',{})
                status['odom_age_s']=now-self.odom_received
                self.status_pub.publish(String(data=json.dumps(json_value(status))))
            self.last_status = now

    def global_loop(self):
        while not self.global_stop.is_set():
            with self.lock:
                history,self.history_document=self.history_document,None
            if history is not None:
                try:
                    path=Path(self.options['history_path'])
                    path.parent.mkdir(parents=True,exist_ok=True)
                    temporary=path.with_suffix('.tmp')
                    temporary.write_text(json.dumps(history,separators=(',',':')))
                    os.replace(temporary,path)
                except OSError as exc:
                    self.export_error=str(exc)
            try:
                epoch, sequence, version, view = self.global_queue.get(timeout=.1)
            except queue.Empty:
                continue
            try:
                metadata=dict(frame=self.frame, input_epoch=epoch, input_sequence=sequence,
                                historical_only=True, valid=False, control_outputs='none',
                                stamp=self.get_clock().now().nanoseconds*1e-9)
                data=self.export_pool.submit(export_document,view.config,view.results,
                    view.generation,view.revision,metadata).result()
                rendered=None
                if self.completed_pub.get_subscription_count():
                    renderer=copy.copy(self);renderer.engine=view
                    rendered=renderer.render_geometry(json.loads(data))
                with self.lock:
                    if epoch != self.epoch or self.reset_pending or version!=self.config_version:
                        continue
                    if rendered is not None:self.completed_pub.publish(rendered)
                    self.map_pub.publish(String(data=data))
                    self.published_revision = (view.generation, view.revision)
                self.export_snapshot(data, epoch)
            except Exception as exc:
                self.export_error = f'Global output: {type(exc).__name__}: {exc}'
                self.get_logger().error(self.export_error)

    def shutdown(self):
        # Join the writer before the final flush; both use the same atomic
        # temporary path and must never race during a service restart.
        self.global_stop.set()
        self.global_thread.join()
        self.export_pool.shutdown(wait=True)
        with self.lock:
            if self.options.get('history_path'):
                path=Path(self.options['history_path'])
                path.parent.mkdir(parents=True,exist_ok=True)
                temporary=path.with_suffix('.tmp')
                temporary.write_text(json.dumps(self.display.to_document(self.frame,self.map_session,asdict(self.config))))
                os.replace(temporary,path)
        super().shutdown()


def main():
    rclpy.init()
    node = RealtimeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
