#!/usr/bin/env python3
"""Independent read-only ROS2 terrain diagnostics; never sends robot commands."""
import json
import math
import struct
import time
import threading
import sys
from collections import OrderedDict, deque
from dataclasses import fields

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from nvblox_msgs.msg import Mesh
from tf2_ros import Buffer, TransformListener, TransformException

from terrain import (Config, Terrain, rasterize, footprint_masks, evaluate,
                     required_free_keys, certify_free, observed_clearance_tops,
                     invalidate_occupied_evidence, geometry_display_state)
from fast_raster import rasterize, rasterize_blocks


def transform_matrix(tf):
    q = tf.rotation
    x,y,z,w = q.x,q.y,q.z,q.w
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    if norm < 1e-9:
        raise ValueError('Invalid zero TF quaternion')
    x,y,z,w = x/norm,y/norm,z/norm,w/norm
    out = np.eye(4)
    out[:3,:3] = [[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                  [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                  [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
    out[:3,3] = [tf.translation.x,tf.translation.y,tf.translation.z]
    return out


class TerrainNode(Node):
    def __init__(self):
        super().__init__('se2_terrain_check')
        defaults = Config()
        for field in fields(Config):
            self.declare_parameter(field.name, getattr(defaults,field.name))
        self.config = Config(**{f.name:self.get_parameter(f.name).value for f in fields(Config)})
        for name, value in dict(output_frame='nvblox_odom', mesh_topic='/nvblox_node/mesh',
                                depth_topic='/camera/d435i/depth/image_rect_raw',
                                info_topic='/camera/d435i/depth/camera_info',
                                odom_topic='/nvblox_lio/odom', window_radius_m=2.,
                                update_period_s=1., raster_budget_s=.35,
                                evidence_ttl_s=5., input_timeout_s=3., selected_yaw_deg=0.,
                                depth_margin_m=.04, max_range_m=4.).items():
            self.declare_parameter(name,value)
        self.frame = self.get_parameter('output_frame').value
        self.terrain = Terrain(self.config)
        self.masks = footprint_masks(self.config)
        self.pending = OrderedDict()
        self.block_signatures = {}
        self.stage_ms = {}
        self.mesh_inbox = deque()
        self.mesh_inbox_lock = threading.Lock()
        self.mesh_inbox_overflow = False
        self.mesh_input_seq = 0
        self.snapshot_input_seq = 0
        self.snapshot_oldest_received = None
        self.snapshot_newest_received = None
        self.evidence = {}
        self.info = self.depth = self.odom = None
        self.mesh_received = self.depth_received = self.odom_received = -math.inf
        self.last_depth_stamp = None
        self.depth_queue = deque(maxlen=30)
        self.used_depth_age_s = None
        self.mesh_size = None
        self.last_error = ''
        self.mesh_error = ''
        self.frame_rejects = 0
        self.last_log = 0.
        self.generation = 0
        # TF reception must not wait for mesh processing on this node.
        self.tf_node = Node('se2_terrain_tf_listener', use_global_arguments=False)
        self.tf_node.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', value=self.get_parameter('use_sim_time').value)])
        self.tf_buffer = Buffer(node=self.tf_node)
        self.tf_listener = TransformListener(self.tf_buffer,self.tf_node)
        self.tf_executor = SingleThreadedExecutor()
        self.tf_executor.add_node(self.tf_node)
        self.tf_thread = threading.Thread(target=self.tf_executor.spin, daemon=True)
        self.tf_thread.start()
        retained_qos = QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers_pub = self.create_publisher(MarkerArray,'/se2_terrain/markers',retained_qos)
        self.spans_pub = self.create_publisher(PointCloud2,'/se2_terrain/spans',1)
        self.status_pub = self.create_publisher(String,'/se2_terrain/status',1)
        sensor_qos = QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.tf_node.create_subscription(Mesh,self.get_parameter('mesh_topic').value,self.receive_mesh,100)
        # Receive the latest sensor state on the independent I/O executor too;
        # fitting a large mesh refresh must not make valid inputs look stale.
        self.tf_node.create_subscription(Image,self.get_parameter('depth_topic').value,self.on_depth,sensor_qos)
        self.tf_node.create_subscription(CameraInfo,self.get_parameter('info_topic').value,self.on_info,sensor_qos)
        self.tf_node.create_subscription(Odometry,self.get_parameter('odom_topic').value,self.on_odom,sensor_qos)
        self.create_timer(self.get_parameter('update_period_s').value,self.tick)
        self.add_on_set_parameters_callback(self.parameters_changed)
        self.get_logger().info('START geometry_only=true control_outputs=none configuration='+str(self.config))

    def parameters_changed(self, parameters):
        # Geometry changes require a rebuild, not a silent parameter-only edit.
        for p in parameters:
            if p.name != 'selected_yaw_deg':
                return SetParametersResult(successful=False,reason='Restart node to rebuild geometry parameters')
            if not isinstance(p.value,(int,float)) or not math.isfinite(p.value):
                return SetParametersResult(successful=False,reason='Yaw must be finite')
        return SetParametersResult(successful=True)

    def on_info(self,msg):
        self.info = msg

    def on_depth(self,msg):
        self.depth = msg
        self.depth_queue.append(msg)
        self.depth_received = time.monotonic()

    def on_odom(self,msg):
        if msg.header.frame_id != self.frame:
            self.frame_rejects += 1
            return
        self.odom = msg
        self.odom_received = time.monotonic()

    def on_mesh(self,msg):
        if msg.header.frame_id != self.frame:
            self.frame_rejects += 1
            return
        if len(msg.block_indices) != len(msg.blocks) or msg.block_size_m <= 0:
            self.mesh_error = 'Invalid mesh block schema; awaiting a full map reset'
            return
        if msg.clear:
            self.terrain.clear()
            self.pending.clear()
            self.block_signatures.clear()
            self.evidence.clear()
            self.mesh_error = ''
            self.generation += 1
        if self.mesh_size is not None and abs(self.mesh_size-msg.block_size_m)>1e-6 and not msg.clear:
            self.mesh_error = 'Mesh block size changed without reset'
            return
        self.mesh_size = msg.block_size_m
        for index, block in zip(msg.block_indices,msg.blocks):
            key = (index.x,index.y,index.z)
            self.pending[key] = block

    def receive_mesh(self,msg):
        # I/O only. Geometry/map-reset mutation remains on the compute thread.
        if msg.header.frame_id != self.frame:
            self.frame_rejects += 1
            return
        with self.mesh_inbox_lock:
            if len(self.mesh_inbox) >= 100:
                self.mesh_inbox.popleft()
                self.mesh_inbox_overflow = True
            self.mesh_input_seq += 1
            self.mesh_inbox.append((msg,time.monotonic(),self.mesh_input_seq))
        self.mesh_received = time.monotonic()

    def consume_blocks(self):
        started = time.monotonic()
        processed = 0
        skipped = 0
        dirty = set()
        batch = {}
        signatures = {}
        budget = self.get_parameter('raster_budget_s').value
        while self.pending and time.monotonic()-started < budget:
            key, block = self.pending.popitem(last=False)
            try:
                vertices = np.asarray([(p.x,p.y,p.z) for p in block.vertices],dtype=np.float64).reshape(-1,3)
                triangles = np.asarray(block.triangles,dtype=np.int64)
                signature = (vertices.tobytes(),triangles.tobytes())
                if self.block_signatures.get(key) == signature or (not len(triangles) and key not in self.terrain.blocks):
                    skipped += 1
                    continue
                batch[key] = (vertices,triangles)
                signatures[key] = signature
            except (ValueError,IndexError) as exc:
                self.mesh_error = f'Invalid mesh block {key}: {exc}'
        try:
            batches = rasterize_blocks(batch,self.config)
            self.block_signatures.update(signatures)
        except (ValueError,IndexError) as exc:
            self.mesh_error = f'Invalid mesh batch: {exc}; awaiting a full map reset'
            batches = {key:{} for key in batch}
        for key,records in batches.items():
            dirty.update(self.terrain.blocks.get(key,{}))
            dirty.update(records)
            self.terrain.replace_block(key,records)
            processed += 1
        raster_end = time.monotonic()
        self.terrain.rebuild()
        rebuild_end = time.monotonic()
        if dirty:
            self.evidence = invalidate_occupied_evidence(self.evidence,self.terrain,dirty)
        self.stage_ms = dict(raster_ms=round((raster_end-started)*1000,1),
                             rebuild_ms=round((rebuild_end-raster_end)*1000,1),
                             invalidate_ms=round((time.monotonic()-rebuild_end)*1000,1),
                             skipped_unchanged_blocks=skipped)
        return processed

    def observe(self,surfaces,now):
        info = self.info
        if not self.depth_queue or info is None:
            raise ValueError('Waiting for depth and CameraInfo')
        ros_now = self.get_clock().now().nanoseconds*1e-9
        timeout = self.get_parameter('input_timeout_s').value
        msg, tf = None, None
        for candidate in reversed(tuple(self.depth_queue)):
            age = ros_now-(candidate.header.stamp.sec+candidate.header.stamp.nanosec*1e-9)
            if not 0 <= age <= timeout:
                continue
            try:
                tf = self.tf_buffer.lookup_transform(self.frame,candidate.header.frame_id,
                                                     Time.from_msg(candidate.header.stamp))
            except TransformException:
                continue
            msg = candidate
            self.used_depth_age_s = age
            break
        if msg is None:
            raise ValueError('Waiting for exact-time depth TF (no fresh paired frame)')
        stamp = (msg.header.stamp.sec,msg.header.stamp.nanosec)
        if (msg.width,msg.height,msg.header.frame_id) != (info.width,info.height,info.header.frame_id):
            raise ValueError('Depth/CameraInfo frame or dimensions differ')
        if any(abs(d)>1e-9 for d in info.d):
            raise ValueError('Non-rectified CameraInfo is not supported')
        if stamp == self.last_depth_stamp:
            return 0
        matrix = transform_matrix(tf.transform)
        inverse = np.eye(4)
        inverse[:3,:3] = matrix[:3,:3].T
        inverse[:3,3] = -matrix[:3,:3].T @ matrix[:3,3]
        if msg.encoding not in ('16UC1','32FC1'):
            raise ValueError('Unsupported depth encoding '+msg.encoding)
        item_size = 2 if msg.encoding == '16UC1' else 4
        dtype = ('>' if msg.is_bigendian else '<') + ('u2' if item_size==2 else 'f4')
        depth = np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,
                           strides=(msg.step,item_size)).astype(np.float32)
        if item_size==2:
            depth *= .001
        keys = required_free_keys(surfaces,self.config)
        # Bound peak allocation, even if a multi-storey scene has many spans.
        count = 0
        for start in range(0,len(keys),12000):
            chunk = keys[start:start+12000]
            free = certify_free(chunk,depth,(info.k[0],info.k[4],info.k[2],info.k[5]),
                                inverse,self.config,self.get_parameter('depth_margin_m').value,
                                self.get_parameter('max_range_m').value)
            for key in chunk[free]:
                # Evidence lifetime starts at acquisition, not processing.
                self.evidence[tuple(key)] = now-self.used_depth_age_s
            count += int(free.sum())
        self.last_depth_stamp = stamp
        return count

    def tick(self):
        started = time.monotonic()
        with self.mesh_inbox_lock:
            incoming = list(self.mesh_inbox)
            self.mesh_inbox.clear()
            overflow = self.mesh_inbox_overflow
            self.mesh_inbox_overflow = False
        if overflow:
            self.mesh_error = 'Mesh receive overflow; awaiting a full map reset'
        if incoming:
            self.snapshot_oldest_received = incoming[0][1]
            self.snapshot_newest_received = incoming[-1][1]
            self.snapshot_input_seq = incoming[-1][2]
        for msg,_,_ in incoming:
            self.on_mesh(msg)
        processed = self.consume_blocks()
        now = time.monotonic()
        ttl = self.get_parameter('evidence_ttl_s').value
        self.evidence = {k:t for k,t in self.evidence.items() if now-t<=ttl}
        radius = self.get_parameter('window_radius_m').value
        x = self.odom.pose.pose.position.x if self.odom else 0.
        y = self.odom.pose.pose.position.y if self.odom else 0.
        # All cached map columns remain visible in the fixed world frame.
        # Deletions and full resets still remove their actual map geometry.
        surfaces = self.terrain.local_surfaces(x,y,math.inf)
        errors = []
        observation_errors = []
        timeout = self.get_parameter('input_timeout_s').value
        ros_now = self.get_clock().now().nanoseconds*1e-9
        for name,received,msg in (('mesh',self.mesh_received,None),
                                  ('depth',self.depth_received,self.depth),
                                  ('odom',self.odom_received,self.odom)):
            destination = observation_errors if name == 'depth' else errors
            if now-received>timeout:
                destination.append(name+' stale/missing')
            if msg is not None:
                stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                if not -.2 <= ros_now-stamp <= timeout:
                    destination.append(name+' timestamp invalid/stale')
        if self.mesh_error:
            errors.append(self.mesh_error)
        certified = 0
        observe_started = time.monotonic()
        if not errors and not observation_errors:
            try:
                certified = self.observe(surfaces,now)
            except (ValueError,TransformException) as exc:
                observation_errors.append(str(exc))
        self.stage_ms['observe_ms'] = round((time.monotonic()-observe_started)*1000,1)
        evaluate_started = time.monotonic()
        tops = observed_clearance_tops(surfaces,self.evidence,self.config,now,ttl)
        diagnostics = {}
        geom,verified = evaluate(surfaces,self.config,self.masks,tops,diagnostics)
        self.stage_ms['evaluate_ms'] = round((time.monotonic()-evaluate_started)*1000,1)
        # Do not mix old geometry with a partially processed update as a pass.
        if self.pending:
            errors.append('geometry update pending')
        if errors:
            geom[:] = 0
        if errors or observation_errors:
            verified[:] = 0
        selected = int(round(self.get_parameter('selected_yaw_deg').value/360*self.config.yaw_bins)) % self.config.yaw_bins
        visible = list(range(len(surfaces)))
        status = self.publish(surfaces,tops,geom,verified,visible,selected,errors,
                              diagnostics['unknown_support'])
        status.update(generation=self.generation,processed_blocks=processed,
                      **self.stage_ms,
                      pending_blocks=len(self.pending),cached_blocks=len(self.terrain.blocks),
                      column_count=len(self.terrain.columns),free_voxels=len(self.evidence),
                      new_free_voxels=certified,frame_rejects=self.frame_rejects,
                      used_depth_age_s=self.used_depth_age_s,
                      update_ms=round((time.monotonic()-started)*1000,1),
                      footprint_length_m=self.config.length+2*self.config.side_margin,
                      footprint_width_m=self.config.width+2*self.config.side_margin,
                      required_height_m=self.config.height+self.config.top_margin,
                      profile='flat_ground_mesh_se2_geometry',control_outputs=False,
                      display_rule='mesh_all_yaw_safe',state_schema_version=2,
                      history_scope='current_map_all_cached_columns',
                      mesh_snapshot_seq=self.snapshot_input_seq,
                      mesh_inputs_consumed=len(incoming),
                      new_geometry_result=processed>0 and not errors,
                      receive_to_output_newest_ms=(None if self.snapshot_newest_received is None else
                          round((time.monotonic()-self.snapshot_newest_received)*1000,1)),
                      receive_to_output_oldest_ms=(None if self.snapshot_oldest_received is None else
                          round((time.monotonic()-self.snapshot_oldest_received)*1000,1)),
                      errors=errors,observation_errors=observation_errors)
        self.status_pub.publish(String(data=json.dumps(status)))
        if time.monotonic()-self.last_log>=5:
            self.last_log=time.monotonic()
            self.get_logger().info('HEALTH '+json.dumps(status))

    def publish(self,surfaces,tops,geom,verified,visible,selected,errors,unknown_support):
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        colors = [(0.15,.9,.25,.9),(1.,.65,.05,.8),(.95,.15,.15,.7),(.5,.5,.5,.5),(.7,.35,.95,.6)]
        names = ['mesh_safe_all_yaw','mesh_restricted_yaw','geometric_reject','unknown_support','stale_or_updating']
        groups = []
        for i,(color,name) in enumerate(zip(colors,names)):
            m = Marker()
            m.header.frame_id = self.frame
            m.header.stamp = stamp
            m.ns = name
            m.id = i
            m.type = Marker.POINTS
            m.action = Marker.ADD
            m.pose.orientation.w = 1.
            m.scale.x = m.scale.y = self.config.resolution*.85
            m.color.r,m.color.g,m.color.b,m.color.a = color
            # Replace the same five namespace/id pairs, never blank the display
            # between updates. Empty groups remove old points in that category.
            m.lifetime.sec = 0
            groups.append(m)
        packet = struct.Struct('<fffffIIIIII')
        data = bytearray()
        counts = [0]*5
        for i in visible:
            s = surfaces[i]
            gm,vm = int(geom[i]),int(verified[i])
            state = geometry_display_state(gm,int(unknown_support[i]),self.config.yaw_bins,not errors)
            counts[state] += 1
            px,py = (s.ix+.5)*self.config.resolution,(s.iy+.5)*self.config.resolution
            groups[state].points.append(Point(x=px,y=py,z=s.z+.015))
            # Infinity here means no observed mesh ceiling, NOT known free.
            data.extend(packet.pack(px,py,s.z,s.ceiling-s.z,max(0.,tops[i]-s.z),
                                    gm&0xffffffff,gm>>32,vm&0xffffffff,vm>>32,int(s.covered),state))
        markers.markers.extend(groups)
        self.markers_pub.publish(markers)
        cloud = PointCloud2()
        cloud.header.frame_id = self.frame
        cloud.header.stamp = stamp
        names = ['x','y','z','mesh_clearance','observed_clearance','geometry_yaw_lo',
                 'geometry_yaw_hi','observed_yaw_lo','observed_yaw_hi','covered','state']
        cloud.fields = [PointField(name=name,offset=i*4,
                                  datatype=PointField.FLOAT32 if i<5 else PointField.UINT32,count=1)
                        for i,name in enumerate(names)]
        cloud.height = 1
        cloud.width = len(visible)
        cloud.point_step = packet.size
        cloud.row_step = len(data)
        cloud.data = bytes(data)
        cloud.is_dense = False
        self.spans_pub.publish(cloud)
        return dict(spans=len(visible),multilevel_columns=sum(len(v)>1 for v in self.terrain.columns.values()),
                    selected_yaw_deg=selected*360/self.config.yaw_bins,
                    safe_all_yaw=counts[0],restricted_yaw=counts[1],rejected=counts[2],
                    selected_yaw_geometry_pass=sum(bool(int(geom[i]) & (1<<selected)) for i in visible),
                    observed_pass=sum(bool(int(verified[i]) & (1<<selected)) for i in visible),
                    unknown_clearance=sum(bool(int(geom[i]) & (1<<selected)) and
                                          not bool(int(verified[i]) & (1<<selected)) for i in visible),
                    partial_support=counts[3],stale_or_updating=counts[4],
                    any_yaw_observed_pass=sum(bool(verified[i]) for i in visible),
                    any_yaw_geometry_pass=sum(bool(geom[i]) for i in visible))


def main():
    # Small native batches release the GIL frequently. Bound the competing
    # Python receiver's timeslice so each return does not incur a 5 ms delay.
    sys.setswitchinterval(.001)
    rclpy.init()
    node = TerrainNode()
    try:
        rclpy.spin(node)
    finally:
        node.tf_executor.shutdown()
        node.tf_thread.join(timeout=2.)
        node.tf_node.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
