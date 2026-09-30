#!/usr/bin/env python3
"""Static bag map, 3D RViz goal picking, and checked global paths. No control."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import colorsys
from datetime import datetime
import json
import math
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))
from planner import Planner, NoPath
from geometry import read_mesh
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Point, PointStamped, PoseStamped, TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from nav_msgs.msg import Path as RosPath
from std_msgs.msg import ColorRGBA, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


def point(xyz):
    return Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))


class GlobalPlannerNode(Node):
    def __init__(self, directory):
        super().__init__('se2_global_planner')
        self.directory = Path(directory)
        self.planner = Planner(directory)
        self.frame = self.planner.document['frame']
        self.static_tf = StaticTransformBroadcaster(self)
        origin = TransformStamped()
        origin.header.frame_id = self.frame
        origin.header.stamp = self.get_clock().now().to_msg()
        origin.child_frame_id = 'se2_global_origin'
        origin.transform.rotation.w = 1.
        self.static_tf.sendTransform(origin)
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.future, self.pending = None, None
        self.generation = 0
        self.pick_start = False
        self.start = self.planner.default_start()
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)
        self.mesh_pub = self.create_publisher(MarkerArray, '/se2_global/mesh', retained)
        self.field_pub = self.create_publisher(MarkerArray, '/se2_global/traversability', retained)
        self.poly_pub = self.create_publisher(MarkerArray, '/se2_global/polygons', retained)
        self.marker_pub = self.create_publisher(MarkerArray, '/se2_global/path_markers', retained)
        self.pose_pub = self.create_publisher(MarkerArray, '/se2_global/query_markers', retained)
        self.path_pub = self.create_publisher(RosPath, '/se2_global/path', retained)
        self.status_pub = self.create_publisher(String, '/se2_global/status', retained)
        self.create_subscription(PointStamped, '/clicked_point', self.clicked, 10)
        self.create_subscription(PointStamped, '/se2_global/start', self.start_clicked, 10)
        self.create_subscription(PoseStamped, '/se2_global/goal_pose', self.pose_goal, 10)
        self.create_service(Trigger, '/se2_global/pick_start', self.pick_start_service)
        self.create_service(Trigger, '/se2_global/reset_start', self.reset_start_service)
        self.create_timer(.1, self.poll)
        self.publish_map()
        self.publish_query()
        self.status('ready', '就绪：在 RViz 选择 Publish Point，点击目标楼层的绿色或橙色地面。')

    def marker(self, identifier, kind, ns):
        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = ns, identifier, kind, Marker.ADD
        m.pose.orientation.w = 1.
        m.scale.x = m.scale.y = m.scale.z = 1.
        return m

    def status(self, state, message, **extra):
        doc = dict(state=state, message=message, map=str(self.directory), frame=self.frame,
                   start=self.start.tolist(), generation=self.generation, control_outputs='none', **extra)
        self.status_pub.publish(String(data=json.dumps(doc, ensure_ascii=False)))
        self.get_logger().info(message)
        target = self.directory / 'runtime'
        target.mkdir(exist_ok=True)
        tmp = target / 'status.tmp'
        tmp.write_text(json.dumps(doc, ensure_ascii=False, indent=2))
        tmp.replace(target / 'status.json')

    def publish_map(self):
        blocks, _ = read_mesh(self.directory / self.planner.document['source_mesh'])
        zs = np.concatenate([v[:, 2] for v, _ in blocks.values()])
        zlo, zhi = float(zs.min()), float(zs.max())
        markers = []
        for i, (vertices, triangles) in enumerate(blocks.values()):
            m = self.marker(i, Marker.TRIANGLE_LIST, 'reconstruction')
            m.points = [point(p) for p in vertices[triangles].reshape(-1, 3)]
            rgb = colorsys.hsv_to_rgb(.72*(1-(float(vertices[:, 2].mean())-zlo)/max(.01,zhi-zlo)), .65, .85)
            m.color = ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=.55)
            markers.append(m)
        self.mesh_pub.publish(MarkerArray(markers=markers))
        from angular_display import orange_color
        field = np.load(self.directory / 'field.npz', allow_pickle=False)
        m = self.marker(0, Marker.CUBE_LIST, 'static_traversability')
        m.scale.x = m.scale.y = self.planner.config.resolution*.95
        m.scale.z = .018
        full = (1 << self.planner.bins)-1
        for cell, comfortable, states in zip(field['cells'], field['comfortable'], field['states']):
            xyz = [(cell[0]+.5)*self.planner.config.resolution,
                   (cell[1]+.5)*self.planner.config.resolution,
                   cell[2]*self.planner.config.vertical_resolution+.025]
            m.points.append(point(xyz))
            fraction = min(1., float(np.nansum(np.maximum(0., states[:,5]-states[:,4])))/(2*math.pi))
            rgba = (.12,.86,.32,.95) if int(comfortable) == full else orange_color(fraction)
            m.colors.append(ColorRGBA(r=rgba[0],g=rgba[1],b=rgba[2],a=rgba[3]))
        field.close()
        self.field_pub.publish(MarkerArray(markers=[m]))
        edges = self.marker(0, Marker.LINE_LIST, 'navmesh_edges')
        edges.scale.x = .008
        edges.color = ColorRGBA(r=.3,g=.75,b=1.,a=.6)
        for p in self.planner.polygons:
            vertices = np.asarray(p['vertices'])
            for a,b in zip(vertices,np.roll(vertices,-1,axis=0)):
                edges.points.extend([point(a+[0,0,.045]),point(b+[0,0,.045])])
        self.poly_pub.publish(MarkerArray(markers=[edges]))

    def publish_query(self, goal=None):
        markers = [self.marker(0, Marker.SPHERE, 'start')]
        markers[0].pose.position = point(self.start[:3]+[0,0,.12])
        markers[0].scale.x = markers[0].scale.y = markers[0].scale.z = .22
        markers[0].color = ColorRGBA(r=.1,g=.9,b=1.,a=1.)
        if goal is not None:
            m=self.marker(1,Marker.SPHERE,'goal')
            m.pose.position=point(np.asarray(goal)+[0,0,.12])
            m.scale.x=m.scale.y=m.scale.z=.22
            m.color=ColorRGBA(r=1.,g=.15,b=.55,a=1.)
            markers.append(m)
        else:
            m=self.marker(1,Marker.SPHERE,'goal');m.action=Marker.DELETE;markers.append(m)
        self.pose_pub.publish(MarkerArray(markers=markers))

    def clear_path(self):
        m=self.marker(0,Marker.LINE_STRIP,'route');m.action=Marker.DELETEALL
        self.marker_pub.publish(MarkerArray(markers=[m]))
        path=RosPath();path.header.frame_id=self.frame;path.header.stamp=self.get_clock().now().to_msg()
        self.path_pub.publish(path)

    def valid_frame(self,msg):
        if msg.header.frame_id != self.frame:
            self.generation+=1;self.pending=None;self.clear_path()
            self.status('invalid_query',f'选点坐标系必须为 {self.frame}，收到 {msg.header.frame_id}。')
            return False
        return True

    def clicked(self,msg):
        if self.pick_start:
            self.pick_start=False
            self.start_clicked(msg)
        elif self.valid_frame(msg):
            self.enqueue([msg.point.x,msg.point.y,msg.point.z])

    def start_clicked(self,msg):
        if not self.valid_frame(msg):return
        try:
            xyz,_=self.planner.project([msg.point.x,msg.point.y,msg.point.z])
            self.start[:3]=xyz
            self.generation+=1;self.pending=None
            self.clear_path();self.publish_query()
            self.status('ready','起点已更新，请使用 Publish Point 点击目标。')
        except NoPath as exc:self.status('invalid_start',str(exc))

    def pose_goal(self,msg):
        if not self.valid_frame(msg):return
        p,q=msg.pose.position,msg.pose.orientation
        values=np.asarray([q.x,q.y,q.z,q.w])
        norm=float(np.linalg.norm(values))
        if not np.isfinite(values).all() or norm<1e-8:
            self.generation+=1;self.pending=None;self.clear_path()
            self.status('invalid_query','目标朝向四元数无效。')
            return
        q.x,q.y,q.z,q.w=map(float,values/norm)
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.enqueue([p.x,p.y,p.z],yaw)

    def enqueue(self,goal,yaw=None):
        if not np.isfinite(goal).all():
            self.generation+=1;self.pending=None;self.clear_path()
            self.status('invalid_query','目标坐标必须为有限值。')
            return
        self.generation+=1
        self.pending=(self.generation,self.start.copy(),np.asarray(goal),yaw)
        self.clear_path();self.publish_query(goal)
        self.status('planning','正在规划跨楼层全局路径…',goal=list(map(float,goal)))

    def pick_start_service(self,request,response):
        self.pick_start=True
        response.success=True;response.message='下一次 Publish Point 点击将设置起点。'
        self.status('pick_start',response.message)
        return response

    def reset_start_service(self,request,response):
        self.start=self.planner.default_start();self.generation+=1;self.pending=None
        self.clear_path();self.publish_query()
        response.success=True;response.message='起点已恢复为 bag 录制终点。'
        self.status('ready',response.message)
        return response

    def poll(self):
        if self.future is not None and self.future.done():
            generation=self.future_generation
            try:
                result=self.future.result()
                if generation==self.generation:
                    self.publish_path(result)
            except Exception as exc:
                if generation==self.generation:
                    self.clear_path();self.status('no_path',str(exc))
            self.future=None
        if self.future is None and self.pending is not None:
            generation,start,goal,yaw=self.pending;self.pending=None
            self.future_generation=generation
            self.future=self.pool.submit(self.planner.query,start,goal,yaw,90.)

    def publish_path(self,result):
        poses=np.asarray(result['poses'])
        path=RosPath();path.header.frame_id=self.frame;path.header.stamp=self.get_clock().now().to_msg()
        for row in poses:
            p=PoseStamped();p.header=path.header;p.pose.position=point(row[:3])
            p.pose.orientation.z=math.sin(row[3]/2);p.pose.orientation.w=math.cos(row[3]/2)
            path.poses.append(p)
        self.path_pub.publish(path)
        line=self.marker(0,Marker.LINE_STRIP,'route');line.scale.x=.065
        line.color=ColorRGBA(r=1.,g=.95,b=.05,a=1.)
        line.points=[point(p[:3]+[0,0,.08]) for p in poses]
        markers=[line];last=None
        for i,p in enumerate(poses):
            if last is not None and np.linalg.norm(p[:3]-last[:3])<.45 and abs(p[3]-last[3])<.35:continue
            m=self.marker(i+1,Marker.ARROW,'heading');m.pose.position=point(p[:3]+[0,0,.12])
            m.pose.orientation.z=math.sin(p[3]/2);m.pose.orientation.w=math.cos(p[3]/2)
            m.scale.x=.30;m.scale.y=.055;m.scale.z=.06
            m.color=ColorRGBA(r=1.,g=.7,b=.0,a=1.)
            markers.append(m);last=p
        self.marker_pub.publish(MarkerArray(markers=markers))
        folder=self.directory/'queries';folder.mkdir(exist_ok=True)
        (folder/(datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.json')).write_text(json.dumps(result,indent=2))
        self.status('success',f"路径 {result['length_m']:.1f} m，高度范围 {result['height_range_m'][0]:.2f}–{result['height_range_m'][1]:.2f} m，规划 {result['seconds']:.2f} s。",
                    result={k:v for k,v in result.items() if k!='poses'})


def main():
    parser=argparse.ArgumentParser();parser.add_argument('map',type=Path);args=parser.parse_args()
    rclpy.init();node=GlobalPlannerNode(args.map)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.pool.shutdown(wait=True,cancel_futures=True)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
