#!/usr/bin/env python3
import argparse,json,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from nav_msgs.msg import Path as RosPath,Odometry
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from scan_planner_msgs.msg import Bspline
from sensor_msgs_py.point_cloud2 import read_points_numpy,read_points
ap=argparse.ArgumentParser();ap.add_argument('output',type=Path);args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=True)
rclpy.init();n=rclpy.create_node('se2_live_capture');data={};clouds={}
def pose(p):
 q=p.orientation;return [p.position.x,p.position.y,p.position.z,float(np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))]
def cb(k,m):
 if isinstance(m,Bspline):
  points=np.array([[p.x,p.y,p.z] for p in m.pos_pts])
  if len(points) and np.max(np.linalg.norm(points-points[0],axis=1))>.03:data[k]=dict(points=points.tolist(),knots=list(m.knots),order=m.order)
 elif isinstance(m,RosPath):
  if m.poses:data[k]=[pose(p.pose) for p in m.poses]
 elif isinstance(m,Odometry):
  data[k]=pose(m.pose.pose);q=m.pose.pose.orientation
  data[k+'_orientation']=[q.x,q.y,q.z,q.w]
 elif isinstance(m,PointCloud2):
  fields=('x','y','z','resolution','state') if k=='observed' else ('x','y','z','resolution') if k=='collision' else ('x','y','z')
  if k=='raw':
   fields=('x','y','z','ring') if any(f.name=='ring' for f in m.fields) else fields
   p=read_points(m,field_names=fields,skip_nans=True);clouds[k]=np.column_stack([p[f] for f in fields])
  else:clouds[k]=read_points_numpy(m,field_names=fields,skip_nans=True)
 else:
  try:data[k]=json.loads(m.data)
  except ValueError:data[k]=m.data
retained=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
for k,t,c,q in [('candidate','/se2_scan/planning/bspline',Bspline,1),('reference','/se2_scan/reference',RosPath,retained),('path','/se2_global/path',RosPath,retained),('detour','/se2_scan/planning/detour',RosPath,1),('body','/se2_scan/aligned_odom',Odometry,qos_profile_sensor_data),('motion','/nvblox_robot/motion_status/raw',String,1),('diagnostics','/se2_scan/route_diagnostics',String,1),('guard','/se2_scan/guard/occupancy',PointCloud2,qos_profile_sensor_data),('world','/se2_scan/world_cloud',PointCloud2,qos_profile_sensor_data)]:
 n.create_subscription(c,t,lambda m,k=k:cb(k,m),q)
end=time.monotonic()+5
for k,t in [('observed','/se2_scan/guard/observed_voxels'),('collision','/se2_scan/guard/collision_occupancy'),('raw','/LIDAR/POINTS_NX'),('optimizer_occupancy','/se2_scan/grid_map/occupancy')]:
 n.create_subscription(PointCloud2,t,lambda m,k=k:cb(k,m),qos_profile_sensor_data)
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
(args.output/'snapshot.json').write_text(json.dumps(data));np.savez(args.output/'clouds.npz',**clouds)
print(json.dumps({k:v for k,v in data.items() if k in ('body','reference','detour','diagnostics')}));n.destroy_node();rclpy.shutdown()
