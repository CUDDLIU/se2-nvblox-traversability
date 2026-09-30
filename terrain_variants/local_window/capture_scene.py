"""Bounded diagnostic capture; subscribes only, no robot commands."""
import time
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from nvblox_msgs.msg import Mesh
from tf2_ros import Buffer, TransformListener, TransformException
from ros_node import transform_matrix

rclpy.init()
node = Node('se2_capture_scene')
buffer = Buffer()
listener = TransformListener(buffer,node)
blocks = {}
depths = deque(maxlen=20)
frames = []
info = None
odom = None
last_stamp = -1.

def mesh(msg):
    if msg.clear:
        blocks.clear()
    for index,block in zip(msg.block_indices,msg.blocks):
        key=(index.x,index.y,index.z)
        if block.triangles:
            blocks[key]=block
        else:
            blocks.pop(key,None)

def camera(msg):
    global info
    info=msg

def position(msg):
    global odom
    odom=msg

sensor=QoSProfile(depth=2,reliability=ReliabilityPolicy.BEST_EFFORT)
node.create_subscription(Mesh,'/nvblox_node/mesh',mesh,10)
node.create_subscription(Image,'/camera/d435i/depth/image_rect_raw',depths.append,sensor)
node.create_subscription(CameraInfo,'/camera/d435i/depth/camera_info',camera,sensor)
node.create_subscription(Odometry,'/nvblox_lio/odom',position,sensor)
start=time.monotonic()
while time.monotonic()-start<12:
    rclpy.spin_once(node,timeout_sec=.05)
    if info is None:
        continue
    for msg in reversed(depths):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if stamp-last_stamp<.8:
            break
        try:
            tf=buffer.lookup_transform('nvblox_odom',msg.header.frame_id,Time.from_msg(msg.header.stamp))
        except TransformException:
            continue
        dtype=('<u2' if msg.encoding=='16UC1' else '<f4')
        image=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,
                         strides=(msg.step,np.dtype(dtype).itemsize)).astype(np.float32)
        if msg.encoding=='16UC1':
            image*=.001
        frames.append((image,transform_matrix(tf.transform),stamp))
        last_stamp=stamp
        break
if not blocks or not frames or odom is None:
    raise RuntimeError(f'Capture incomplete: blocks={len(blocks)}, frames={len(frames)}, odom={odom is not None}')
vertices=[]
triangles=[]
keys=[]
nv=nt=0
bounds=[]
for key,block in sorted(blocks.items()):
    v=np.array([(p.x,p.y,p.z) for p in block.vertices])
    t=np.array(block.triangles,dtype=np.int64).reshape(-1,3)
    vertices.extend(v)
    triangles.extend(t+nv)
    keys.append(key)
    bounds.append((nv,nv+len(v),nt,nt+len(t)))
    nv+=len(v);nt+=len(t)
p=odom.pose.pose.position
np.savez_compressed('/tmp/se2_flat_scene.npz',vertices=vertices,triangles=triangles,
                    keys=keys,bounds=bounds,depth=np.stack([f[0] for f in frames]),
                    camera_to_world=np.stack([f[1] for f in frames]),
                    stamps=[f[2] for f in frames],k=np.array(info.k),odom=[p.x,p.y,p.z])
print(f'CAPTURE blocks={len(keys)} vertices={nv} triangles={nt} depth_frames={len(frames)}')
node.destroy_node()
rclpy.shutdown()
