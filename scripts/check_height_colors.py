"""Check unclipped multi-level colors while preserving mesh geometry and deletion."""
import colorsys,json,sys
import numpy as np
from geometry_msgs.msg import Point32
from nvblox_msgs.msg import Mesh,MeshBlock,Index3D
from rclpy.serialization import serialize_message,deserialize_message
from mesh_wire import recolor_mesh
palette=np.array([(*colorsys.hsv_to_rgb((.66-i/1024)%1.,.82,.98),1.) for i in range(1024)],dtype=np.float32)
z=[-30.13,-7.13,-6.13,-3.13,-1.13,0.13,1.13,2.13,3.13,6.13,7.13,30.13]
m=Mesh();m.header.frame_id='nvblox_odom';m.header.stamp.sec=42;m.block_size_m=.56
m.block_indices=[Index3D(x=0,y=0,z=0),Index3D(x=1,y=0,z=0)]
b=MeshBlock();b.vertices=[Point32(x=float(i),y=0.,z=float(h)) for i,h in enumerate(z)];b.triangles=list(range(12))
m.blocks=[b,MeshBlock()];m.clear=True
raw=serialize_message(m);m=deserialize_message(raw,Mesh);b=m.blocks[0]
out=deserialize_message(recolor_mesh(raw,0,6,palette,cyclic=True,contour_spacing=.25),Mesh)
assert out.header==m.header and out.block_indices==m.block_indices and out.clear==m.clear
assert out.blocks[0].vertices==b.vertices and out.blocks[0].triangles==b.triangles
assert out.blocks[1]==m.blocks[1]
c=np.array([[v.r,v.g,v.b,v.a] for v in out.blocks[0].colors])
assert np.isfinite(c).all() and (c>=0).all() and (c<=1).all()
assert np.linalg.norm(c[1,:3]-c[2,:3])>.2 # both below old minimum
assert np.linalg.norm(c[7,:3]-c[8,:3])>.2 # both above old maximum
assert np.allclose(c[6],c[10],atol=1e-5) # same world height modulo 6 m
print(json.dumps(dict(result='PASS',heights=z,rgba=c.tolist(),geometry_unchanged=True,empty_deletion_preserved=True,clear_preserved=True),indent=2))
