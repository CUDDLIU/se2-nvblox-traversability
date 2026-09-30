"""Height colors on serialized Mesh arrays, without ROS Point/Color objects."""
import colorsys
import numpy as np
import rclpy
from rclpy.node import Node
from nvblox_msgs.msg import Mesh,MeshBlock,Index3D
from mesh_wire import recolor_mesh,check_schema


class HeightMesh(Node):
    def __init__(self):
        super().__init__('nvblox_height_mesh')
        check_schema(Mesh,MeshBlock,Index3D)
        self.declare_parameter('min_height_m',-.7)
        self.declare_parameter('max_height_m',1.8)
        self.low=float(self.get_parameter('min_height_m').value)
        self.high=float(self.get_parameter('max_height_m').value)
        if self.high<=self.low:raise ValueError('Invalid color height range')
        self.palette=np.array([(*colorsys.hsv_to_rgb(.78-.62*i/255,.9,.95),1.)
                               for i in range(256)],dtype=np.float32)
        self.pub=self.create_publisher(Mesh,'/nvblox/height_mesh',5)
        self.create_subscription(Mesh,'/nvblox_node/mesh',self.callback,100,raw=True)

    def callback(self,raw):
        self.pub.publish(recolor_mesh(raw,self.low,self.high,self.palette))


def main():
    rclpy.init();node=HeightMesh()
    try:rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
