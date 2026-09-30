"""Height colors on serialized Mesh arrays, without ROS Point/Color objects."""
import colorsys
import math
import numpy as np
import rclpy
from rclpy.node import Node
from nvblox_msgs.msg import Mesh,MeshBlock,Index3D
from mesh_wire import recolor_mesh,check_schema


class HeightMesh(Node):
    def __init__(self):
        super().__init__('nvblox_height_mesh')
        check_schema(Mesh,MeshBlock,Index3D)
        self.declare_parameter('cyclic_height_colors',True)
        self.declare_parameter('height_cycle_m',6.)
        self.declare_parameter('height_origin_m',0.)
        self.declare_parameter('contour_spacing_m',.25)
        self.cyclic=bool(self.get_parameter('cyclic_height_colors').value)
        self.contour=float(self.get_parameter('contour_spacing_m').value)
        if not math.isfinite(self.contour) or self.contour<0:raise ValueError('Invalid contour spacing')
        self.declare_parameter('min_height_m',-.7)
        self.declare_parameter('max_height_m',1.8)
        self.low=float(self.get_parameter('min_height_m').value)
        self.high=float(self.get_parameter('max_height_m').value)
        if self.cyclic:
            self.low=float(self.get_parameter('height_origin_m').value)
            self.high=self.low+float(self.get_parameter('height_cycle_m').value)
        if not math.isfinite(self.low) or not math.isfinite(self.high) or self.high<=self.low:raise ValueError('Invalid color height range')
        self.palette=np.array([(*colorsys.hsv_to_rgb((.66-i/1024)%1.,.82,.98),1.)
                               for i in range(1024)],dtype=np.float32) if self.cyclic else np.array(
            [(*colorsys.hsv_to_rgb(.78-.62*i/255,.9,.95),1.) for i in range(256)],dtype=np.float32)
        self.get_logger().info(f'Height colors: cyclic={self.cyclic}, period/range={self.high-self.low:.2f} m, contours={self.contour:.2f} m')
        self.pub=self.create_publisher(Mesh,'/nvblox/height_mesh',5)
        self.create_subscription(Mesh,'/nvblox_node/mesh',self.callback,100,raw=True)

    def callback(self,raw):
        self.pub.publish(recolor_mesh(raw,self.low,self.high,self.palette,cyclic=self.cyclic,contour_spacing=self.contour))


def main():
    rclpy.init();node=HeightMesh()
    try:rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
