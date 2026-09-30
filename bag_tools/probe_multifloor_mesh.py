"""Read-only replay-domain probe; counts real vertices, excluding deletion blocks."""
import json
from pathlib import Path
import signal
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry
from nvblox_msgs.msg import Mesh

sys.path.insert(0, str(Path(__file__).resolve().parent.parent/'scripts'))
from mesh_wire import Reader


def main():
    output=Path(sys.argv[1])
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('multifloor_mesh_probe')
    state=dict(odom_z=None, max_odom_z=None, streams={}, errors=[])
    running=True
    def stop(*_):
        nonlocal running
        running=False
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    def odom(msg):
        z=msg.pose.pose.position.z
        state['odom_z']=z
        state['max_odom_z']=max(state['max_odom_z'] or z,z)
    def mesh(key,raw):
        d=state['streams'].setdefault(key,dict(messages=0,geometry_messages=0,upper_geometry_messages=0,
            upstairs_geometry_messages=0,max_vertex_z=None,bins={}))
        try:
            r=Reader(raw);sec=r.scalar('i');nano=r.scalar('I')
            r.take(r.scalar('I'),1);r.scalar('f');r.array('i4',3,skip=True)
            blocks=r.scalar('I');max_z=None;nv=0
            for _ in range(blocks):
                n=r.scalar('I');a=np.frombuffer(r.take(n*12),dtype=r.endian+'f4').reshape(-1,3)
                if n:
                    z=float(np.max(a[:,2]));max_z=z if max_z is None else max(max_z,z);nv+=n
                r.array('f4',3,skip=True);r.array('f4',4,skip=True);r.array('i4',skip=True)
            d['messages']+=1
            d['geometry_messages']+=nv>0
            d['upper_geometry_messages']+=max_z is not None and max_z>3.
            d['upstairs_geometry_messages']+=max_z is not None and max_z>5. and (state['odom_z'] or 0)>5.
            if max_z is not None:d['max_vertex_z']=max(max_z,d['max_vertex_z'] or max_z)
            d['last_stamp']=sec+nano/1e9
            if state['odom_z'] is not None:
                b=d['bins'].setdefault(str(int(state['odom_z'])),dict(messages=0,geometry_messages=0,max_vertex_z=None))
                b['messages']+=1;b['geometry_messages']+=nv>0
                if max_z is not None:b['max_vertex_z']=max(max_z,b['max_vertex_z'] or max_z)
        except Exception as error:state['errors'].append(str(error))
    subscriptions=[node.create_subscription(Odometry,'/nvblox_lio/odom_lidar',odom,qos_profile_sensor_data)]
    for topic in ('/nvblox_node/mesh','/nvblox/height_mesh'):
        subscriptions.append(node.create_subscription(Mesh,topic,lambda raw,k=topic:mesh(k,raw),qos_profile_sensor_data,raw=True))
    previous=0.
    try:
        while running:
            rclpy.spin_once(node,timeout_sec=.1)
            now=time.monotonic()
            if now-previous>1:
                tmp=output.with_suffix('.tmp');tmp.write_text(json.dumps(state,indent=2));tmp.replace(output);previous=now
    finally:
        output.write_text(json.dumps(state,indent=2));node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
