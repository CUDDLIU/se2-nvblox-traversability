"""ROS message-level rendering tests; no DDS, camera, or robot required."""
import math
import unittest
from types import SimpleNamespace
import numpy as np
from rclpy.time import Time
from visualization_msgs.msg import Marker
from terrain import Config,Surface
from ros_node import TerrainNode


class DisplayTests(unittest.TestCase):
    def test_stable_marker_ids_no_expiry_and_empty_replacement(self):
        markers=[]
        fake=SimpleNamespace(frame='nvblox_odom',config=Config(),
             terrain=SimpleNamespace(columns={}),
             get_clock=lambda:SimpleNamespace(now=lambda:Time()),
             markers_pub=SimpleNamespace(publish=markers.append),
             spans_pub=SimpleNamespace(publish=lambda msg:None))
        full=(1<<fake.config.yaw_bins)-1
        TerrainNode.publish(fake,[Surface(200,0,0,math.inf,True)],np.array([0.]),
            np.array([full],dtype=np.uint64),np.array([0],dtype=np.uint64),[0],0,[],
            np.array([0],dtype=np.uint64))
        first=markers[-1].markers
        self.assertEqual(len(first[0].points),1)
        TerrainNode.publish(fake,[],np.array([]),np.array([],dtype=np.uint64),
            np.array([],dtype=np.uint64),[],0,[],np.array([],dtype=np.uint64))
        second=markers[-1].markers
        self.assertEqual([(m.ns,m.id) for m in first],[(m.ns,m.id) for m in second])
        self.assertTrue(all(m.action==Marker.ADD for m in first+second))
        self.assertTrue(all(m.lifetime.sec==0 and m.lifetime.nanosec==0 for m in first+second))
        self.assertTrue(all(len(m.points)==0 for m in second))


if __name__=='__main__':
    unittest.main()
