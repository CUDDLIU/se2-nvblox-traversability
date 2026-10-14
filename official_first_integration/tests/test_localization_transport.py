import importlib.util,sys,time,unittest,tempfile
from pathlib import Path
from types import SimpleNamespace as N
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'official_first_integration/source'),str(ROOT/'scanplanner_integration')]
from localization_node import Localization
from alignment import MapAligner
from runtime import cloud_message
from geometry_msgs.msg import PoseStamped
spec=importlib.util.spec_from_file_location('latency_transport',ROOT/'official_first_integration/native/adapter.py');transport=importlib.util.module_from_spec(spec);spec.loader.exec_module(transport)
class LocalizationTransport(unittest.TestCase):
 def test_lost_tracker_invalidates_route_and_reacquires_near_propagated_pose(self):
  class Failed:
   def done(self):return True
   def result(self):raise ValueError('map correction jumped')
  with tempfile.TemporaryDirectory() as d:
   cache=Path(d)/'cache';cache.write_text('old');sent=[];states=[];r=Rotation.from_euler('z',.2).as_matrix();p=np.array([1.,-20.,3.])
   fake=N(get_publishers_info_by_topic=lambda _: [N(endpoint_gid=b'\x01')],publisher_key='01',cache_loaded=True,
    future=Failed(),good_at=time.monotonic()-2,stable=2,transform=(np.array([.1,0,0]),r),poses=deque([np.r_[time.time(),p,0,0,0,1]]),
    aligner=N(),generation=0,relocalizations=0,cache=cache,pub=N(publish=sent.append),clouds=[],state=lambda *a,**k:states.append((a,k)))
   Localization.tick(fake)
   self.assertIsNone(fake.transform);self.assertEqual(fake.good_at,0);self.assertEqual(fake.relocalizations,1)
   np.testing.assert_allclose(fake.aligner.anchor,r@p+[.1,0,0]);self.assertFalse(cache.exists());self.assertEqual(len(sent),1)
 def test_adapter_checks_robot_correction_and_invalidates_failed_localization(self):
  body=np.array([0.,-20.,3.]);cleared=[]
  fake=N(raw_previous=(time.time(),body),translation=np.zeros(3),rotation=Rotation.identity(),last_tf=True,tf_at=time.time(),route=object(),clear_route=cleared.append)
  r=Rotation.from_euler('x',.02);t=body-r.apply(body)
  m=PoseStamped();m.header.frame_id='se2_scan_map';ns=time.time_ns();m.header.stamp.sec,m.header.stamp.nanosec=divmod(ns,10**9)
  m.pose.position.x,m.pose.position.y,m.pose.position.z=map(float,t);q=r.as_quat();m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w=map(float,q)
  self.assertGreater(np.linalg.norm(t),.15);transport.Adapter.transform(fake,m);self.assertFalse(cleared);self.assertGreater(fake.tf_at,0)
  m.pose.position.x+=.3;transport.Adapter.transform(fake,m);self.assertEqual(cleared,['localization_jump'])
  transport.Adapter.transform(fake,PoseStamped());self.assertEqual(cleared[-1],'localization_invalid');self.assertEqual(fake.tf_at,0)
if __name__=='__main__':unittest.main()
