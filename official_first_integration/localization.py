#!/usr/bin/env python3
"""FIRST localization with a persisted, validated-pose search hint."""
import argparse
import json
import math
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]

def load_pose_hint(path,mesh_sha256,now=None):
    try:
        d=json.loads(Path(path).read_text());pose=d['pose'];now=time.time() if now is None else now
        if d['mesh_sha256']!=mesh_sha256 or not 0<=now-d['time_s']<3600:return None
        if len(pose)!=4 or not all(math.isfinite(float(v)) for v in pose):return None
        return [float(v) for v in pose]
    except (OSError,ValueError,TypeError,KeyError):return None

def main():
    sys.path.insert(0,str(ROOT/'scanplanner_integration'))
    from localization_node import Localization
    from reference_alignment import ReferenceAligner
    import rclpy
    from scipy.spatial.transform import Rotation
    ap=argparse.ArgumentParser();ap.add_argument('map');ap.add_argument('--output',required=True)
    ap.add_argument('--pose-cache',type=Path,required=True);ap.add_argument('--initial-map-pose',type=float,nargs=4)
    args=ap.parse_args()
    class RememberedLocalization(Localization):
        def __init__(self):
            self.last_write=0.
            super().__init__(args.map,args.output,args.initial_map_pose)
            old=self.aligner;self.aligner=ReferenceAligner(args.map)
            self.aligner.anchor=old.anchor
            if hasattr(old,'seed_yaw'):self.aligner.seed_yaw=old.seed_yaw
            self.get_logger().info('Localization reference: '+self.aligner.reference_kind)
        def odom(self,msg):
            super().odom(msg)
            now=time.monotonic();stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            age=self.get_clock().now().nanoseconds/1e9-stamp
            if self.transform is None or not self.good_at or now-self.good_at>1.5 or now-self.last_write<.5 or not 0<=age<.2:return
            if msg.header.frame_id!='nvblox_odom' or msg.child_frame_id!='base_link_dog' or not self.poses:return
            if abs(self.poses[-1][0]-stamp)>1e-6:return
            t,r=self.transform;raw=self.poses[-1];position=r@raw[1:4]+t
            rotation=r@Rotation.from_quat(raw[4:]).as_matrix()
            pose=[*map(float,position),math.atan2(rotation[1,0],rotation[0,0])]
            record=dict(mesh_sha256=self.aligner.mesh_sha256,time_s=time.time(),pose=pose,source='fresh_matched_body_pose')
            tmp=args.pose_cache.with_suffix('.tmp');tmp.write_text(json.dumps(record));tmp.replace(args.pose_cache)
            self.last_write=now
    rclpy.init();node=RememberedLocalization()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.pool.shutdown(wait=True,cancel_futures=True);node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
