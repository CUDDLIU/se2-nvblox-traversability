"""Explicit acceptance of cached startup through the production BagManager."""
import json
import os
from pathlib import Path
import subprocess
import time
from bag_manager import BagManager


def main():
    root=Path(__file__).resolve().parent.parent
    out=root/'bag_tools/validation/20260929_replay_start'
    manager=BagManager();result={};node=None
    try:
        started=time.monotonic()
        manager.play('20260929_125643_0ae23c',rebuild=True,sensor_mode='lidar')
        result['start_to_playing_s']=time.monotonic()-started
        result['session']=str(manager.session)
        result['prepare']=json.loads((manager.session/'prepare_timing.json').read_text())
        os.environ.update(manager.env(True))
        import rclpy
        from rclpy.qos import QoSProfile,ReliabilityPolicy
        from nvblox_msgs.msg import Mesh
        from rosgraph_msgs.msg import Clock
        rclpy.init();node=rclpy.create_node('replay_start_acceptance')
        meshes=[];clocks=[]
        node.create_subscription(Mesh,'/nvblox_node/mesh',lambda msg:meshes.append(time.monotonic()),
                                 QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT))
        node.create_subscription(Clock,'/clock',lambda msg:clocks.append(msg.clock.sec*10**9+msg.clock.nanosec),
                                 QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT))
        deadline=time.monotonic()+20
        while time.monotonic()<deadline and (len(meshes)<5 or len(clocks)<10):
            rclpy.spin_once(node,timeout_sec=.1)
        result.update(mesh_messages=len(meshes),clock_messages=len(clocks),
                      clock_advanced=bool(clocks and clocks[-1]>clocks[0]),mode=manager.tick()['mode'])
        assert result['prepare']['cache_hit']
        assert result['start_to_playing_s']<40
        assert len(meshes)>=5 and result['clock_advanced'] and result['mode']=='play'
        subprocess.run(['gnome-screenshot','-f',str(out/'cached_start_rviz.png')],timeout=10)
        result['result']='PASS';print(json.dumps(result),flush=True)
    except Exception as exc:
        result.update(result='FAIL',error=str(exc));raise
    finally:
        if node is not None:
            node.destroy_node()
            if rclpy.ok():rclpy.shutdown()
        manager.stop_play()
        try:manager.restore_live()
        finally:manager.close()
        (out/'startup_result.json').write_text(json.dumps(result,indent=2))


if __name__=='__main__':main()
