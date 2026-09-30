"""One bounded diagnostic run of the real node, no robot commands."""
import cProfile
import pstats
import time
import rclpy
from ros_node import TerrainNode

rclpy.init()
node=TerrainNode()
original=node.terrain.rebuild
profile=cProfile.Profile()
done=False
def rebuild():
    global done
    if node.terrain.dirty and not done:
        profile.runcall(original)
        pstats.Stats(profile).strip_dirs().sort_stats('cumulative').print_stats(25)
        done=True
    else:
        original()
node.terrain.rebuild=rebuild
started=time.monotonic()
try:
    while not done and time.monotonic()-started<25:
        rclpy.spin_once(node,timeout_sec=.1)
finally:
    node.tf_executor.shutdown()
    node.tf_thread.join(timeout=2.)
    node.tf_node.destroy_node()
    node.destroy_node()
    rclpy.shutdown()
