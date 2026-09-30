#!/usr/bin/env python3
"""Exercise the default bag session: measured start, 3-D goal, pause/resume."""
import json,time
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from nav_msgs.msg import Odometry,Path
from rosbag2_interfaces.srv import IsPaused

def main():
    rclpy.init();node=rclpy.create_node('se2_scan_goal_smoke')
    state=[];body=[];height=[];paths=[];paused=[]
    retained=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String,'/se2_global/status',lambda m:state.append(json.loads(m.data)),retained)
    node.create_subscription(Odometry,'/nvblox_lio/odom',lambda m:body.append((time.monotonic(),m)),qos_profile_sensor_data)
    node.create_subscription(String,'/nvblox_robot/motion_status/raw',lambda m:height.append(time.monotonic()),10)
    node.create_subscription(Path,'/se2_global/path',paths.append,retained)
    pub=node.create_publisher(PointStamped,'/clicked_point',10);client=node.create_client(IsPaused,'/rosbag2_player/is_paused')
    def wait(fn,timeout):
        end=time.monotonic()+timeout
        while time.monotonic()<end and not fn():rclpy.spin_once(node,timeout_sec=.02)
        assert fn(),state[-1:] or 'no status'
    try:
        wait(lambda:state and body and height and time.monotonic()-body[-1][0]<.2 and time.monotonic()-height[-1]<.3 and client.service_is_ready(),65)
        # Bag has a stationary beginning; test a checked goal along that floor.
        goal=PointStamped();goal.header.frame_id='nvblox_odom';goal.point.x=.9;goal.point.y=-.15;goal.point.z=-.59
        # Each DDS reader has its own queue: let the planner receive the same
        # first telemetry frame that this test just saw before submitting a goal.
        settle=time.monotonic()+.5
        while time.monotonic()<settle:rclpy.spin_once(node,timeout_sec=.02)
        initial=state[-1]['generation'];pub.publish(goal)
        end=time.monotonic()+90;last_query=0
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.02)
            if time.monotonic()-last_query>.05:
                future=client.call_async(IsPaused.Request());future.add_done_callback(lambda f:paused.append((time.monotonic(),f.result().paused)) if not f.exception() else None);last_query=time.monotonic()
            if state[-1]['generation']>initial and state[-1]['state']=='invalid_start':
                settle=time.monotonic()+.3
                while time.monotonic()<settle:rclpy.spin_once(node,timeout_sec=.02)
                initial=state[-1]['generation'];pub.publish(goal)
            if state[-1]['generation']>initial and state[-1]['state'] in ('success','no_path'):break
        assert state[-1]['state']=='success',state[-1]
        wait(lambda:paths and paths[-1].poses,3)
        future=client.call_async(IsPaused.Request())
        wait(lambda:future.done(),5)
        if future.result().paused:
            resumed_after=time.monotonic()
            wait(lambda:body[-1][0]>resumed_after,5)
            future=client.call_async(IsPaused.Request());wait(lambda:future.done(),5)
        assert not future.result().paused,'player did not resume'
        paused.append((time.monotonic(),False))
        assert any(value for _,value in paused),'bag was not paused during global planning'
        start=state[-1]['start']
        assert abs(start[3])<.4,'start yaw came from old bag endpoint'
        assert abs(paths[-1].poses[-1].pose.position.z+.59)<.1
        print(json.dumps(dict(state='passed',start=start,path_poses=len(paths[-1].poses),pause_observed=True,resumed=True,global_seconds=state[-1]['result']['seconds']),ensure_ascii=False),flush=True)
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
