#!/usr/bin/env python3
"""A live telemetry session must never send motion, even when commands exist."""
import json,os,signal,socket,subprocess,sys,time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core import packet,decode_packet

def main():
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.bind(('127.0.0.1',0));sock.setblocking(False)
    rclpy.init();node=rclpy.create_node('scan_telemetry_test');received=[];raw=[]
    pub=node.create_publisher(TwistStamped,'/se2_scan/cmd_vel_stamped',qos_profile_sensor_data)
    node.create_subscription(String,'/nvblox_robot/motion_status/raw',lambda m:raw.append(json.loads(m.data)),10)
    process=subprocess.Popen([sys.executable,str(Path(__file__).resolve().parents[1]/'m20_udp.py'),'--telemetry-only','--host','127.0.0.1','--port',str(sock.getsockname()[1])],start_new_session=True)
    peer=None;last=0.;began=time.monotonic()
    try:
        while time.monotonic()-began<4:
            rclpy.spin_once(node,timeout_sec=.003);now=time.monotonic()
            while True:
                try:data,peer=sock.recvfrom(65536)
                except BlockingIOError:break
                received.append(decode_packet(data)[1])
            if peer and now-last>.08:
                sock.sendto(packet(1002,6,dict(BasicStatus=dict(ControlUsageMode=1,HES=0,MotionState=17,Direction=0,Sleep=0))),peer)
                sock.sendto(packet(1002,4,dict(MotionStatus=dict(Height=.57))),peer);last=now
            m=TwistStamped();m.header.frame_id='base_link_dog';m.header.stamp=node.get_clock().now().to_msg();m.twist.linear.x=.1;pub.publish(m)
        os.killpg(process.pid,signal.SIGINT);process.wait(timeout=5)
        while True:
            try:data,peer=sock.recvfrom(65536);received.append(decode_packet(data)[1])
            except BlockingIOError:break
        assert received and raw
        assert all((d['Type'],d['Command'])==(100,100) for d in received),received
        assert raw[-1]['PatrolDevice']['Items']['MotionStatus']['Height']==.57
        print(json.dumps(dict(state='passed',telemetry_frames=len(raw),outbound_heartbeats=len(received),motion_packets=0)))
    finally:
        if process.poll() is None:os.killpg(process.pid,signal.SIGINT);process.wait(timeout=5)
        sock.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
