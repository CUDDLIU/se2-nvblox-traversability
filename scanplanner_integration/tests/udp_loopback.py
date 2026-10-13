#!/usr/bin/env python3
"""Exercise the actual 30 Hz UDP process against a localhost M20 emulator."""
import argparse,json,os,signal,socket,subprocess,sys,time
from pathlib import Path
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.qos import qos_profile_sensor_data
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core import packet,decode_packet

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--enter-navigation',action='store_true');ap.add_argument('--normal-speed',action='store_true');args=ap.parse_args();mode=0 if args.enter_navigation else 1;mode_requests=0
    vx,vy,wz=(1.2,0.,.8) if args.normal_speed else (.1,0.,.2)
    root=Path(__file__).resolve().parents[1]
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.bind(('127.0.0.1',0));sock.setblocking(False)
    rclpy.init();node=rclpy.create_node('m20_udp_loopback_test');pub=node.create_publisher(TwistStamped,'/se2_scan/cmd_vel_stamped',qos_profile_sensor_data)
    proc=subprocess.Popen([sys.executable,str(root/'m20_udp.py'),'--loopback-test','--host','127.0.0.1','--port',str(sock.getsockname()[1]),*(['--enter-navigation'] if args.enter_navigation else [])],start_new_session=True)
    peer=None;seq=0;received=[];began=time.monotonic();last_status=0;stopped_at=None
    try:
        while time.monotonic()-began<5:
            now=time.monotonic();rclpy.spin_once(node,timeout_sec=.002)
            while True:
                try:data,peer=sock.recvfrom(65536)
                except BlockingIOError:break
                _,doc=decode_packet(data)
                if (doc['Type'],doc['Command'])==(2,25):received.append((time.monotonic(),doc['Items']))
                if (doc['Type'],doc['Command'])==(1101,5):
                    assert doc['Items']==dict(Mode=1);mode=1;mode_requests+=1
            if peer and now-last_status>.08:
                basic=dict(ControlUsageMode=mode,HES=0,MotionState=17,Direction=0,Sleep=0)
                sock.sendto(packet(1002,6,dict(BasicStatus=basic),seq),peer);seq+=1
                sock.sendto(packet(1002,4,dict(MotionStatus=dict(Height=.57)),seq),peer);seq+=1;last_status=now
            if now-began<3:
                m=TwistStamped();m.header.stamp=node.get_clock().now().to_msg();m.header.frame_id='base_link_dog';m.twist.linear.x=vx;m.twist.linear.y=vy;m.twist.angular.z=wz;pub.publish(m)
            elif stopped_at is None:stopped_at=now
            else:
                # Delayed old commands must never refresh the watchdog.
                m=TwistStamped();m.header.stamp=node.get_clock().now().to_msg();m.header.stamp.sec-=2
                m.header.frame_id='base_link_dog';m.twist.linear.x=.1;pub.publish(m)
        assert any(abs(v['X']-vx)<1e-8 and abs(v['Y']-vy)<1e-8 and abs(v['Yaw']-wz)<1e-8 for _,v in received)
        tail=[v for t,v in received if t>stopped_at+.2]
        assert tail and all(v['X']==0 and v['Y']==0 and v['Yaw']==0 for v in tail)
        active=[t for t,v in received if t>began+1 and t<began+3]
        hz=(len(active)-1)/(active[-1]-active[0]);assert hz>=20,hz
        assert mode_requests==(1 if args.enter_navigation else 0)
        os.killpg(proc.pid,signal.SIGINT);proc.wait(timeout=5);shutdown=[]
        while True:
            try:data,_=sock.recvfrom(65536);shutdown.append(decode_packet(data)[1])
            except BlockingIOError:break
        if args.enter_navigation:
            assert shutdown[-1]['Type']==1101 and shutdown[-1]['Command']==5 and shutdown[-1]['Items']==dict(Mode=0)
            assert all(d['Items']['X']==0 and d['Items']['Yaw']==0 for d in shutdown[-4:-1])
        print(json.dumps(dict(state='passed',velocity_hz=hz,packets=len(received),tested_velocity=[vx,vy,wz],mode_requests=mode_requests,mode_restoration_tested=args.enter_navigation,watchdog='zero within 200 ms',destination='127.0.0.1')))
    finally:
        if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT);proc.wait(timeout=5)
        sock.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
