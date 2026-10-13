#!/usr/bin/env python3
"""M20 true-velocity sender. Usage-mode change requires --enter-navigation."""
import argparse
import json
import signal
import socket
import time
from pathlib import Path
from core import packet,velocity_packet,decode_packet


class Gate:
    def __init__(self):
        self.command=(0.,0.,0.);self.command_at=-1e9
        self.basic={};self.basic_at=-1e9;self.motion_at=-1e9
        self.failed=False;self.last_error=None
    def receive(self,doc,now):
        key=(doc.get('Type'),doc.get('Command'));items=doc.get('Items',{})
        if key==(1002,6) and isinstance(items.get('BasicStatus'),dict) and items['BasicStatus']:
            self.basic=items['BasicStatus'];self.basic_at=now
        if key==(1002,4):
            try:
                if .2<float(items['MotionStatus']['Height'])<1.:self.motion_at=now
            except (KeyError,TypeError,ValueError):pass
        error=doc.get('ErrorCode',items.get('ErrorCode',0))
        if error not in (None,0,'0'):
            self.failed=True;self.last_error=dict(type=key[0],command=key[1],code=error,message=items.get('ErrorMessage'))
    def stop_reason(self,now):
        b=self.basic
        if self.failed:return 'error_latched'
        if now-self.command_at>=.12:return 'stale_command'
        if now-self.basic_at>=1.2:return 'stale_basic_status'
        if now-self.motion_at>=.35:return 'stale_motion_status'
        if b.get('ControlUsageMode')!=1:return 'not_navigation_mode'
        if b.get('HES')!=0:return 'emergency_stop'
        if b.get('Direction')!=0:return 'reversed_body_direction'
        if b.get('MotionState') not in (1,17):return 'motion_state_not_ready'
        if b.get('Sleep')!=0:return 'not_awake'
        return None
    def output(self,now):
        return self.command if self.stop_reason(now) is None else (0.,0.,0.)


def main():
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    ap=argparse.ArgumentParser();ap.add_argument('--host',default='10.21.31.103');ap.add_argument('--port',type=int,default=30000)
    ap.add_argument('--execute',action='store_true');ap.add_argument('--telemetry-only',action='store_true')
    ap.add_argument('--enter-navigation',action='store_true',help='Request navigation usage mode once, then respect operator mode changes')
    ap.add_argument('--output',type=Path);ap.add_argument('--loopback-test',action='store_true');args=ap.parse_args()
    if args.execute and args.telemetry_only:ap.error('Choose execution or telemetry-only mode')
    if args.enter_navigation and (args.telemetry_only or not (args.execute or args.loopback_test and args.host=='127.0.0.1')):ap.error('--enter-navigation requires execution or a localhost test')
    if not args.execute and not args.telemetry_only and not (args.loopback_test and args.host=='127.0.0.1'):
        ap.error('Specify --execute for hardware or --loopback-test --host 127.0.0.1 for tests')
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.bind(('0.0.0.0',0));sock.setblocking(False)
    target=(socket.gethostbyname(args.host),args.port);gate=Gate();rclpy.init();node=rclpy.create_node('se2_scan_m20')
    status=node.create_publisher(String,'/se2_scan/m20_status',1)
    raw=node.create_publisher(String,'/nvblox_robot/motion_status/raw',10)
    if args.output:args.output.mkdir(parents=True,exist_ok=True)
    command_rx=dict(received=0,accepted=0,rejected_stale=0,rejected_future=0,
                    rejected_frame=0,rejected_value=0,last_age_s=None,last_rejection=None)
    def receive_command(m):
        command_rx['received']+=1
        try:
            age=node.get_clock().now().nanoseconds/1e9-(m.header.stamp.sec+m.header.stamp.nanosec/1e9)
            command_rx['last_age_s']=age
            reason=('frame' if m.header.frame_id!='base_link_dog' else
                    'future' if age<0 else 'stale' if age>=.12 else None)
            if reason:
                command_rx['rejected_'+reason]+=1;command_rx['last_rejection']=reason;return
            v=m.twist;velocity_packet(v.linear.x,v.linear.y,v.angular.z)
            gate.command=(v.linear.x,v.linear.y,v.angular.z);gate.command_at=time.monotonic()-age
            command_rx['accepted']+=1
        except ValueError:
            gate.failed=True;command_rx['rejected_value']+=1;command_rx['last_rejection']='value'
    # Consume the newest command, never work through a backlog at the same rate
    # as its publisher. A five-frame SensorDataQoS queue exceeds the 120 ms
    # watchdog at 30 Hz and can remain stale after a single scheduler stall.
    latest=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(TwistStamped,'/se2_scan/cmd_vel_stamped',receive_command,latest)
    sequence=0;last_heartbeat=-1e9;last_report=-1e9;deadline=time.monotonic();running=[True]
    mode_confirmed=False;mode_attempts=0;mode_requested_at=-1e9;previous_mode=None
    velocity_packets=0;nonzero_packets=0;last_send=None;max_send_period_ms=0.
    receive_stats={};received_at={}
    signal.signal(signal.SIGINT,lambda *_:running.__setitem__(0,False));signal.signal(signal.SIGTERM,lambda *_:running.__setitem__(0,False))
    def send(data):sock.sendto(data,target)
    try:
        while running[0]:
            rclpy.spin_once(node,timeout_sec=0)
            now=time.monotonic()
            # Bound receive work so a telemetry burst cannot starve 30 Hz sends.
            for _ in range(32):
                try:data,peer=sock.recvfrom(65536)
                except BlockingIOError:break
                if peer!=target:continue
                try:
                    message_id,doc=decode_packet(data);gate.receive(doc,now)
                    key=f'{doc.get("Type")}/{doc.get("Command")}'
                    stats=receive_stats.setdefault(key,dict(packets=0,max_gap_s=0.))
                    stats['packets']+=1
                    if key in received_at:stats['max_gap_s']=max(stats['max_gap_s'],now-received_at[key])
                    received_at[key]=now
                    if (doc.get('Type'),doc.get('Command'))==(1002,4):
                        raw.publish(String(data=json.dumps(dict(receive_time_ns=time.time_ns(),message_id=message_id,PatrolDevice=doc))))
                except (ValueError,TypeError):continue
            if now-last_heartbeat>=.5:
                send(packet(100,100,{},sequence));sequence+=1;last_heartbeat=now
            if gate.basic.get('ControlUsageMode')==1:mode_confirmed=True
            if args.enter_navigation and not mode_confirmed and not gate.failed and now-mode_requested_at>1.:
                b=gate.basic
                prepared=(now-gate.basic_at<1.2 and now-gate.motion_at<.35 and b.get('HES')==0 and
                          b.get('Direction')==0 and b.get('MotionState') in (1,17) and b.get('Sleep')==0)
                if prepared:
                    if mode_attempts>=3:gate.failed=True
                    else:
                        if previous_mode is None:previous_mode=b.get('ControlUsageMode')
                        send(packet(1101,5,dict(Mode=1),sequence));sequence+=1;mode_attempts+=1;mode_requested_at=now
            command=gate.output(now)
            if not args.telemetry_only:
                send(velocity_packet(*command,sequence));sequence+=1;velocity_packets+=1
                if any(command):nonzero_packets+=1
                sent_at=time.monotonic()
                if last_send is not None:max_send_period_ms=max(max_send_period_ms,(sent_at-last_send)*1000)
                last_send=sent_at
            if now-last_report>=1:
                report=dict(command=command,command_age=now-gate.command_at,basic_age=now-gate.basic_at,
                    motion_age=now-gate.motion_at,failed=gate.failed,basic=gate.basic,telemetry_only=args.telemetry_only,
                    navigation_mode_requested=args.enter_navigation,mode_attempts=mode_attempts,mode_confirmed=mode_confirmed,
                    stop_reason='telemetry_only' if args.telemetry_only else gate.stop_reason(now),
                    command_receive=command_rx,velocity_packets=velocity_packets,nonzero_packets=nonzero_packets,
                    max_send_period_ms=max_send_period_ms,
                    last_error=gate.last_error,telemetry_receive=receive_stats,udp_local_port=sock.getsockname()[1])
                status.publish(String(data=json.dumps(report)))
                if args.output:
                    tmp=args.output/'status.tmp';tmp.write_text(json.dumps(report,indent=2));tmp.replace(args.output/'status.json')
                    with (args.output/'health.jsonl').open('a') as log:
                        log.write(json.dumps(dict(time_ns=time.time_ns(),**report))+'\n')
                last_report=now
            deadline+=1/30
            time.sleep(max(0.,deadline-time.monotonic()))
            if time.monotonic()-deadline>.1:deadline=time.monotonic()
    finally:
        for _ in range(0 if args.telemetry_only else 3):
            try:send(velocity_packet(0.,0.,0.,sequence));sequence+=1
            except OSError:pass
        # Restore only a mode this process changed, after stop packets. A later
        # operator mode change is never overwritten. UDP delivery is best effort.
        if mode_attempts and previous_mode in (0,2) and gate.basic.get('ControlUsageMode')==1:
            try:send(packet(1101,5,dict(Mode=previous_mode),sequence));sequence+=1
            except OSError:pass
        sock.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
