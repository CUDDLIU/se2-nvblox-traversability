#!/usr/bin/env python3
"""M20 true-velocity sender. Usage-mode change requires --enter-navigation."""
import argparse
import json
import math
import signal
import socket
import struct
import time
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scanplanner_integration"))
from core import packet,velocity_packet,decode_packet

# Includes physical posture transition plus the BasicStatus reporting delay.
# Keep in sync with the advance gait request horizon in src/node.cpp.
GAIT_CONFIRM_TIMEOUT=6.


def update_gait_confirmation(state,gate,now):
    if state['pending'] and now-gate.basic_at<1.2 and gate.basic.get('Gait')==state['pending']:
        state['confirmation_seconds']=now-state['requested_at']
        state['pending']=0
    if state['pending'] and now-state['requested_at']>GAIT_CONFIRM_TIMEOUT and not state['failed']:
        state['failed']=True;gate.failed=True
        if gate.last_error is None:
            gate.last_error=dict(source='gait_confirmation_timeout',requested_gait=state['pending'],
                                 actual_gait=gate.basic.get('Gait'),elapsed_seconds=now-state['requested_at'])
    # A late confirmation is telemetry, never authority to release a latched
    # fault and resume an old route without a fresh execution session.


def official_velocity_packet(x,y,yaw,sequence=0):
    # NOS YAML component limits. Preserve the original command, including float32
    # representation at 0.6 m/s, without imposing the legacy 0.4 lateral cap.
    if not all(math.isfinite(v) for v in (x,y,yaw)):
        raise ValueError('non-finite official velocity')
    if abs(x)>1.5001 or abs(y)>.6001 or abs(yaw)>1.0001:
        raise ValueError('velocity outside official navigation limits')
    return packet(2,25,dict(X=x,Y=y,Z=0.,Roll=0.,Pitch=0.,Yaw=yaw),sequence)


def source_velocity_packet(x,y,yaw,sequence=0):
    # Envelope of the captured AOS task profiles. Each task's narrower limits
    # are also checked in the source adapter and compiled planner.
    if not all(math.isfinite(v) for v in (x,y,yaw)) or abs(x)>2.0001 or abs(y)>1.0001 or abs(yaw)>1.0001:
        raise ValueError('velocity outside source task limits')
    return packet(2,25,dict(X=x,Y=y,Z=0.,Roll=0.,Pitch=0.,Yaw=yaw),sequence)


def source_gait_step(transaction,state,gate,now,enabled):
    """At most one gait request; cancelled requests still await physical settling."""
    wanted=state['wanted'];b=gate.basic
    active=enabled and wanted and now-state['wanted_at']<.35
    fresh=now-gate.basic_at<1.2
    ready=(not gate.failed and fresh and now-gate.motion_at<.35
           and b.get('MotionState')==17 and b.get('HES')==0 and b.get('Sleep')==0
           and b.get('Direction')==0 and b.get('ControlUsageMode')==1)
    event=None
    if transaction.state=='waiting':
        # A physical request already sent cannot be undone by clearing the ROS
        # goal. Keep motion stopped until confirmation or a latched timeout;
        # never overlap the old and new gait transactions.
        if fresh and b.get('Gait')==transaction.target:
            transaction.update(transaction.target,now)
        elif now>=transaction.deadline:
            if active and wanted==transaction.target and ready:
                event=transaction.update(b.get('Gait'),now)
            else:transaction.state='timeout'
        if transaction.state in ('waiting','timeout'):
            state['pending']=transaction.target
            state['attempts']=transaction.attempts;state['transaction']=transaction.state
            if transaction.state=='timeout':
                state['failed']=True;gate.failed=True
                gate.last_error=dict(source='gait_confirmation_timeout',requested_gait=transaction.target,attempts=transaction.attempts)
            return event
    if not active:
        transaction.cancel();state['pending']=0;return None
    if not ready:
        state['pending']=wanted;return None
    if transaction.state in ('idle','cancelled') or transaction.target!=wanted:
        event=transaction.begin(wanted,b.get('Gait'),now)
        state['requested_at']=now
    state['pending']=wanted if transaction.state!='confirmed' else 0
    state['attempts']=transaction.attempts;state['transaction']=transaction.state
    return event


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
    from std_msgs.msg import String, Int32, Float64MultiArray
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    ap=argparse.ArgumentParser();ap.add_argument('--host',default='10.21.31.103');ap.add_argument('--port',type=int,default=30000)
    ap.add_argument('--official-limits',action='store_true',help='Use the NOS component limits for original-binary output')
    ap.add_argument('--source-planner',action='store_true',help='Use source task limits and recovered three-attempt gait transaction')
    ap.add_argument('--execute',action='store_true');ap.add_argument('--telemetry-only',action='store_true')
    ap.add_argument('--enter-navigation',action='store_true',help='Request navigation usage mode once, then respect operator mode changes')
    ap.add_argument('--status-source',choices=('nos','udp'),help='Default: NOS internal telemetry for hardware, UDP for localhost tests')
    ap.add_argument('--nos-target',default='user@10.21.31.106');ap.add_argument('--nos-password-file',type=Path)
    ap.add_argument('--output',type=Path);ap.add_argument('--loopback-test',action='store_true')
    args=ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    if args.execute and args.telemetry_only:ap.error('Choose execution or telemetry-only mode')
    if args.enter_navigation and (args.telemetry_only or not (args.execute or args.loopback_test and args.host=='127.0.0.1')):ap.error('--enter-navigation requires execution or a localhost test')
    if not args.execute and not args.telemetry_only and not (args.loopback_test and args.host=='127.0.0.1'):
        ap.error('Specify --execute for hardware or --loopback-test --host 127.0.0.1 for tests')
    encode_velocity=source_velocity_packet if args.source_planner else official_velocity_packet if args.official_limits else velocity_packet
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.bind(('0.0.0.0',0));sock.setblocking(False)
    # Linux kernel receive times distinguish network/server gaps from a delayed
    # Python receiver. Old queued status must not refresh motion watchdogs.
    timestamp_option=getattr(socket,'SO_TIMESTAMPNS',35)
    try:sock.setsockopt(socket.SOL_SOCKET,timestamp_option,1)
    except OSError:timestamp_option=None
    target=(socket.gethostbyname(args.host),args.port);gate=Gate();rclpy.init();node=rclpy.create_node('first_m20')
    status=node.create_publisher(String,'/first/m20_status',1)
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
            v=m.twist;encode_velocity(v.linear.x,v.linear.y,v.angular.z)
            gate.command=(v.linear.x,v.linear.y,v.angular.z);gate.command_at=time.monotonic()-age
            command_rx['accepted']+=1
        except ValueError:
            gate.failed=True;command_rx['rejected_value']+=1;command_rx['last_rejection']='value'
    # Consume the newest command, never work through a backlog at the same rate
    # as its publisher. A five-frame SensorDataQoS queue exceeds the 120 ms
    # watchdog at 30 Hz and can remain stale after a single scheduler stall.
    latest=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(TwistStamped,'/first/cmd_vel_stamped',receive_command,latest)
    gait_state=dict(wanted=0,wanted_at=-1e9,pending=0,requested_at=0.,failed=False)
    if args.source_planner:
        from source.gait import GaitSwitch
        gait_transaction=GaitSwitch()
    def requested_gait(m):
        if m.data in (0x3002,0x3003) or args.source_planner and m.data==0:
            gait_state['wanted']=m.data;gait_state['wanted_at']=time.monotonic()
    node.create_subscription(Int32,'/first/request_gait',requested_gait,1)
    robot_state=node.create_publisher(Float64MultiArray,'/first/robot_state',latest)
    from status_relay import StatusRelay
    status_source=args.status_source or ('udp' if args.loopback_test and args.host=='127.0.0.1' else 'nos')
    relay=StatusRelay(args.nos_target,args.nos_password_file) if status_source=='nos' else None
    height=[0.];height_stamp=[0.]
    sequence=0;last_heartbeat=-1e9;last_report=-1e9;deadline=time.monotonic();running=[True]
    mode_confirmed=False;mode_attempts=0;mode_requested_at=-1e9;previous_mode=None
    velocity_packets=0;nonzero_packets=0;last_send=None;max_send_period_ms=0.
    receive_stats={};received_at={};wire_received_at={};last_loop=time.monotonic();max_loop_ms=0.
    signal.signal(signal.SIGINT,lambda *_:running.__setitem__(0,False));signal.signal(signal.SIGTERM,lambda *_:running.__setitem__(0,False))
    def send(data):sock.sendto(data,target)
    def receive_document(message_id,doc,received_now,sampled_at,queue_age):
        gate.receive(doc,sampled_at)
        key=f'{doc.get("Type")}/{doc.get("Command")}'
        stats=receive_stats.setdefault(key,dict(packets=0,max_gap_s=0.,max_wire_gap_s=0.,max_receive_queue_ms=0.))
        stats['packets']+=1
        if key in received_at:stats['max_gap_s']=max(stats['max_gap_s'],received_now-received_at[key])
        received_at[key]=received_now
        stats['max_receive_queue_ms']=max(stats['max_receive_queue_ms'],queue_age*1000)
        if key in wire_received_at:stats['max_wire_gap_s']=max(stats['max_wire_gap_s'],sampled_at-wire_received_at[key])
        wire_received_at[key]=sampled_at
        if (doc.get('Type'),doc.get('Command'))==(1002,4):
            raw.publish(String(data=json.dumps(dict(receive_time_ns=time.time_ns(),message_id=message_id,PatrolDevice=doc))))
            height[0]=float(doc.get('Items',{}).get('MotionStatus',{}).get('Height',0.))
            height_stamp[0]=time.time()-(time.monotonic()-sampled_at)
    try:
        while running[0]:
            # Command and gait each arrive at 30 Hz. One spin per 30 Hz send
            # cycle processes only half the callbacks and starves freshness.
            # Drain a bounded number; UDP sending retains its own deadline.
            callback_deadline=time.monotonic()+.002
            for _ in range(8):
                rclpy.spin_once(node,timeout_sec=0)
                if time.monotonic()>=callback_deadline:break
            now=time.monotonic()
            max_loop_ms=max(max_loop_ms,(now-last_loop)*1000);last_loop=now
            # Bound receive work so a telemetry burst cannot starve 30 Hz sends.
            for _ in range(32):
                try:data,ancillary,_,peer=sock.recvmsg(65536,socket.CMSG_SPACE(struct.calcsize('@ll')))
                except BlockingIOError:break
                if peer!=target:continue
                wire_at=None
                for level,kind,value in ancillary:
                    if level==socket.SOL_SOCKET and kind==timestamp_option and len(value)>=struct.calcsize('@ll'):
                        sec,nsec=struct.unpack('@ll',value[:struct.calcsize('@ll')]);wire_at=sec+nsec/1e9
                queue_age=max(0.,time.time()-wire_at) if wire_at is not None else 0.
                received_now=time.monotonic()
                try:
                    message_id,doc=decode_packet(data)
                    # Keep command acknowledgements/errors on the original UDP
                    # socket. Hardware telemetry has exactly one authority.
                    if relay is None or doc.get('Type')!=1002:
                        receive_document(message_id,doc,received_now,received_now-queue_age,queue_age)
                except (ValueError,TypeError):continue
            if relay is not None:
                for mid,doc,sampled_at,age in relay.poll():
                    if doc.get('Type')==1002:
                        try:receive_document(mid,doc,time.monotonic(),sampled_at,age)
                        except (ValueError,TypeError):continue
                if not relay.connected:
                    gate.basic_at=-1e9;gate.motion_at=-1e9
            now=time.monotonic()
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
            if args.source_planner:
                request=source_gait_step(gait_transaction,gait_state,gate,now,args.execute)
                if request is not None:
                    send(encode_velocity(0.,0.,0.,sequence));sequence+=1
                    send(packet(2,23,dict(GaitParam=request),sequence));sequence+=1
            else:update_gait_confirmation(gait_state,gate,now)
            if (not args.source_planner and args.execute and gait_state['wanted'] and now-gait_state['wanted_at']<.35 and not gait_state['pending'] and not gate.failed
                and gate.basic.get('Gait')!=gait_state['wanted'] and gate.basic.get('MotionState')==17
                and now-gate.basic_at<1.2 and now-gate.motion_at<.35 and gate.basic.get('HES')==0
                and gate.basic.get('ControlUsageMode')==1):
                send(packet(2,23,dict(GaitParam=gait_state['wanted']),sequence));sequence+=1
                gait_state['pending']=gait_state['wanted'];gait_state['requested_at']=now
            robot_state.publish(Float64MultiArray(data=[height[0],float(gate.basic.get('Gait',0)),
                float(gate.basic.get('MotionState',0)),now-gate.basic_at,now-gate.motion_at,float(gait_state['pending']),height_stamp[0]]))
            command=gate.output(now)
            if args.source_planner and gait_state['pending']:command=(0.,0.,0.)
            if not args.telemetry_only:
                send(encode_velocity(*command,sequence));sequence+=1;velocity_packets+=1
                if any(command):nonzero_packets+=1
                sent_at=time.monotonic()
                if last_send is not None:max_send_period_ms=max(max_send_period_ms,(sent_at-last_send)*1000)
                last_send=sent_at
            if now-last_report>=1:
                report=dict(gait=gait_state.copy(),command=command,command_age=now-gate.command_at,basic_age=now-gate.basic_at,
                    motion_age=now-gate.motion_at,failed=gate.failed,basic=gate.basic,telemetry_only=args.telemetry_only,
                    navigation_mode_requested=args.enter_navigation,mode_attempts=mode_attempts,mode_confirmed=mode_confirmed,
                    stop_reason='telemetry_only' if args.telemetry_only else gate.stop_reason(now),
                    command_receive=command_rx,velocity_packets=velocity_packets,nonzero_packets=nonzero_packets,
                    max_send_period_ms=max_send_period_ms,
                    max_loop_period_ms=max_loop_ms,kernel_receive_timestamps=timestamp_option is not None,
                    status_source=status_source,status_relay=relay.health() if relay else None,
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
        shutdown=dict(time_ns=time.time_ns(),zero_packets=0,restore_mode_requested=None,errors=[])
        for _ in range(0 if args.telemetry_only else 3):
            try:
                send(encode_velocity(0.,0.,0.,sequence));sequence+=1;shutdown['zero_packets']+=1
            except OSError as exc:shutdown['errors'].append(str(exc))
            time.sleep(1/30)
        # Restore only a mode this process changed, after stop packets. A later
        # operator mode change is never overwritten. UDP delivery is best effort.
        if mode_attempts and previous_mode in (0,2) and gate.basic.get('ControlUsageMode')==1:
            try:
                send(packet(1101,5,dict(Mode=previous_mode),sequence));sequence+=1
                shutdown['restore_mode_requested']=previous_mode
            except OSError as exc:shutdown['errors'].append(str(exc))
        if args.output:(args.output/'shutdown.json').write_text(json.dumps(shutdown,indent=2))
        if relay is not None:relay.close()
        sock.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
