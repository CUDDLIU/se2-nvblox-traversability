"""M20 telemetry receiver. The only outbound packet is the 100/100 heartbeat.

Joint names and units are from SDK guide V1.3.0 section 1.3.1.2.
These are protocol coordinates, NOT calibrated URDF joint coordinates.
"""
import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import signal
import socket
import struct
import time

PREFIX = '/nvblox_robot/motion_status'
TOPICS = (PREFIX + '/raw', PREFIX + '/joints', PREFIX + '/wheels')
MODEL_TOPICS = ('/nvblox_robot/joint_states', '/nvblox_robot/robot_description')
NAMES = tuple(leg + joint for leg in ('LeftFront', 'RightFront', 'LeftBack', 'RightBack')
              for joint in ('HipX', 'HipY', 'Knee', 'Wheel'))
MAGIC = bytes.fromhex('eb91eb90')
HEADER = struct.Struct('<4sHHB7x')


def heartbeat(sequence):
    body = json.dumps({'PatrolDevice': {'Type': 100, 'Command': 100,
        'Time': datetime.now().strftime('%Y-%m-%d %H:%M:%S'), 'Items': {}}},
        separators=(',', ':')).encode()
    return HEADER.pack(MAGIC, len(body), sequence % 65536, 1) + body


def parse_packet(packet):
    if len(packet) < HEADER.size:
        raise ValueError('truncated protocol header')
    magic, size, sequence, fmt = HEADER.unpack_from(packet)
    if magic != MAGIC or fmt != 1 or len(packet) != HEADER.size + size:
        raise ValueError('invalid magic, format, or packet size')
    def reject_constant(value):
        raise ValueError('non-finite JSON value: ' + value)
    value = json.loads(packet[HEADER.size:].decode('utf-8'), parse_constant=reject_constant)
    patrol = value.get('PatrolDevice') if isinstance(value, dict) else None
    if not isinstance(patrol, dict) or not isinstance(patrol.get('Items'), dict):
        raise ValueError('missing PatrolDevice/Items')
    return sequence, patrol


def split_joints(patrol):
    """Return 12 measured angles and 4 measured speeds; never invent wheel angles."""
    motor = patrol['Items'].get('MotorStatus')
    if not isinstance(motor, dict):
        raise ValueError('missing MotorStatus')
    values = motor.get('Joint')
    if values is None:
        if not all(name in motor for name in NAMES):
            raise ValueError('missing named joints')
        values = [motor[name] for name in NAMES]
    if not isinstance(values, list) or len(values) != 16:
        raise ValueError('Joint must have 16 elements')
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
           for v in values):
        raise ValueError('joint values must be finite numbers')
    for name, value in zip(NAMES, values):
        if name in motor:
            named = motor[name]
            if (isinstance(named, bool) or not isinstance(named, (int, float))
                    or not math.isfinite(named) or abs(named - value) > 1e-5):
                raise ValueError('named/array joint mismatch: ' + name)
    angles = [(n, float(v)) for i, (n, v) in enumerate(zip(NAMES, values)) if i % 4 != 3]
    speeds = [(n, float(v)) for i, (n, v) in enumerate(zip(NAMES, values)) if i % 4 == 3]
    return angles, speeds


def atomic_json(path, value):
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(value, ensure_ascii=False, indent=2))
    tmp.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--host', default='10.21.31.103')
    parser.add_argument('--port', type=int, default=30000)
    parser.add_argument('--duration', type=float, default=0)
    parser.add_argument('--ros', action='store_true')
    parser.add_argument('--latest-only', action='store_true', help='bounded live state; bags archive all frames')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    node = None
    if args.ros:
        import rclpy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        rclpy.init(args=[])
        node = rclpy.create_node('se2_motion_status_recorder')
        raw_pub = node.create_publisher(String, TOPICS[0], 100)
        joint_pub = node.create_publisher(JointState, TOPICS[1], 100)
        wheel_pub = node.create_publisher(JointState, TOPICS[2], 100)

    running = True
    def stop(*_):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    target = (socket.gethostbyname(args.host), args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
    sock.bind(('', 0))
    sock.settimeout(.05)
    start = time.monotonic()
    next_hb = next_health = start
    first = last = None
    status = dict(schema_version=1, peer=list(target), frames=0, joint_frames=0,
        heartbeats=0, invalid_packets=0, invalid_joint_frames=0, socket_errors=0,
        gaps_over_300ms=0, max_interval_s=0., last_receive_time_ns=None,
        time_basis='local receive time; robot Time retained verbatim; not acquisition time',
        joint_coordinates='M20 protocol; URDF sign/zero calibration not verified',
        leg_unit='rad', wheel_unit='rad/s', expected_hz=10,
        control_outputs='heartbeat 100/100 only; no actuation or TF')

    def health(final=False):
        now = time.monotonic()
        status.update(elapsed_s=now-start, running=not final,
            receive_hz=(status['frames']-1)/(last-first) if first is not None and last > first else 0.,
            age_s=now-last if last is not None else None, updated_time_ns=time.time_ns())
        atomic_json(args.output/'health.json', status)

    try:
        with (Path(os.devnull) if args.latest_only else args.output/'motion_status.jsonl').open('a', buffering=1) as stream:
            health()
            while running and (not args.duration or time.monotonic()-start < args.duration):
                now = time.monotonic()
                if now >= next_hb:
                    try:
                        sock.sendto(heartbeat(status['heartbeats']), target)
                        status['heartbeats'] += 1
                    except OSError as e:
                        status['socket_errors'] += 1
                        status['last_error'] = str(e)
                    next_hb = now + .8
                if now >= next_health:
                    health()
                    next_health = now + 1
                if node is not None:
                    rclpy.spin_once(node, timeout_sec=0)
                try:
                    packet, address = sock.recvfrom(65535)
                except socket.timeout:
                    continue
                except OSError as e:
                    status['socket_errors'] += 1
                    status['last_error'] = str(e)
                    continue
                received_ns, received_mono = time.time_ns(), time.monotonic()
                if address != target:
                    continue
                try:
                    sequence, patrol = parse_packet(packet)
                except (ValueError, UnicodeError):
                    status['invalid_packets'] += 1
                    continue
                if patrol.get('Type') != 1002 or patrol.get('Command') != 4:
                    continue
                record = dict(receive_time_ns=received_ns, elapsed_s=received_mono-start,
                              message_id=sequence, PatrolDevice=patrol)
                raw = json.dumps(record, ensure_ascii=False, allow_nan=False)
                stream.write(raw + '\n')
                if args.latest_only:
                    atomic_json(args.output/'latest.json', record)
                if last is not None:
                    dt = received_mono-last
                    status['max_interval_s'] = max(status['max_interval_s'], dt)
                    status['gaps_over_300ms'] += int(dt > .3)
                if first is None:
                    first = received_mono
                last = received_mono
                status['frames'] += 1
                status['last_receive_time_ns'] = received_ns
                if node is not None:
                    raw_pub.publish(String(data=raw))
                try:
                    angles, speeds = split_joints(patrol)
                except ValueError as e:
                    status['invalid_joint_frames'] += 1
                    status['last_error'] = str(e)
                    continue
                status['joint_frames'] += 1
                if node is not None:
                    stamp = rclpy.time.Time(nanoseconds=received_ns).to_msg()
                    joints = JointState()
                    joints.header.stamp = stamp
                    joints.name = [n for n, _ in angles]
                    joints.position = [v for _, v in angles]
                    joint_pub.publish(joints)
                    wheels = JointState()
                    wheels.header.stamp = stamp
                    wheels.name = [n for n, _ in speeds]
                    wheels.velocity = [v for _, v in speeds]
                    wheel_pub.publish(wheels)
    finally:
        sock.close()
        health(final=True)
        print(json.dumps(status, ensure_ascii=False), flush=True)
        if node is not None:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
