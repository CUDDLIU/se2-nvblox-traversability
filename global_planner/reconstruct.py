#!/usr/bin/env python3
"""Rebuild one complete sensor bag into a persisted incremental mesh snapshot.

Uses an isolated ROS domain, recorded poses, and the current LiDAR preset.
Only owns its mapper/player/TF/deskew children. No robot commands or live TF.
"""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'bag_tools'))
sys.path.insert(0, str(ROOT / 'terrain_variants/local_window'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--domain', type=int, default=78)
    args = parser.parse_args()
    bag = args.bag.resolve()
    out = args.output or bag / 'global_plans' / datetime.now().strftime('map_%Y%m%d_%H%M%S')
    out.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='1',
                      FASTRTPS_DEFAULT_PROFILES_FILE=str(ROOT / 'terrain_variants/fastdds_large_shm.xml'))
    os.environ.pop('FASTDDS_DEFAULT_PROFILES_FILE', None)
    import numpy as np
    import yaml
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from nvblox_msgs.msg import Mesh
    from rosgraph_msgs.msg import Clock
    from rosbag2_interfaces.srv import Resume
    from rebuild_config import write_configs, DESKEWED
    from replay_lidar import prepare_lidar
    from replay_profiles import STAIRS_PROFILE
    from replay_transforms import prepare, source_key
    from mesh_wire import decode_mesh

    adapter = prepare_lidar(ROOT)
    meta = yaml.safe_load((bag / 'data/metadata.yaml').read_text())['rosbag2_bagfile_information']
    duration = meta['duration']['nanoseconds'] / 1e9
    start = meta['starting_time']['nanoseconds_since_epoch'] / 1e9
    prepare(bag, out)
    config = json.loads((bag / 'session.json').read_text()).get('settings', {})
    # Replay preset takes precedence over the bag's old flat-ground limits.
    config.update(max_step=.20, max_slope_deg=40.)
    terrain = ROOT.parent / 'se2_terrain_check'
    if not (terrain / 'paper_config.yaml').is_file():
        terrain = ROOT / 'deployment/jetson'
    mapper, _ = write_configs(ROOT, terrain, bag, out,
                             config, 'lidar', STAIRS_PROFILE)
    doc = yaml.safe_load(mapper.read_text())
    # A full offline map must not drop distant blocks from the publication.
    doc['nvblox_node']['ros__parameters']['layer_visualization_exclusion_radius_m'] = 0.
    mapper.write_text(yaml.safe_dump(doc, sort_keys=False))
    qos = out / 'play_qos.yaml'
    qos.write_text(yaml.safe_dump({'/tf_static': {'durability': 'transient_local',
                                                'reliability': 'reliable', 'history': 'keep_all'}}))
    report = dict(bag=bag.name, source_key=source_key(bag), frame='nvblox_odom',
                  domain=args.domain, input='lidar', duration_s=duration,
                  adapter=adapter, state='starting', control_outputs='none')
    (out / 'reconstruction.json').write_text(json.dumps(report, indent=2))
    rclpy.init()
    node = rclpy.create_node('se2_global_mesh_capture')
    blocks, trajectory, counts = {}, [], dict(mesh_messages=0, deleted_blocks=0)
    clock = [start]
    frame = ['nvblox_odom']
    children, logs = {}, []
    running = [True]

    def mesh(raw):
        msg = decode_mesh(raw)
        if msg.clear:
            blocks.clear()
        frame[0] = msg.header.frame_id
        for key, block in zip(msg.block_indices, msg.blocks):
            key = (key.x, key.y, key.z)
            if len(block.triangles_array):
                blocks[key] = (block.vertices_array.copy(), block.triangles_array.copy())
            else:
                blocks.pop(key, None)
                counts['deleted_blocks'] += 1
        counts['mesh_messages'] += 1

    def odom(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        trajectory.append([msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                           p.x, p.y, p.z, q.x, q.y, q.z, q.w])

    node.create_subscription(Mesh, '/nvblox_node/mesh', mesh, 20, raw=True)
    node.create_subscription(Odometry, '/nvblox_lio/odom_lidar', odom, qos_profile_sensor_data)
    node.create_subscription(Clock, '/clock',
                             lambda m: clock.__setitem__(0, m.clock.sec + m.clock.nanosec / 1e9),
                             qos_profile_sensor_data)

    def spawn(name, command, additions=None):
        log = (out / (name + '.log')).open('w')
        logs.append(log)
        env = os.environ.copy()
        env.update(additions or {})
        children[name] = subprocess.Popen(command, env=env, stdout=log,
                                           stderr=subprocess.STDOUT, start_new_session=True)

    def stop(*_):
        running[0] = False

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    print('OUTPUT ' + str(out), flush=True)
    error = None
    try:
        spawn('timed_tf', ['/usr/bin/python3', str(ROOT / 'bag_tools/replay_transforms.py'), 'run', str(out)])
        spawn('deskew', [str(ROOT / 'terrain_variants/deskew/build/terrain_lidar_deskew'),
                         '--ros-args', '-p', 'use_sim_time:=true', '-p', 'restore_scan_time:=false'])
        spawn('mapper', ['/opt/ros/humble/lib/nvblox_ros/nvblox_node', '--ros-args',
                         '--params-file', str(mapper), '-r', 'pointcloud:=' + DESKEWED],
              {'LD_PRELOAD': str(ROOT / 'terrain_variants/fast_lidar/libfast_lidar.so'),
               'SE2_LIDAR_SELF_FILTER': '1'})
        topics = ['/tf', '/tf_static', '/nvblox_lio/odom', '/nvblox_lio/odom_lidar', '/LIDAR/POINTS_NX']
        spawn('player', ['ros2', 'bag', 'play', str(bag / 'data'), '--clock', '50', '--rate', '1',
                         '--start-paused', '--disable-keyboard-controls', '--read-ahead-queue-size', '5000',
                         '--qos-profile-overrides-path', str(qos), '--topics', *topics,
                         '--remap', '/tf:=/bag_reference/tf'])
        resume = node.create_client(Resume, '/rosbag2_player/resume')
        until = time.monotonic() + 45
        ready = False
        while running[0] and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.1)
            required = {DESKEWED: 'nvblox_node', '/LIDAR/POINTS_NX': 'terrain_lidar_deskew'}
            ready = ((out / 'timed_transforms.ready').exists() and resume.service_is_ready()
                     and all(any(s.node_name == n for s in node.get_subscriptions_info_by_topic(t))
                             for t, n in required.items()))
            if ready:
                break
        if not ready:
            raise RuntimeError('Reconstruction subscribers did not become ready')
        future = resume.call_async(Resume.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        if not future.done() or future.exception():
            raise RuntimeError('Cannot resume bag')
        started = last = time.monotonic()
        eof = None
        while running[0]:
            rclpy.spin_once(node, timeout_sec=.05)
            now = time.monotonic()
            for name, process in children.items():
                if name != 'player' and process.poll() is not None:
                    raise RuntimeError(f'{name} stopped unexpectedly: {process.returncode}')
            if now - started > duration + 90:
                raise RuntimeError('Complete bag replay timed out')
            if now - last > 20:
                print(f'REBUILD {clock[0]-start:.1f}/{duration:.1f}s, {len(blocks)} blocks', flush=True)
                last = now
            if children['player'].poll() is not None:
                if children['player'].returncode != 0:
                    raise RuntimeError('Bag player failed')
                eof = eof or now
                if now - eof >= 4:
                    break
        if not running[0]:
            raise RuntimeError('Reconstruction interrupted')
        if not blocks or not trajectory or clock[0] - start < duration - 1:
            raise RuntimeError('Incomplete reconstruction; refusing to label it a global map')
        arrays = {'keys': np.asarray(list(blocks), dtype=np.int32),
                  'trajectory': np.asarray(trajectory, dtype=np.float64)}
        for i, (vertices, triangles) in enumerate(blocks.values()):
            arrays['v' + str(i)] = vertices
            arrays['t' + str(i)] = triangles
        np.savez_compressed(out / 'mesh.npz', **arrays)
        report.update(state='complete', frame=frame[0], elapsed_bag_s=clock[0]-start,
                      blocks=len(blocks), triangles=sum(len(t) for _, t in blocks.values()), **counts)
    except Exception as exc:
        error = exc
        report.update(state='failed', error=str(exc), **counts)
    finally:
        for process in children.values():
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in children.values():
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        for log in logs:
            log.close()
        (out / 'reconstruction.json').write_text(json.dumps(report, indent=2))
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    if error:
        raise error
    print('COMPLETE ' + str(out / 'mesh.npz'), flush=True)


if __name__ == '__main__':
    main()
