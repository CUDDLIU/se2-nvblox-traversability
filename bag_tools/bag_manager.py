"""Bag storage and subprocess ownership. All writable state stays in the package."""
from datetime import datetime
import fcntl
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import threading
import time
import uuid

import yaml
from rebuild_config import DEPTH, INFO, DESKEWED, MODES, REFERENCE_TOPICS, playback_topics, write_configs
from replay_lidar import prepare_lidar
from input_diagnostics import process_start
from motion_status import TOPICS as MOTION_TOPICS, MODEL_TOPICS
from recording_policy import CACHE_BYTES, critical_qos, recorder_env, cache_losses
from clip_view import center_clip_view
from replay_profiles import DEFAULT_SENSOR_MODE, STAIRS_PROFILE, SOURCE_EXPERIMENT, initial_settings

CORE_TOPICS = (
    '/tf', '/tf_static', '/camera/d435i/depth/image_rect_raw',
    '/camera/d435i/depth/camera_info', '/camera/d435i/color/image_raw',
    '/camera/d435i/color/camera_info', '/camera/d435i/infra1/image_rect_raw',
    '/camera/d435i/infra2/image_rect_raw', '/nvblox_lio/odom',
    '/nvblox_lio/odom_lidar', '/nvblox_node/mesh', '/nvblox/height_mesh',
    '/se2_navmesh/local_display', '/se2_navmesh/local_validity_display',
    '/se2_navmesh/completed_markers', '/se2_navmesh/local_result',
    '/se2_terrain/status', '/IMU', '/nvblox_lio/cloud_world', '/nvblox_lio/diagnostics',
) + MOTION_TOPICS + MODEL_TOPICS
LIDAR_TOPICS = ('/LIDAR/POINTS', '/LIDAR/POINTS2', '/LIDAR/POINTS_NX',
                '/LIDAR/IMU201', '/LIDAR/IMU202')
# Do not add more remote point-cloud readers to the single-ingress bridge.
RECORD_LIDAR_TOPICS = ('/LIDAR/POINTS_NX', '/LIDAR/IMU201', '/LIDAR/IMU202')
RECORD_CORE_TOPICS = tuple(t for t in CORE_TOPICS if not (
    t in ('/nvblox_node/mesh', '/nvblox/height_mesh', '/se2_terrain/status')
    or t.startswith('/se2_navmesh/')))
RETAINED_TOPICS = ('/tf_static', '/se2_navmesh/local_display',
                   '/se2_navmesh/local_validity_display', '/se2_navmesh/completed_markers',
                   '/se2_terrain/status', '/nvblox_robot/robot_description')


def atomic_json(path, data):
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2))
    tmp.replace(path)


def folder_bytes(path):
    return sum(p.stat().st_size for p in path.rglob('*') if p.is_file() and not p.is_symlink())


def metadata(path):
    try:
        value = yaml.safe_load((path / 'metadata.yaml').read_text())
        info = value['rosbag2_bagfile_information']
        topics = {t['topic_metadata']['name']: t['message_count']
                  for t in info.get('topics_with_message_count', [])}
        return dict(duration=info.get('duration', {}).get('nanoseconds', 0) / 1e9,
                    messages=info.get('message_count', 0), topics=topics)
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError):
        return dict(duration=0, messages=0, topics={})


class BagManager:
    def __init__(self, package_root=None, terrain_root=None, live_domain=0, replay_domain=73):
        self.root = Path(package_root or Path(__file__).resolve().parent.parent).resolve()
        self.tools = self.root / 'bag_tools'
        self.bags = self.root / 'bags'
        self.trash = self.bags / '.trash'
        self.runtime = self.tools / 'runtime'
        self.terrain = Path(terrain_root or '/home/nvidia/scanplanner_test/se2_terrain_check')
        for p in (self.bags, self.trash, self.runtime):
            p.mkdir(parents=True, exist_ok=True)
        self.live_domain, self.replay_domain = int(live_domain), int(replay_domain)
        self.base_env = os.environ.copy()
        if self.live_domain == self.replay_domain:
            raise ValueError('实时与回放必须使用不同的 ROS domain')
        self.lock_file = (self.runtime / 'manager.lock').open('a')
        try:
            fcntl.flock(self.lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self.lock_file.close()
            raise RuntimeError('另一个调参窗口已管理 bag，请使用原窗口')
        self.guard = threading.RLock()
        self.children = {}
        self.active = None
        self.mode = 'idle'
        self.message = '就绪'
        self.paused = False
        self.started = 0.
        self.last_disk_check = 0.
        self.closed = False
        self.rebuild = False
        self.sensor_mode = 'depth'
        self.session = None
        self.live_suspended = self.runtime / 'live_rviz_suspended'
        self.mapping_suspended = self.runtime / 'live_mapping_suspended.json'
        self.recording_inputs = self.runtime / 'recording_inputs_active.json'
        self.live_suspended.unlink(missing_ok=True)
        self._write_qos()

    def _write_qos(self):
        retained = dict(reliability='reliable', durability='transient_local',
                        history='keep_last', depth=100)
        # Recording all /tf_static publishers needs more than a single cached transform.
        record = {t: dict(retained) for t in RETAINED_TOPICS}
        record.update(critical_qos())
        play = {t: dict(retained) for t in RETAINED_TOPICS}
        play['/nvblox/height_mesh'] = dict(reliability='reliable', durability='volatile',
                                         history='keep_last', depth=100)
        for name, value in [('record_qos.yaml', record), ('play_qos.yaml', play)]:
            (self.runtime / name).write_text(yaml.safe_dump(value))
        for topic in REFERENCE_TOPICS:
            if topic in play:
                play['/bag_reference' + topic] = play[topic]
        (self.runtime / 'play_qos.yaml').write_text(yaml.safe_dump(play))

    def env(self, replay=False):
        env = self.base_env.copy()
        env['ROS_DOMAIN_ID'] = str(self.replay_domain if replay else self.live_domain)
        if replay:
            # Replay is local to this Jetson, independent from live robot discovery.
            env['ROS_LOCALHOST_ONLY'] = '1'
            env.pop('FASTRTPS_DEFAULT_PROFILES_FILE', None)
            env.pop('FASTDDS_DEFAULT_PROFILES_FILE', None)
            profile = self.root / 'terrain_variants/fastdds_large_shm.xml'
            if profile.is_file():
                env['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(profile)
            env['OPENBLAS_NUM_THREADS'] = '1'
            env['OMP_NUM_THREADS'] = '3'
            env['OMP_WAIT_POLICY'] = 'PASSIVE'
            if self.session:
                env['SE2_MAP_SESSION'] = 'bag:' + str(self.session)
        return env

    def _spawn(self, name, args, directory, replay=False, env_overrides=None):
        log = (directory / (name + '.log')).open('ab', buffering=0)
        try:
            env = self.env(replay)
            if not replay and name in ('record', 'record_map', 'motion_status'):
                env = recorder_env(env, directory / (name + '_transport.xml'))
            if env_overrides:
                env.update(env_overrides)
            proc = subprocess.Popen(args, env=env, cwd=self.root,
                                    stdin=subprocess.DEVNULL, stdout=log, stderr=log,
                                    start_new_session=True)
        finally:
            log.close()
        self.children[name] = proc
        return proc

    def _stop_child(self, name):
        proc = self.children.pop(name, None)
        if proc is None:
            return
        # Send to the group even if the CLI parent has exited: RViz may be its child.
        for sig, timeout in ((signal.SIGINT, 20), (signal.SIGTERM, 5), (signal.SIGKILL, 2)):
            try:
                os.killpg(proc.pid, sig)
            except ProcessLookupError:
                break
            try:
                proc.wait(timeout=timeout)
                # ros2 run normally waits for its child on SIGINT.
                break
            except subprocess.TimeoutExpired:
                continue

    def _helper(self, *args, replay=True):
        result = subprocess.run(['/usr/bin/python3', str(self.tools / 'ros_helper.py'), *args],
                                env=self.env(replay), capture_output=True, text=True, timeout=40)
        if result.returncode:
            raise RuntimeError(result.stderr.strip()[-1200:] or result.stdout.strip()[-1200:])
        return result.stdout

    def _live_rviz_active(self):
        result = subprocess.run(['systemctl', '--user', 'is-active', '--quiet',
                                 'se2-terrain-rviz'], timeout=10)
        return result.returncode == 0

    def _suspend_live_rviz(self):
        was_active = self._live_rviz_active()
        self.live_suspended.touch()
        # Only the startup script's named service is ours to stop.
        result = subprocess.run(['systemctl', '--user', 'stop', 'se2-terrain-rviz'],
                                capture_output=True, text=True, timeout=35)
        if result.returncode and 'not loaded' not in result.stderr:
            self.live_suspended.unlink(missing_ok=True)
            raise RuntimeError('关闭实时 RViz 失败：' + result.stderr.strip())
        return was_active

    def _suspend_live_mapping(self):
        """Release the live GPU mapper; keep LIO and its odometry origin running."""
        units = []
        for unit in ('se2-terrain-check', 'nvblox-d435i-shadow'):
            result = subprocess.run(['systemctl', '--user', 'is-active', '--quiet', unit], timeout=10)
            if result.returncode == 0:
                units.append(unit)
        if not units:
            return False
        settings = {}
        if 'se2-terrain-check' in units:
            # Persist live slider values before terminating their owning node.
            settings = json.loads(self._helper('live-settings', replay=False).strip().splitlines()[-1])
        atomic_json(self.mapping_suspended, dict(units=units, settings=settings))
        for unit in units:
            result = subprocess.run(['systemctl', '--user', 'stop', unit],
                                    capture_output=True, text=True, timeout=40)
            if result.returncode:
                raise RuntimeError('暂停实时建图失败：' + result.stderr.strip())
        return True

    def _start_live(self, rviz_only=False, new_map=False):
        if self.recording_inputs.exists() and not rviz_only:
            # The same camera service currently contains sensors only. Restart
            # it through the original launcher to restore GPU mapping as well.
            subprocess.run(['systemctl', '--user', 'stop', 'nvblox-d435i-shadow'],
                           check=True, timeout=40)
        log_path = self.runtime / 'restore_live.log'
        with log_path.open('w') as log:
            result = subprocess.run(['bash', str(self.terrain / 'start_after_boot.sh'),
                                     '--prepare-record' if new_map else '--rviz-only' if rviz_only else '--restore-live'],
                                    env=self.env(), stdout=log, stderr=subprocess.STDOUT, timeout=90)
        if result.returncode:
            raise RuntimeError('恢复实时重建失败，可再次点击重试。\n' + log_path.read_text()[-1500:])
        time.sleep(1)
        if not self._live_rviz_active():
            raise RuntimeError('实时 RViz 未启动，请查看 ' + str(log_path))
        if not rviz_only and self.mapping_suspended.exists():
            if not new_map:
                self._helper('configure-live', str(self.mapping_suspended), replay=False)
            self.mapping_suspended.unlink()
        if not rviz_only:
            self.recording_inputs.unlink(missing_ok=True)

    def _start_record_inputs(self, settings, new_map):
        atomic_json(self.recording_inputs, dict(settings=settings, mesh_enabled=False))
        atomic_json(self.mapping_suspended, dict(settings=settings, reason='sensor-only recording'))
        self.live_suspended.touch()
        with (self.runtime/'start_recording_inputs.log').open('w') as log:
            result = subprocess.run(['bash', str(self.tools/'start_recording_inputs.sh'),
                '--new-map' if new_map else '--keep-map'], env=self.env(),
                stdout=log, stderr=subprocess.STDOUT, timeout=90)
        if result.returncode:
            raise RuntimeError('启动纯输入录制失败，请查看 start_recording_inputs.log')

    def _restore_after_record(self):
        if self.recording_inputs.exists():
            self.live_suspended.unlink(missing_ok=True)
            self._start_live()

    def restore_live(self):
        with self.guard:
            if self.mode == 'record':
                raise RuntimeError('请先停止并保存录制')
            self.stop_play()
            self.live_suspended.unlink(missing_ok=True)
            self._start_live()
            self.message = '已恢复实时重建与 RViz，调参对象已切回实时程序'

    def checked_path(self, key, deleted=False):
        base = self.trash if deleted else self.bags
        if not key or Path(key).name != key or key.startswith('.'):
            raise ValueError('无效的 bag 名称')
        path = base / key
        if path.is_symlink() or path.resolve().parent != base.resolve() or not path.is_dir():
            raise ValueError('bag 必须位于指定的数据目录中')
        return path

    def _manifest(self, path):
        try:
            return json.loads((path / 'session.json').read_text())
        except (ValueError, OSError):
            return {}

    def _update(self, path, **values):
        doc = self._manifest(path)
        doc.update(values)
        atomic_json(path / 'session.json', doc)

    def entries(self, deleted=False):
        base = self.trash if deleted else self.bags
        rows = []
        for path in sorted(base.iterdir(), reverse=True):
            if path.name.startswith('.') or not path.is_dir() or path.is_symlink():
                continue
            doc = self._manifest(path)
            info = metadata(path / 'data')
            state = doc.get('state', '未知')
            if state == 'recording' and path != self.active:
                state = '异常中断，可尝试修复'
            if state == 'ready' and doc.get('map_summary', {}).get('had_input_gaps'):
                state = '已保存（有断流）'
            rows.append(dict(key=path.name, title=doc.get('title', path.name),
                             created=doc.get('created', ''), state=state,
                             size=folder_bytes(path), **info))
        return rows

    def record(self, title='', lidar=True, settings=None, new_map=True):
        with self.guard:
            if self.mode != 'idle':
                raise RuntimeError('请先停止当前录制或回放')
            if shutil.disk_usage(self.root).free < 2 * 1024**3:
                raise RuntimeError('可用磁盘不足 2 GiB，请先整理 bag')
            key = datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:6]
            path = self.bags / key
            path.mkdir()
            topics = list(RECORD_CORE_TOPICS + (RECORD_LIDAR_TOPICS if lidar else ()))
            self._update(path, title=title.strip() or key, created=datetime.now().isoformat(timespec='seconds'),
                         state='recording', topics_requested=topics, settings=settings or {},
                         depth_note='当前 ROS 深度输入，可能已包含相机驱动滤波；不是未滤波原始深度',
                         mapping_mode='superlio_online', loaded_old_map=False,
                         fresh_map=bool(new_map), map_directory='maps', replay_directory='replays',
                         mesh_enabled_during_recording=False, recording_mode='sensors_and_lio')
            self._update(path, recorder_cache_bytes=CACHE_BYTES,
                         recorder_transport='SHM queue 2048 / segment 64 MiB; inherited UDP; recorder only')
            shutil.copy2(self.runtime / 'record_qos.yaml', path / 'record_qos.yaml')
            for source in (self.terrain / 'paper_config.yaml', self.root / 'config/d435i_viewer.yaml'):
                if source.exists():
                    shutil.copy2(source, path / source.name)
            source = self.root / 'config/m20_nvblox_shadow.yaml'
            if source.exists():
                shutil.copy2(source, path / 'nvblox_config.yaml')
            try:
                atomic_json(self.runtime/'diagnostic_target.json',dict(path=str(path),
                            pid=os.getpid(),process_start=process_start(os.getpid())))
                self._start_record_inputs(settings or {}, new_map)
                for name in ('nvblox-lio', 'nvblox-d435i-shadow'):
                    result = subprocess.run(['systemctl', '--user', 'show', name, '-p', 'InvocationID', '--value'],
                                            capture_output=True, text=True, timeout=10, check=True)
                    self._update(path, **{name.replace('-', '_') + '_session': result.stdout.strip()})
                source = self.terrain.parent / 'Super-LIO-nx-scanplanner-port/install/super_lio/share/super_lio/config/lidar_points.yaml'
                if source.exists():
                    shutil.copy2(source, path / 'superlio_base_config.yaml')
                shutil.copy2(self.tools / 'run_lio_online.sh', path / 'superlio_launch.sh')
                self._spawn('motion_status', ['/usr/bin/python3', str(self.tools / 'motion_capture.py'),
                            str(path / 'telemetry')], path)
                proc = self._spawn('record', ['ros2', 'bag', 'record', '-s', 'sqlite3',
                    '-o', str(path / 'data'), '--max-bag-size', str(1024**3),
                    '--max-cache-size', str(CACHE_BYTES), '--qos-profile-overrides-path',
                    str(path / 'record_qos.yaml'), *topics], path)
                time.sleep(.8)
                if proc.poll() is not None:
                    raise RuntimeError('录制程序退出，请查看 record.log')
                self._spawn('record_map', ['/usr/bin/python3', str(self.tools / 'record_map.py'),
                                          str(path / 'maps')], path)
                deadline = time.monotonic() + 15
                while time.monotonic() < deadline:
                    health = self._map_health(path)
                    if all(health.get('counts', {}).get(k, 0) > 0 for k in ('map', 'odom', 'depth', 'imu', 'lidar')):
                        break
                    if self.children['record_map'].poll() is not None:
                        raise RuntimeError('地图保存程序退出，请查看 record_map.log')
                    time.sleep(.3)
                else:
                    raise RuntimeError('录制输入未就绪：' + health.get('warning', '缺少地图或位姿'))
            except Exception:
                (self.runtime/'diagnostic_target.json').unlink(missing_ok=True)
                self._stop_child('record_map')
                self._stop_child('motion_status')
                self._stop_child('record')
                self._update(path, state='failed', **metadata(path / 'data'))
                try:
                    self._restore_after_record()
                except Exception as restore_error:
                    self._update(path, restore_error=str(restore_error))
                raise
            self.mode, self.active, self.started = 'record', path, time.monotonic()
            self.message = '正在录制传感器与独立地图（Mesh 已关闭）：' + self._manifest(path)['title']
            return path.name

    def finish_record(self):
        with self.guard:
            if self.mode != 'record':
                return
            path = self.active
            if 'record_map' in self.children:
                self._stop_child('record_map')
            if 'motion_status' in self.children:
                self._stop_child('motion_status')
            self._stop_child('record')
            info = metadata(path / 'data')
            success = (path / 'data/metadata.yaml').is_file() and info['messages'] > 0
            self._update(path, state='ready' if success else 'incomplete',
                         recorder_cache_lost=cache_losses(path/'record.log'),
                         finished=datetime.now().isoformat(timespec='seconds'),
                         map_summary=self._map_health(path),
                         motion_status_summary=self._motion_health(path), **info)
            (self.runtime/'diagnostic_target.json').unlink(missing_ok=True)
            self.mode, self.active = 'idle', None
            health = self._map_health(path)
            self.message = ('bag 与地图已保存' if success else '录制没有完整数据；请检查日志或尝试修复')
            lost = cache_losses(path/'record.log')
            if lost:
                self.message += f'；⚠ 录制缓存丢失 {lost} 条消息，详见 record.log'
            if not health.get('points'):
                self.message += '；未获得有效地图点云'
            elif health.get('warning'):
                self.message += '；' + health['warning']
            elif health.get('had_input_gaps'):
                self.message += '；录制期间曾断流，详见 maps/input_events.jsonl'
            motion = self._motion_health(path)
            if (path/'motion_status.log').exists() and not motion.get('frames'):
                self.message += '；未收到运控状态，详见 motion_status.log'
            try:
                self._restore_after_record()
            except Exception as exc:
                self.message += '；恢复实时重建失败，请点击恢复按钮重试：' + str(exc)

    def _motion_health(self, path):
        try:
            return json.loads((path / 'telemetry/health.json').read_text())
        except (OSError, ValueError):
            return {}

    def _map_health(self, path):
        try:
            return json.loads((path / 'maps/map.json').read_text())
        except (OSError, ValueError):
            return {}

    def _adopt_replays(self, path):
        """Bring older runtime exports under their owning bag before management."""
        for session in self.runtime.glob('replay_*'):
            if not session.is_dir() or session.is_symlink():
                continue
            try:
                doc = json.loads((session / 'replay.json').read_text())
            except (OSError, ValueError):
                continue
            if doc.get('bag') == path.name:
                target = path / 'replays' / session.name
                target.parent.mkdir(exist_ok=True)
                if not target.exists():
                    session.rename(target)
                    if self.session == session:
                        self.session = target

    def play(self, key, rate=1., rebuild=False, settings=None, sensor_mode=DEFAULT_SENSOR_MODE,
             terrain_profile=STAIRS_PROFILE):
        with self.guard:
            if self.mode != 'idle':
                raise RuntimeError('请先停止当前录制或回放')
            path = self.checked_path(key)
            info = metadata(path / 'data')
            topics = [t for t in CORE_TOPICS + LIDAR_TOPICS if info['topics'].get(t, 0)]
            if not topics or not (path / 'data/metadata.yaml').is_file():
                raise RuntimeError('没有可回放的数据；异常中断的录制可先尝试修复')
            if not rebuild and not any(t in topics for t in ('/nvblox/height_mesh', '/se2_navmesh/local_display',
                                             '/LIDAR/POINTS_NX')):
                raise RuntimeError('这个包没有已配置的 RViz 可视化话题')
            topics, remaps = playback_topics({t: info['topics'][t] for t in topics}, rebuild, sensor_mode)
            initial = initial_settings(self._manifest(path).get('settings', {}), settings, terrain_profile)
            adapter = prepare_lidar(self.root) if rebuild and sensor_mode != 'depth' else None
            rate = float(rate)
            if rate not in (.25, .5, 1., 2.):
                raise ValueError('请选择 0.25、0.5、1 或 2 倍速')
            self._helper('ensure-empty')
            self._adopt_replays(path)
            (path / 'replays').mkdir(exist_ok=True)
            session = path / 'replays' / ('replay_' + datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:6])
            session.mkdir(exist_ok=True)
            self.session, self.rebuild = session, bool(rebuild)
            self.sensor_mode = sensor_mode if rebuild else None
            atomic_json(session / 'replay.json', dict(bag=key, rebuild=self.rebuild, rate=rate,
                sensor_mode=self.sensor_mode, lidar_adapter=adapter,
                terrain_profile=terrain_profile if rebuild else None,
                terrain_implementation='local_window' if rebuild and terrain_profile == STAIRS_PROFILE else 'recorded',
                profile_source=SOURCE_EXPERIMENT if rebuild and terrain_profile == STAIRS_PROFILE else None))
            live_was_active = False
            mapping_was_active = False
            try:
                live_was_active = self._suspend_live_rviz()
                timed = {'model': False}
                if '/tf' in topics:
                    # Replay TF by its measurement timestamp, not its potentially
                    # delayed recording arrival; use the recorded model and joints.
                    result = subprocess.run(['/usr/bin/python3', str(self.tools/'replay_transforms.py'),
                        'prepare', str(path), str(session)], env=self.env(True),
                        capture_output=True, text=True, timeout=180)
                    if result.returncode:
                        raise RuntimeError('回放 TF 准备失败：' + result.stderr[-1500:])
                    timed = json.loads((session/'timed_transforms.json').read_text())
                    remaps.append('/tf:=/bag_reference/tf')
                    if timed['model']:
                        self._spawn('replay_model', ['/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher',
                            '--ros-args', '-r', '__node:=se2_bag_measured_model',
                            '--params-file', str(session/'recorded_robot.yaml'),
                            '-r', 'joint_states:=/bag_rebuild/joint_states_timed',
                            '-r', 'robot_description:=/bag_rebuild/robot_description'], session, True)
                    self._spawn('timed_tf', ['/usr/bin/python3', str(self.tools/'replay_transforms.py'),
                                'run', str(session)], session, True)
                    deadline = time.monotonic()+30
                    while not (session/'timed_transforms.ready').exists():
                        if self.children['timed_tf'].poll() is not None or time.monotonic()>deadline:
                            raise RuntimeError('回放 TF 发布者未就绪，详见 timed_tf.log')
                        time.sleep(.1)
                if rebuild:
                    mapping_was_active = self._suspend_live_mapping()
                    nvblox, tuning = write_configs(self.root, self.terrain, path, session, initial,
                                                   sensor_mode, terrain_profile)
                    mapper_env = None
                    if adapter:
                        self._spawn('deskew', [str(self.root / 'terrain_variants/deskew/build/terrain_lidar_deskew'),
                            '--ros-args', '-p', 'use_sim_time:=true', '-p', 'restore_scan_time:=false'], session, True)
                        mapper_env = dict(LD_PRELOAD=str(self.root / 'terrain_variants/fast_lidar/libfast_lidar.so'),
                                          SE2_LIDAR_SELF_FILTER='1')
                    self._spawn('nvblox', ['/opt/ros/humble/lib/nvblox_ros/nvblox_node',
                        '--ros-args', '--params-file', str(nvblox),
                        '-r', 'camera_0/depth/image:=' + DEPTH,
                        '-r', 'camera_0/depth/camera_info:=' + INFO,
                        '-r', 'pointcloud:=' + DESKEWED], session, True, env_overrides=mapper_env)
                    self._spawn('height', ['/usr/bin/python3',
                        str(self.root / 'scripts/mesh_height.py'), '--ros-args',
                        '-p', 'use_sim_time:=true'], session, True)
                    checker = (self.root / 'terrain_variants/checker_node.py'
                               if terrain_profile == STAIRS_PROFILE else self.tools / 'rebuild_node.py')
                    self._spawn('terrain', [str(self.terrain.parent / 'se2-terrain-venv/bin/python'),
                        str(checker), '--ros-args', '--params-file', str(tuning)], session, True,
                        env_overrides={'TERRAIN_IMPLEMENTATION': 'local_window'}
                        if terrain_profile == STAIRS_PROFILE else None)
                    self._spawn('reference', ['/usr/bin/python3', str(self.tools/'reference_cache.py')], session, True)
                rviz = yaml.safe_load((self.terrain / 'paper.rviz').read_text())
                vm = rviz['Visualization Manager']
                if self._manifest(path).get('mapping_mode') == 'recorded_superlio_clip':
                    center_clip_view(vm, path)
                if '/nvblox_robot/robot_description' in topics:
                    for display in vm['Displays']:
                        if display.get('Class') == 'rviz_default_plugins/RobotModel':
                            display['Enabled'] = False
                    vm['Displays'].append(dict(Class='rviz_default_plugins/RobotModel',
                        Name='nvblox · 实测关节模型', Enabled=True, Alpha=.65,
                        **{'Description Source': 'Topic', 'Description Topic': {
                            'Value': '/nvblox_robot/robot_description', 'Depth': 1,
                            'Reliability Policy': 'Reliable', 'Durability Policy': 'Transient Local'},
                           'Visual Enabled': True, 'Collision Enabled': False}))
                for display in vm['Displays']:
                    value = display.get('Topic', {}).get('Value')
                    if value and value not in topics and not rebuild:
                        display['Enabled'] = False
                    if value == '/nvblox/height_mesh':
                        display['Name'] = '重建 Mesh' if rebuild else 'Bag 回放 · 录制时的 Mesh'
                vm['Displays'].append(dict(Class='rviz_default_plugins/PointCloud2',
                    Name='Bag 回放 · 雷达点云（可勾选）', Enabled=not rebuild and '/nvblox/height_mesh' not in topics,
                    Topic={'Value': '/LIDAR/POINTS_NX', 'Reliability Policy': 'Best Effort',
                           'Durability Policy': 'Volatile', 'Depth': 5},
                    **{'Size (m)': .025, 'Style': 'Points', 'Color Transformer': 'AxisColor'}))
                if rebuild:
                    # Original outputs are only comparison layers. They cannot
                    # enter the new mapper/checker under their original names.
                    import copy
                    for topic in ('/nvblox/height_mesh', '/se2_navmesh/local_display'):
                        if topic not in topics:
                            continue
                        layer = copy.deepcopy(next(d for d in vm['Displays'] if d.get('Topic', {}).get('Value') == topic))
                        layer['Name'] = '对照 · 录制时的 ' + ('Mesh' if topic.endswith('height_mesh') else '可通行结果')
                        layer['Enabled'] = False
                        layer['Topic']['Value'] = '/bag_rebuild/' + ('reference_mesh' if topic.endswith('height_mesh') else 'reference_display')
                        vm['Displays'].append(layer)
                saved_map = path / 'maps/superlio_map.pcd'
                if saved_map.exists() and self._map_health(path).get('points', 0) > 0:
                    self._spawn('saved_map', ['/usr/bin/python3', str(self.tools / 'map_publisher.py'),
                                             str(saved_map)], session, True)
                    vm['Displays'].append(dict(Class='rviz_default_plugins/PointCloud2',
                        Name='绑定的 SuperLIO 地图（可勾选）', Enabled=False,
                        Topic={'Value': '/bag_reference/superlio_map', 'Reliability Policy': 'Reliable',
                               'Durability Policy': 'Transient Local', 'Depth': 1},
                        **{'Size (m)': .05, 'Style': 'Points', 'Color Transformer': 'AxisColor'}))
                cfg = session / 'bag_playback.rviz'
                cfg.write_text(yaml.safe_dump(rviz, sort_keys=False, allow_unicode=True))
                self._spawn('rviz', ['ros2', 'run', 'rviz2', 'rviz2', '-d', str(cfg),
                    '--ros-args', '-r', '__node:=se2_bag_rviz', '-p', 'use_sim_time:=true'], session, True)
                self._spawn('play', ['ros2', 'bag', 'play', str(path / 'data'),
                    '--clock', '30', '--rate', str(rate), '--start-paused',
                    '--disable-keyboard-controls', '--read-ahead-queue-size', '100',
                    '--qos-profile-overrides-path', str(self.runtime / 'play_qos.yaml'),
                    '--topics', *topics, *(['--remap', *remaps] if remaps else [])], session, True)
                # RViz must subscribe before the first incremental Mesh / clear is published.
                expected = [t for t in ('/nvblox/height_mesh', '/se2_navmesh/local_display') if rebuild or t in topics]
                if not expected:
                    expected = ['/LIDAR/POINTS_NX']
                self._helper('wait-rviz', *expected)
                if timed['model']:
                    self._helper('wait-replay-model')
                if rebuild:
                    self._helper('wait-rebuild', sensor_mode)
                self._helper('resume')
            except Exception as exc:
                self._stop_child('play')
                self._stop_child('rviz')
                for child in ('terrain', 'height', 'nvblox', 'deskew', 'reference', 'saved_map', 'timed_tf', 'replay_model'):
                    self._stop_child(child)
                self.rebuild = False
                self.live_suspended.unlink(missing_ok=True)
                if mapping_was_active or self.mapping_suspended.exists():
                    try:
                        self._start_live()
                    except Exception as restore_error:
                        raise RuntimeError(f'{exc}\n{restore_error}') from exc
                elif live_was_active:
                    try:
                        self._start_live(rviz_only=True)
                    except Exception as restore_error:
                        raise RuntimeError(f'{exc}\n{restore_error}') from exc
                raise
            self.mode, self.active, self.paused = 'play', path, False
            self.message = (f'正在重建回放 · {MODES[sensor_mode]}：' if rebuild else '正在原样回放：') + self._manifest(path).get('title', key)

    def toggle_pause(self):
        with self.guard:
            if self.mode != 'play':
                return
            self._helper('resume' if self.paused else 'pause')
            self.paused = not self.paused
            self.message = ('输入已暂停，仍可调参重算' if self.rebuild else '回放已暂停') if self.paused else ('正在重建回放' if self.rebuild else '正在原样回放')

    def open_global_plan(self, key):
        with self.guard:
            if self.mode != 'idle':
                raise RuntimeError('请先停止录制或关闭当前回放/规划窗口')
            path = self.checked_path(key)
            candidates = list((path / 'global_plans').glob('*/navmesh.json'))
            if not candidates:
                raise RuntimeError('这个 bag 尚未构建全局通行图；请先运行 global_planner 的整包建图流程')
            source = max(candidates, key=lambda p: p.stat().st_mtime)
            directory = source.parent.resolve()
            if not directory.is_relative_to(path.resolve()):
                raise ValueError('全局图不能指向 bag 目录之外')
            if not all((directory / name).is_file() for name in ('mesh.npz', 'field.npz')):
                raise RuntimeError('全局图不完整，缺少 Mesh 或可通行场')
            session = directory / 'runtime'
            session.mkdir(exist_ok=True)
            (session / 'status.json').unlink(missing_ok=True)
            self._suspend_live_rviz()
            self._spawn('global_plan', ['/bin/bash', str(self.root / 'global_planner/start.sh'), str(directory)], session,
                        env_overrides={'SE2_GLOBAL_DOMAIN': '79'})
            self.mode, self.active, self.session = 'plan', path, session
            self.rebuild, self.paused = False, False
            self.message = '全局规划启动中；在 RViz 使用 Publish Point 点击目标楼层地面'

    def global_plan_action(self, action):
        with self.guard:
            if self.mode != 'plan' or action not in ('pick_start', 'reset_start'):
                raise RuntimeError('请先打开全局路径规划')
            env = self.env(True)
            env['ROS_DOMAIN_ID'] = '79'
            result = subprocess.run(['ros2', 'service', 'call', '/se2_global/' + action,
                                     'std_srvs/srv/Trigger', '{}'], env=env,
                                    capture_output=True, text=True, timeout=10)
            if result.returncode:
                raise RuntimeError(result.stderr[-1000:])
            self.message = ('下一次 RViz Publish Point 点击将设置起点' if action == 'pick_start'
                            else '已恢复为 bag 录制终点')

    def stop_play(self):
        with self.guard:
            self._stop_child('global_plan')
            self._stop_child('play')
            self._stop_child('rviz')
            for child in ('terrain', 'height', 'nvblox', 'deskew', 'reference', 'saved_map', 'timed_tf', 'replay_model'):
                self._stop_child(child)
            if self.mode in ('play', 'finished', 'plan'):
                self.mode, self.active, self.paused = 'idle', None, False
                self.message = '回放已关闭；点击“恢复实时重建”打开实时 RViz'
            self.rebuild = False

    def save_comparison(self, title=''):
        with self.guard:
            if not self.rebuild or self.mode not in ('play', 'finished'):
                raise RuntimeError('请先启动重建回放')
            data = json.loads(self._helper('snapshot'))
            directory = self.session / 'comparisons'
            directory.mkdir(exist_ok=True)
            destination = directory / (datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:6] + '.json')
            data.update(title=title.strip(), bag=self.active.name, sensor_mode=self.sensor_mode,
                        replay=json.loads((self.session / 'replay.json').read_text()),
                        note='原结果可能包含录制开始前的地图；重建使用所选包内传感器与已录位姿')
            atomic_json(destination, data)
            self.message = '对比快照已保存：' + destination.name
            return str(destination)

    def rename(self, key, title, deleted=False):
        with self.guard:
            path = self.checked_path(key, deleted)
            if not title.strip() or len(title) > 120:
                raise ValueError('名称长度应为 1–120 个字符')
            self._update(path, title=title.strip())

    def move_to_trash(self, key):
        with self.guard:
            path = self.checked_path(key)
            if path == self.active:
                raise RuntimeError('请先停止这个 bag 的录制或回放')
            self._adopt_replays(path)
            target = self.trash / key
            if target.exists():
                raise RuntimeError('回收站已有同名包')
            path.rename(target)
            if self.session and self.session.is_relative_to(path):
                self.session = target / self.session.relative_to(path)

    def restore(self, key):
        with self.guard:
            path = self.checked_path(key, True)
            target = self.bags / key
            if target.exists():
                raise RuntimeError('数据目录已有同名包')
            path.rename(target)
            if self.session and self.session.is_relative_to(path):
                self.session = target / self.session.relative_to(path)

    def repair(self, key):
        with self.guard:
            if self.mode != 'idle':
                raise RuntimeError('请先停止录制或回放')
            path = self.checked_path(key)
            if not (path / 'data').is_dir():
                raise RuntimeError('没有 bag 数据目录')
            # Reindex only reconstructs metadata; never removes SQLite data.
            with (path / 'repair.log').open('ab') as log:
                result = subprocess.run(['ros2', 'bag', 'reindex', str(path / 'data')],
                    env=self.env(), stdout=log, stderr=log, timeout=180)
            info = metadata(path / 'data')
            if result.returncode or not info['messages']:
                raise RuntimeError('修复未成功，详情见 repair.log')
            self._update(path, state='ready', **info)
            self.message = '索引已修复'

    def purge(self, key):
        with self.guard:
            path = self.checked_path(key, True)
            self._adopt_replays(path)
            shutil.rmtree(path)
            if self.session and self.session.is_relative_to(path):
                self.session = None
            self.message = '已永久删除所选 bag、绑定地图及回放结果'

    def tick(self):
        with self.guard:
            if self.mode == 'plan':
                if self.children['global_plan'].poll() is not None:
                    code = self.children['global_plan'].returncode
                    self.stop_play()
                    self.message = ('全局规划窗口已关闭；可重新打开或恢复实时重建' if code == 0 else
                                    '全局规划退出，请查看诊断日志 global_plan.log / planner.log')
                else:
                    try:
                        state = json.loads((self.session / 'status.json').read_text())
                        self.message = state['message']
                    except (OSError, ValueError, KeyError):
                        pass
            elif self.mode == 'record':
                health = self._map_health(self.active)
                self.message = '正在录制传感器与地图（Mesh 已关闭）：' + self._manifest(self.active).get('title', self.active.name)
                lost = cache_losses(self.active/'record.log')
                if lost:
                    self.message += f'；⚠ 录制缓存已丢 {lost} 条消息'
                if health.get('warning'):
                    self.message += '；⚠ ' + health['warning']
                if 'motion_status' in self.children:
                    motion = self._motion_health(self.active)
                    age = motion.get('age_s')
                    stale = time.time_ns() - motion.get('updated_time_ns', 0) > 3_000_000_000
                    if self.children['motion_status'].poll() is not None:
                        self.message += '；⚠ 运控状态接收退出，其他输入继续录制'
                    elif stale or age is None or age > 1:
                        self.message += '；⚠ 运控状态尚未收到或已中断'
                    else:
                        self.message += f"；运控状态 {motion.get('receive_hz', 0):.1f} Hz"
                if 'record_map' in self.children and self.children['record_map'].poll() is not None:
                    self.finish_record()
                    self.message = '地图保存进程退出，已停止并保存 bag；请检查 record_map.log'
                    return self.tick()
                if self.children['record'].poll() is not None:
                    self.finish_record()
                    self.message = '录制进程已退出，请检查 bag 状态及 record.log'
                elif time.monotonic() - self.last_disk_check > 3:
                    self.last_disk_check = time.monotonic()
                    if shutil.disk_usage(self.root).free < 2 * 1024**3:
                        self.finish_record()
                        self.message = '磁盘剩余不足 2 GiB，已自动停止并保存录制'
            elif self.mode in ('play', 'finished'):
                failed = [name for name in ('nvblox', 'height', 'terrain', 'deskew', 'reference', 'timed_tf', 'replay_model')
                          if name in self.children and self.children[name].poll() is not None]
                if failed:
                    details = []
                    for name in failed:
                        log = self.session / (name + '.log')
                        if log.exists():
                            with log.open('rb') as stream:
                                stream.seek(max(0, log.stat().st_size-8192))
                                details.append(stream.read().decode(errors='replace'))
                    reason = ('GPU 内存不足' if any('out of memory' in d.lower() for d in details)
                              else '重建进程异常退出：' + ', '.join(failed))
                    atomic_json(self.session / 'failure.json', dict(reason=reason,
                                failed_processes=failed, time=datetime.now().isoformat()))
                    self.stop_play()
                    self.message = reason + '，回放已停止；可打开回放结果查看日志，或恢复实时重建'
                elif self.children['rviz'].poll() is not None:
                    self.stop_play()
                    self.message = '回放 RViz 已关闭，回放已停止'
                elif self.mode == 'play' and self.children['play'].poll() is not None:
                    code = self.children['play'].returncode
                    self.mode = 'finished'
                    self.message = (('输入播放完毕；重建结果保留，可继续调参、保存对比。' if self.rebuild else '回放结束；RViz 保留最后画面。关闭回放后可重新播放。')
                                    if code == 0 else '回放异常退出，请查看 bag_tools/runtime 中的日志')
            return dict(mode=self.mode, message=self.message, paused=self.paused,
                        rebuild=self.rebuild, session=str(self.session) if self.session else None,
                        elapsed=time.monotonic()-self.started if self.mode == 'record' else 0,
                        free=shutil.disk_usage(self.root).free,
                        active=self.active.name if self.active else None,
                        input_health=self.input_health())

    def input_health(self):
        try:
            doc=json.loads((self.root/'diagnostics/live/latest.json').read_text())
            if time.time()-doc['time_unix']>5:
                return '输入诊断未更新，请检查 se2-input-diagnostics 服务'
            return '实时输入：'+doc['summary']
        except (OSError,ValueError,KeyError):
            return '输入诊断尚未启动；运行原启动命令会自动开启'

    def close(self):
        with self.guard:
            if self.closed:
                return
            self.finish_record()
            self.stop_play()
            self.live_suspended.unlink(missing_ok=True)
            fcntl.flock(self.lock_file, fcntl.LOCK_UN)
            self.lock_file.close()
            self.closed = True
