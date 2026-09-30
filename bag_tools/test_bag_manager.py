import json
import hashlib
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import yaml
from bag_manager import BagManager, CORE_TOPICS, LIDAR_TOPICS, RECORD_LIDAR_TOPICS, metadata
from rebuild_config import DEPTH, INFO, LIDAR, SENSOR_TOPICS, REFERENCE_TOPICS, playback_topics, write_configs
from replay_lidar import prepare_lidar
from motion_status import TOPICS as MOTION_TOPICS
from bag_manager import RECORD_CORE_TOPICS
from input_diagnostics import classify
from replay_profiles import STAIRS_PROFILE, initial_settings


class BagTests(unittest.TestCase):
    def test_global_plan_owns_viewer_and_protects_bag_bundle(self):
        path = self.bag()
        directory = path / 'global_plans/map_test'
        directory.mkdir(parents=True)
        for name in ('navmesh.json', 'mesh.npz', 'field.npz'):
            (directory / name).write_text('{}')
        with patch.object(self.manager, '_spawn') as spawn, patch.object(self.manager, '_suspend_live_rviz'):
            self.manager.open_global_plan('test')
        self.assertEqual(self.manager.mode, 'plan')
        self.assertEqual(spawn.call_args.args[0], 'global_plan')
        with self.assertRaises(RuntimeError):
            self.manager.move_to_trash('test')
        self.manager.stop_play()
        self.assertEqual(self.manager.mode, 'idle')
        self.manager.move_to_trash('test')
        self.assertTrue((self.manager.trash/'test/global_plans/map_test/navmesh.json').is_file())

    def test_global_plan_missing_map_and_recording_leave_state_unchanged(self):
        self.bag()
        with self.assertRaises(RuntimeError):self.manager.open_global_plan('test')
        self.assertEqual(self.manager.mode, 'idle')
        self.manager.mode = 'record'
        with self.assertRaises(RuntimeError):self.manager.open_global_plan('test')
        self.manager.mode = 'idle'

    def test_sensor_recording_retains_rebuild_inputs_without_mesh_outputs(self):
        for mode in ('depth', 'fusion', 'lidar'):
            topics,_ = playback_topics({t:1 for t in RECORD_CORE_TOPICS+RECORD_LIDAR_TOPICS}, True, mode)
            self.assertIn('/tf',topics)
        self.assertIn('/nvblox_robot/joint_states',RECORD_CORE_TOPICS)
        self.assertFalse(any('mesh' in t or t.startswith('/se2_navmesh/') for t in RECORD_CORE_TOPICS))

    def test_recording_diagnostics_still_check_depth_without_demanding_mesh(self):
        streams={k:dict(receive_age_s=.1,hz=10) for k in ('lidar','imu','odom','pose_tf','depth')}
        services={k:{'ActiveState':'active'} for k in ('nvblox-lio.service','nvblox-d435i-shadow.service')}
        result=classify(streams,services,True,{},recording=True)
        self.assertFalse(result['problems'])
        streams['depth']['receive_age_s']=4
        result=classify(streams,services,True,{},recording=True)
        self.assertTrue(any('D435i' in p for p in result['problems']))

    def test_finished_recording_is_saved_even_if_live_restore_fails(self):
        self.manager.active=self.bag();self.manager.mode='record'
        with patch.object(self.manager,'_restore_after_record',side_effect=RuntimeError('camera restart failed')):
            self.manager.finish_record()
        self.assertEqual(self.manager.mode,'idle')
        self.assertEqual(self.manager.entries()[0]['state'],'ready')
        self.assertIn('恢复实时重建失败',self.manager.message)

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.manager = BagManager(self.root, self.root / 'terrain')

    def tearDown(self):
        self.manager.close()
        self.tmp.cleanup()

    def bag(self, key='test'):
        path = self.manager.bags / key
        (path / 'data').mkdir(parents=True)
        (path / 'session.json').write_text(json.dumps(dict(title='测试', state='ready')))
        (path / 'data/metadata.yaml').write_text(yaml.safe_dump(dict(rosbag2_bagfile_information=dict(
            duration=dict(nanoseconds=2_000_000_000), message_count=4,
            topics_with_message_count=[dict(topic_metadata=dict(name='/nvblox/height_mesh'), message_count=4)]))))
        return path

    def test_metadata_and_management_preserve_data(self):
        path = self.bag()
        self.assertEqual(metadata(path / 'data')['duration'], 2)
        self.manager.rename('test', '走廊瓷砖')
        self.assertEqual(self.manager.entries()[0]['title'], '走廊瓷砖')
        self.manager.move_to_trash('test')
        self.assertFalse(path.exists())
        self.assertEqual(len(self.manager.entries(deleted=True)), 1)
        self.manager.restore('test')
        self.assertTrue((path / 'data/metadata.yaml').exists())

    def test_refuses_paths_outside_storage(self):
        (self.manager.bags / 'escape').symlink_to(self.root, target_is_directory=True)
        for key in ('../other', str(self.root), 'escape', '.trash', ''):
            with self.assertRaises(ValueError):
                self.manager.checked_path(key)

    def test_running_bag_cannot_be_trashed(self):
        self.manager.active = self.bag()
        with self.assertRaises(RuntimeError):
            self.manager.move_to_trash('test')

    def test_only_trash_can_be_purged(self):
        path = self.bag()
        with self.assertRaises(ValueError):
            self.manager.purge('test')
        self.assertTrue(path.exists())
        self.manager.move_to_trash('test')
        self.manager.purge('test')
        self.assertFalse((self.manager.trash / 'test').exists())

    def test_process_lock(self):
        with self.assertRaises(RuntimeError):
            BagManager(self.root)

    def test_replay_environment_isolated(self):
        with patch.dict(os.environ, {'FASTRTPS_DEFAULT_PROFILES_FILE': '/live.xml'}):
            live, replay = self.manager.env(), self.manager.env(True)
        self.assertEqual(live['ROS_DOMAIN_ID'], '0')
        self.assertEqual(replay['ROS_DOMAIN_ID'], '73')
        self.assertEqual(replay['ROS_LOCALHOST_ONLY'], '1')
        self.assertNotIn('FASTRTPS_DEFAULT_PROFILES_FILE', replay)

    def test_disk_guard_stops_and_finalizes_recording(self):
        self.manager.active = self.bag()
        self.manager.mode = 'record'
        process = Mock()
        process.poll.return_value = None
        self.manager.children['record'] = process
        with patch('bag_manager.shutil.disk_usage', return_value=Mock(free=1024)), \
                patch.object(self.manager, '_stop_child') as stop:
            result = self.manager.tick()
        stop.assert_called_once_with('record')
        self.manager.children.clear()
        self.assertEqual(result['mode'], 'idle')
        self.assertIn('自动停止', result['message'])
        self.assertEqual(self.manager.entries()[0]['state'], 'ready')

    def test_record_failure_is_visible_and_does_not_claim_recording(self):
        with patch.object(self.manager, '_start_live'), \
                patch('bag_manager.subprocess.run', return_value=Mock(returncode=0, stdout='invocation')), \
                patch('bag_manager.shutil.copy2'), \
                patch.object(self.manager, '_spawn', side_effect=OSError('cannot start')):
            with self.assertRaises(OSError):
                self.manager.record('失败测试')
        self.assertEqual(self.manager.mode, 'idle')
        self.assertEqual(self.manager.entries()[0]['state'], 'failed')

    def test_topics_exclude_robot_control(self):
        self.assertIn('/tf_static', CORE_TOPICS)
        self.assertIn('/LIDAR/POINTS2', LIDAR_TOPICS)
        self.assertFalse(any('cmd_vel' in t or 'command' in t for t in CORE_TOPICS + LIDAR_TOPICS))
        self.assertNotIn('/LIDAR/POINTS', RECORD_LIDAR_TOPICS)
        self.assertNotIn('/LIDAR/POINTS2', RECORD_LIDAR_TOPICS)
        self.assertIn('/IMU', CORE_TOPICS)

    def test_motion_telemetry_replays_without_starting_receiver_or_tf(self):
        available = {name: 10 for name in SENSOR_TOPICS}
        topics, remaps = playback_topics(available, True)
        self.assertTrue(set(MOTION_TOPICS).issubset(topics))
        self.assertTrue(set(MOTION_TOPICS).issubset(CORE_TOPICS))
        self.assertFalse(any(name in remaps for name in MOTION_TOPICS))
        old_bag = {name: n for name, n in available.items() if name not in MOTION_TOPICS}
        topics, _ = playback_topics(old_bag, True)
        self.assertFalse(set(MOTION_TOPICS).intersection(topics))

    def test_telemetry_failure_does_not_stop_other_recording_inputs(self):
        self.manager.active = self.bag()
        self.manager.mode = 'record'
        record, telemetry = Mock(), Mock()
        record.poll.return_value = None
        telemetry.poll.return_value = 1
        self.manager.children.update(record=record, motion_status=telemetry)
        with patch('bag_manager.shutil.disk_usage', return_value=Mock(free=10*1024**3)):
            state = self.manager.tick()
        self.assertEqual(state['mode'], 'record')
        self.assertIn('运控状态接收退出', state['message'])
        with patch.object(self.manager, '_stop_child') as stop:
            self.manager.finish_record()
        self.assertEqual([c.args[0] for c in stop.call_args_list], ['motion_status', 'record'])
        self.manager.children.clear()

    def test_map_and_old_replays_follow_bag_through_trash_restore_purge(self):
        bag = self.bag()
        (bag / 'maps').mkdir()
        (bag / 'maps/superlio_map.pcd').write_bytes(b'owned map')
        old = self.manager.runtime / 'replay_old'
        old.mkdir()
        (old / 'replay.json').write_text(json.dumps({'bag': 'test'}))
        (old / 'navmesh.json').write_text('{}')
        other = self.manager.runtime / 'replay_unrelated'
        other.mkdir()
        (other / 'replay.json').write_text(json.dumps({'bag': 'another'}))
        self.manager.move_to_trash('test')
        self.assertFalse(old.exists())
        trashed = self.manager.trash / 'test'
        self.assertTrue((trashed / 'maps/superlio_map.pcd').exists())
        self.assertTrue((trashed / 'replays/replay_old/navmesh.json').exists())
        self.manager.restore('test')
        self.assertEqual((bag / 'maps/superlio_map.pcd').read_bytes(), b'owned map')
        self.manager.move_to_trash('test')
        self.manager.purge('test')
        self.assertFalse(trashed.exists())
        self.assertTrue(other.exists())

    def test_restore_stops_replay_before_starting_live_and_can_retry(self):
        self.manager.mode = 'finished'
        self.manager.rebuild = True
        self.manager.live_suspended.touch()
        def start():
            self.assertEqual(self.manager.mode, 'idle')
            self.assertFalse(self.manager.rebuild)
            self.assertFalse(self.manager.live_suspended.exists())
            raise RuntimeError('sensor unavailable')
        with patch.object(self.manager, '_start_live', side_effect=start):
            with self.assertRaisesRegex(RuntimeError, 'sensor unavailable'):
                self.manager.restore_live()
        with patch.object(self.manager, '_start_live') as start:
            self.manager.restore_live()
            start.assert_called_once()

    def test_failed_replay_restores_previously_open_live_rviz(self):
        self.bag()
        with patch.object(self.manager, '_helper'), \
                patch.object(self.manager, '_suspend_live_rviz', return_value=True), \
                patch.object(self.manager, '_start_live') as restore:
            # No paper.rviz: startup fails after the live window is suspended.
            with self.assertRaises(FileNotFoundError):
                self.manager.play('test')
            restore.assert_called_once_with(rviz_only=True)
        self.assertEqual(self.manager.mode, 'idle')
        self.assertFalse(self.manager.live_suspended.exists())

    def test_restore_cannot_interrupt_recording(self):
        self.manager.mode = 'record'
        try:
            with patch.object(self.manager, '_start_live') as start:
                with self.assertRaises(RuntimeError):
                    self.manager.restore_live()
                start.assert_not_called()
        finally:
            self.manager.mode = 'idle'

    def test_rebuild_suspends_gpu_services_but_preserves_lio_and_settings(self):
        with patch('bag_manager.subprocess.run', return_value=Mock(returncode=0)) as run, \
                patch.object(self.manager, '_helper', return_value='{"max_slope_deg": 12.0}'):
            self.assertTrue(self.manager._suspend_live_mapping())
        doc = json.loads(self.manager.mapping_suspended.read_text())
        self.assertEqual(doc['settings'], {'max_slope_deg': 12.0})
        stopped = [c.args[0][-1] for c in run.call_args_list if 'stop' in c.args[0]]
        self.assertEqual(stopped, ['se2-terrain-check', 'nvblox-d435i-shadow'])
        self.assertNotIn('nvblox-lio', stopped)

    def test_gpu_failure_is_explained_and_persisted(self):
        self.manager.mode = 'play'
        self.manager.session = self.root/'replay'
        self.manager.session.mkdir()
        (self.manager.session/'nvblox.log').write_text('CUDA error: out of memory')
        dead = Mock()
        dead.poll.return_value = 139
        self.manager.children['nvblox'] = dead
        with patch.object(self.manager, '_stop_child'):
            result = self.manager.tick()
        self.manager.children.clear()
        self.assertEqual(result['mode'], 'idle')
        self.assertIn('GPU 内存不足', result['message'])
        self.assertEqual(json.loads((self.manager.session/'failure.json').read_text())['reason'], 'GPU 内存不足')

    def test_rebuild_plays_sensors_and_remaps_old_results(self):
        topics, remaps = playback_topics({t: 5 for t in CORE_TOPICS + LIDAR_TOPICS}, True)
        self.assertNotIn('/nvblox_node/mesh', topics)
        self.assertNotIn('/se2_navmesh/local_result', topics)
        self.assertIn('/camera/d435i/depth/image_rect_raw', topics)
        for topic in REFERENCE_TOPICS:
            self.assertIn(topic + ':=/bag_reference' + topic, remaps)

    def test_rebuild_rejects_missing_pose_and_intrinsics(self):
        with self.assertRaisesRegex(ValueError, '缺少重建输入'):
            playback_topics({'/camera/d435i/depth/image_rect_raw': 20}, True)

    def test_lidar_replay_needs_no_camera_and_fusion_requires_both(self):
        available = {t: 5 for t in SENSOR_TOPICS if t not in (DEPTH, INFO)}
        topics, _ = playback_topics(available, True, 'lidar')
        self.assertIn(LIDAR, topics)
        self.assertNotIn(DEPTH, topics)
        with self.assertRaisesRegex(ValueError, DEPTH):
            playback_topics(available, True, 'fusion')
        available.update({DEPTH: 5, INFO: 5})
        available.pop(LIDAR)
        playback_topics(available, True, 'depth')
        with self.assertRaisesRegex(ValueError, LIDAR):
            playback_topics(available, True, 'fusion')

    def test_missing_lidar_inputs_leave_live_services_alone(self):
        self.bag()
        with patch.object(self.manager, '_suspend_live_rviz') as rviz, \
                patch.object(self.manager, '_suspend_live_mapping') as mapping:
            with self.assertRaisesRegex(ValueError, '缺少重建输入'):
                self.manager.play('test', rebuild=True, sensor_mode='lidar')
            rviz.assert_not_called()
            mapping.assert_not_called()

    def test_lidar_adapter_rejects_changed_installed_library(self):
        variants = self.root / 'terrain_variants'
        for name in ('fast_lidar/libfast_lidar.so', 'deskew/build/terrain_lidar_deskew',
                     'fastdds_large_shm.xml', 'fast_lidar/installed_library.sha256'):
            path = variants / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('fixture')
        (variants / 'deskew/build/terrain_lidar_deskew').chmod(0o700)
        installed = self.root / 'installed.so'
        installed.write_bytes(b'installed nvBlox test library')
        (variants / 'fast_lidar/installed_library.sha256').write_text(hashlib.sha256(installed.read_bytes()).hexdigest())
        evidence = prepare_lidar(self.root, installed)
        self.assertFalse(evidence['restore_scan_time'])
        self.assertFalse(evidence['multi_lidar'])
        self.assertNotIn('LD_PRELOAD', self.manager.env(True))
        self.assertEqual(self.manager.env(True)['FASTRTPS_DEFAULT_PROFILES_FILE'], str(variants / 'fastdds_large_shm.xml'))
        installed.write_bytes(b'updated library')
        with self.assertRaisesRegex(RuntimeError, '安装库已变化'):
            prepare_lidar(self.root, installed)

    def test_deskew_failure_stops_owned_replay(self):
        self.manager.mode, self.manager.rebuild = 'play', True
        self.manager.session = self.root / 'replay'
        self.manager.session.mkdir()
        dead = Mock()
        dead.poll.return_value = 1
        self.manager.children['deskew'] = dead
        with patch.object(self.manager, '_stop_child') as stop:
            result = self.manager.tick()
            self.assertIn('deskew', [call.args[0] for call in stop.call_args_list])
        self.manager.children.clear()
        self.assertEqual(result['mode'], 'idle')
        self.assertIn('deskew', result['message'])

    def test_replay_exports_never_overwrite_live_files(self):
        (self.root / 'config').mkdir()
        (self.root / 'config/m20_nvblox_shadow.yaml').write_text(yaml.safe_dump(
            {'nvblox_node': {'ros__parameters': {'use_lidar': True, 'voxel_size': .15}}}))
        self.manager.terrain.mkdir()
        live = self.manager.terrain / 'paper_config.yaml'
        live.write_text(yaml.safe_dump({'se2_terrain_check': {'ros__parameters': {'max_step': .03}}}))
        before = live.read_bytes()
        bag = self.bag()
        (bag/'nvblox_config.yaml').write_text(yaml.safe_dump({'nvblox_node': {'ros__parameters': {
            'layer_visualization_exclusion_height_m': 2.0}}}))
        session = self.manager.runtime / 'run'
        session.mkdir()
        mapping, tuning = write_configs(self.root, self.manager.terrain, bag, session, {'max_step': .07})
        params = yaml.safe_load(tuning.read_text())['se2_terrain_check']['ros__parameters']
        self.assertEqual(params['max_step'], .07)
        self.assertTrue(params['use_sim_time'])
        for key in ('output_json', 'history_path'):
            self.assertEqual(Path(params[key]).parent, session)
        self.assertEqual(live.read_bytes(), before)
        nvblox = yaml.safe_load(mapping.read_text())['nvblox_node']['ros__parameters']
        self.assertFalse(nvblox['use_lidar'])
        self.assertTrue(nvblox['use_depth'])
        self.assertEqual(nvblox['voxel_size'], .05)
        self.assertEqual(nvblox['layer_visualization_exclusion_height_m'], 0.)
        original = (bag/'nvblox_config.yaml').read_bytes()
        for mode in ('lidar', 'fusion'):
            mapping, tuning = write_configs(self.root, self.manager.terrain, bag, session, {'max_step': .07}, mode)
            params = yaml.safe_load(mapping.read_text())['nvblox_node']['ros__parameters']
            self.assertTrue(params['use_lidar'])
            self.assertEqual(params['use_depth'], mode == 'fusion')
            self.assertEqual(params['voxel_size'], .07)
            self.assertEqual(yaml.safe_load(tuning.read_text())['se2_terrain_check']['ros__parameters']['max_step'], .07)
        self.assertEqual(live.read_bytes(), before)
        self.assertEqual((bag/'nvblox_config.yaml').read_bytes(), original)

        # An old bag's flat-ground snapshot must not undo the selected stair
        # default; dimensions and explicit per-replay tuning still survive.
        selected = initial_settings({'max_step': .03, 'max_slope_deg': 15., 'width': .61},
                                    None, STAIRS_PROFILE)
        _, tuning = write_configs(self.root, self.manager.terrain, bag, session,
                                   selected, 'lidar', STAIRS_PROFILE)
        cfg = yaml.safe_load(tuning.read_text())['se2_terrain_check']['ros__parameters']
        self.assertEqual((cfg['max_step'], cfg['max_slope_deg'], cfg['width']), (.2, 40., .61))
        self.assertEqual(cfg['tile_cells'], 8)
        self.assertTrue(cfg['surface_normal_filter'] and cfg['stair_riser_filter'])
        explicit = initial_settings({}, {'max_step': .12}, STAIRS_PROFILE)
        self.assertEqual(explicit['max_step'], .12)
        self.assertEqual(live.read_bytes(), before)
        self.assertEqual((bag/'nvblox_config.yaml').read_bytes(), original)


if __name__ == '__main__':
    unittest.main()
