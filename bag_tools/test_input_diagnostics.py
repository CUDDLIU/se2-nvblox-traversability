import json
import os
from pathlib import Path
import struct
import tempfile
import unittest

from input_diagnostics import JsonLog, Recorder, Stream, bound_bag, classify, header_stamp, process_start, lidar_ring_counts


class DiagnosticsTests(unittest.TestCase):
    def test_ring_groups_endian_padding_and_partial_sensor_loss(self):
        from types import SimpleNamespace as NS
        for order,big in (('<',False),('>',True)):
            data=bytearray(64)
            for row,values in enumerate(((0,95),(96,191))):
                for col,ring in enumerate(values):struct.pack_into(order+'H',data,row*32+col*12+3,ring)
            msg=NS(header=NS(frame_id='lidar_link'),fields=[NS(name='ring',datatype=4,count=1,offset=3)],
                width=2,height=2,point_step=12,row_step=32,data=data,is_bigendian=big)
            self.assertEqual(lidar_ring_counts(msg),dict(front=2,rear=2,unknown=0))
            struct.pack_into(order+'H',data,32+3,192)
            self.assertEqual(lidar_ring_counts(msg),dict(front=2,rear=1,unknown=1))
            msg.data=data[:10]
            with self.assertRaises(ValueError):lidar_ring_counts(msg)
        streams={'lidar':dict(receive_age_s=.1,hz=10),
                 'lidar_front':dict(receive_age_s=.1,hz=10),
                 'lidar_rear':dict(receive_age_s=3.,hz=0)}
        self.assertIn('后雷达分组超过 2 秒没有点，总点云仍在更新',classify(streams,{},False,{})['problems'])

    def test_cdr_and_stopped_rate(self):
        for order, flag in [('<', 1), ('>', 0)]:
            self.assertEqual(header_stamp(bytes([0, flag, 0, 0])+struct.pack(order+'iI', 10, 500000000)), 10.5)
        with self.assertRaises(ValueError):
            header_stamp(b'')
        stream = Stream()
        for stamp, now in [(100, 1), (100, 1.1), (99, 1.2)]:
            stream.receive(stamp, now)
        self.assertEqual(stream.sample(2, 101, 1)['hz'], 3)
        stopped = stream.sample(5, 104, 3)
        self.assertEqual(stopped['hz'], 0)
        self.assertAlmostEqual(stopped['receive_age_s'], 3.8)
        self.assertEqual(stopped['timestamp_rollbacks'], 1)
        self.assertEqual(stopped['duplicate_stamps'], 1)

    def test_depth_failure_and_expected_replay_pause(self):
        streams = {k:dict(receive_age_s=.1, hz=10) for k in ('lidar','imu','odom','pose_tf','depth','mesh')}
        services = {k:dict(ActiveState='active') for k in ('nvblox-lio.service','nvblox-d435i-shadow.service')}
        streams['depth']['receive_age_s'] = 70
        result = classify(streams, services, False, {}, True)
        self.assertEqual(result['problems'], ['D435i 深度无新消息（同期有 USB/相机错误）'])
        self.assertEqual(classify(streams, services, True, {})['problems'], [])
        streams['lidar']['receive_age_s'] = 3
        result = classify(streams, services, True, {'IpReasmFails': 100})
        self.assertIn('雷达入口无新消息', result['problems'])
        self.assertTrue(any('分片重组失败' in p for p in result['problems']))

    def test_rotation_is_bounded_and_valid_json(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'log.jsonl'
            logger = JsonLog(path, limit=100, backups=2)
            for i in range(100):
                logger.append(dict(index=i, message='检测日志'))
            self.assertEqual(len(list(Path(folder).iterdir())), 3)
            for file in Path(folder).iterdir():
                for line in file.read_text().splitlines():
                    self.assertIn('index', json.loads(line))
            self.assertEqual(json.loads(path.read_text().splitlines()[-1])['index'], 99)

    def test_bag_binding_preroll_and_lifecycle(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            runtime = root/'bag_tools/runtime'
            runtime.mkdir(parents=True)
            bag = root/'bags/test'
            bag.mkdir(parents=True)
            manifest = bag/'session.json'
            manifest.write_text(json.dumps(dict(state='recording')))
            pointer = runtime/'diagnostic_target.json'
            doc = dict(path=str(bag), pid=os.getpid(), process_start=process_start(os.getpid()))
            logger = Recorder(root)
            logger.emit('samples', dict(summary='before'))
            pointer.write_text(json.dumps(doc))
            self.assertEqual(bound_bag(root), bag)
            logger.emit('samples', dict(summary='during'))
            self.assertEqual(json.loads((bag/'diagnostics/pre_record.jsonl').read_text())['summary'], 'before')
            captured = (bag/'diagnostics/samples.jsonl').read_text()
            manifest.write_text(json.dumps(dict(state='ready')))
            logger.emit('samples', dict(summary='after'))
            self.assertEqual((bag/'diagnostics/samples.jsonl').read_text(), captured)
            manifest.write_text(json.dumps(dict(state='recording')))
            doc['process_start'] = 'stale-process'
            pointer.write_text(json.dumps(doc))
            self.assertIsNone(bound_bag(root))
            doc['process_start'] = process_start(os.getpid())
            pointer.write_text(json.dumps(doc))
            bag.rename(root/'bags/.deleted')
            logger.emit('samples', dict(summary='deleted'))
            self.assertFalse(bag.exists())
            doc['path'] = str(root/'bags/.deleted')
            pointer.write_text(json.dumps(doc))
            self.assertIsNone(bound_bag(root))


if __name__ == '__main__':
    unittest.main()
