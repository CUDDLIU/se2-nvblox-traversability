"""Adapter lifecycle tests using lightweight ROS message/executor substitutes.

Run on a development machine without ROS or on Jetson; geometry is intentionally
stubbed here so these tests exercise reception, generation and publication state.
"""
import copy
import importlib.util
import json
import sys
import tempfile
import threading
import time
import types
import unittest
from dataclasses import dataclass
from pathlib import Path
from unittest.mock import patch

import numpy as np


class Message:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class Marker:
    ADD, DELETE, DELETEALL, CUBE, POINTS, LINE_LIST, TRIANGLE_LIST = 0, 2, 3, 1, 8, 5, 11
    CUBE_LIST = 6

    def __init__(self):
        self.header = Message(frame_id='', stamp=None)
        self.pose = Message(orientation=Message(w=0), position=Message(x=0, y=0, z=0))
        self.scale = Message(x=0, y=0, z=0)
        self.color = Message(r=0, g=0, b=0, a=0)
        self.points = []
        self.action = self.ADD
        self.ns = ''
        self.id = 0


class MarkerArray:
    def __init__(self, markers=None):
        self.markers = markers or []


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(copy.deepcopy(message))


@dataclass
class Config:
    yaw_bins: int = 4
    resolution: float = .1
    vertical_resolution: float = .01
    slab_cells: int = 80


def load_adapter(filename='paper_node.py', extra_modules=None):
    modules = {}
    def add(name, **entries):
        module = types.ModuleType(name)
        module.__dict__.update(entries)
        modules[name] = module
    add('rclpy')
    add('rclpy.node', Node=object)
    add('rclpy.executors', SingleThreadedExecutor=object)
    add('rclpy.qos', DurabilityPolicy=Message(TRANSIENT_LOCAL=1),
        ReliabilityPolicy=Message(RELIABLE=1, BEST_EFFORT=2), QoSProfile=Message)
    for package in ('geometry_msgs', 'nav_msgs', 'nvblox_msgs', 'rcl_interfaces',
                    'std_msgs', 'visualization_msgs'):
        add(package)
    add('geometry_msgs.msg', Point=Message)
    add('nav_msgs.msg', Odometry=Message)
    add('nvblox_msgs.msg', Mesh=Message)
    add('rcl_interfaces.msg', SetParametersResult=Message)
    add('std_msgs.msg', String=Message)
    add('visualization_msgs.msg', Marker=Marker, MarkerArray=MarkerArray)
    add('paper_pipeline', Config=Config, Engine=object)
    modules.update(extra_modules or {})
    spec = importlib.util.spec_from_file_location('_paper_node_test', Path(__file__).with_name(filename))
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


adapter = load_adapter()


class Engine:
    def __init__(self):
        self.pending = set()
        self.results = {}
        self.generation = self.revision = 0
        self.index = Message(blocks={})
        self.updates = []
        self.fail_snapshot = False

    def update(self, blocks, clear=False):
        self.updates.append((set(blocks), clear))
        if clear:
            self.generation += 1
            self.pending.clear()
            self.index.blocks.clear()
        if blocks:
            self.index.blocks.update(blocks)
            self.pending.update((0, 1))
        if blocks or clear:
            self.revision += 1
        return {}

    def process(self, max_slabs):
        if self.pending:
            self.pending.remove(min(self.pending))
        return {}

    def snapshot(self):
        if self.fail_snapshot:
            raise RuntimeError('graph failure')
        return dict(polygons=[dict(vertices=[[0, 0, 0], [1, 0, 0], [0, 1, 0]], yaw_mask=15)],
                    graph={'nodes': [], 'edges': []}, generation=self.generation, revision=self.revision)


def make_node(output_json=''):
    node = adapter.PaperNode.__new__(adapter.PaperNode)
    node.config = Config()
    node.options = dict(max_inbox_blocks=2, input_timeout_s=3., output_json=output_json,
                        max_slabs_per_tick=1, publish_rejected_cells=True,
                        publish_debug_intervals=False)
    node.frame = 'world'
    node.engine = Engine()
    node.lock = threading.RLock()
    node.inbox = {}
    node.reset_pending = False
    node.epoch = node.input_sequence = node.frame_rejects = node.mesh_messages = 0
    node.mesh_received = node.odom_received = time.monotonic()
    node.mesh_size = node.odom_position = None
    node.input_error = node.compute_error = node.export_error = ''
    node.published_revision = node.published_epoch = node.last_document = None
    node.last_rendered = node.last_visual_history = node.last_validity = None
    node.last_metrics = {}
    node.last_log = 0.
    for name in ('markers', 'completed', 'solids', 'free', 'map', 'status'):
        setattr(node, name+'_pub', Publisher())
    node.get_clock = lambda: Message(now=lambda: Message(to_msg=lambda: None, nanoseconds=10**9))
    node.get_logger = lambda: Message(info=lambda _: None, error=lambda _: None)
    return node


def mesh(key=0, clear=False, frame='world'):
    return Message(header=Message(frame_id=frame), clear=clear, block_size_m=1.,
                   block_indices=[Message(x=key, y=0, z=0)],
                   blocks=[Message(vertices=[Message(x=0., y=0., z=0.)], triangles=[0, 0, 0])])


class AdapterTests(unittest.TestCase):
    def test_coalescing_keeps_latest_and_overflow_latches_until_reset(self):
        node = make_node()
        one, two = mesh(), mesh()
        node.receive_mesh(one)
        node.receive_mesh(two)
        self.assertEqual(len(node.inbox), 1)
        self.assertIs(node.inbox[(0, 0, 0)], two.blocks[0])
        node.receive_mesh(mesh(1))
        node.receive_mesh(mesh(2))
        self.assertIn('capacity', node.input_error)
        node.receive_mesh(mesh(5, clear=True))
        self.assertEqual(node.input_error, '')
        self.assertEqual(set(node.inbox), {(5, 0, 0)})
        self.assertTrue(node.reset_pending)

    def test_frozen_cycle_completes_with_new_input_and_marks_history(self):
        node = make_node()
        node.receive_mesh(mesh(0))
        node.tick()
        node.receive_mesh(mesh(1))
        node.tick()
        self.assertEqual(node.engine.updates[1][0], set())
        self.assertEqual(set(node.inbox), {(1, 0, 0)})
        self.assertEqual(node.published_revision, (0, 1))
        document = json.loads(node.map_pub.messages[-1].data)
        self.assertTrue(document['updating'])
        self.assertFalse(document['valid'])
        marker = next(m for m in node.markers_pub.messages[-1].markers if m.ns == 'all_yaw')
        self.assertEqual(marker.color.b, .78)
        completed = next(m for m in node.completed_pub.messages[-1].markers if m.ns == 'all_yaw')
        self.assertEqual(completed.color.g, .85)
        self.assertFalse(any(m.action == Marker.DELETEALL for m in node.markers_pub.messages[-1].markers))
        node.tick()
        node.tick()
        self.assertTrue(json.loads(node.map_pub.messages[-1].data)['valid'])
        marker = next(m for m in node.markers_pub.messages[-1].markers if m.ns == 'all_yaw')
        self.assertEqual(marker.color.g, .85)

    def test_stale_map_recolors_without_deleting_and_recovers(self):
        node = make_node()
        node.tick()
        node.mesh_received = time.monotonic()-20
        node.tick()
        self.assertFalse(json.loads(node.map_pub.messages[-1].data)['valid'])
        self.assertTrue(node.last_visual_history)
        self.assertTrue(all(m.action == (Marker.ADD if m.points else Marker.DELETE)
                            for m in node.markers_pub.messages[-1].markers))
        node.mesh_received = time.monotonic()
        node.tick()
        self.assertFalse(node.last_visual_history)

    def test_export_complete_revision_even_when_new_updates_are_waiting(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'latest.json'
            node = make_node(str(path))
            node.receive_mesh(mesh(0))
            node.tick()
            node.receive_mesh(mesh(1))
            node.tick()
            document = json.loads(path.read_text())
            self.assertEqual(document['revision'], 1)
            self.assertTrue(document['polygons'])
            self.assertTrue(document['updating'])
            self.assertFalse(document['valid'])

    def test_reset_clears_display_and_export_immediately(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'latest.json'
            node = make_node(str(path))
            node.tick()
            self.assertTrue(path.exists())
            node.receive_mesh(mesh(clear=True))
            self.assertFalse(path.exists())
            self.assertIsNone(node.last_document)
            self.assertEqual(node.markers_pub.messages[-1].markers[0].action, Marker.DELETEALL)
            self.assertEqual(node.completed_pub.messages[-1].markers[0].action, Marker.DELETEALL)
            self.assertFalse(json.loads(node.map_pub.messages[-1].data)['valid'])
            node.export_snapshot('{"old":true}', epoch=0)
            self.assertFalse(path.exists())

    def test_bad_mesh_or_graph_failure_latches_until_full_reset(self):
        node = make_node()
        node.engine.fail_snapshot = True
        node.tick()
        self.assertIn('graph failure', node.compute_error)
        node.engine.fail_snapshot = False
        node.tick()
        self.assertTrue(node.compute_error)
        node.receive_mesh(mesh(clear=True))
        node.tick()
        node.tick()
        self.assertEqual(node.compute_error, '')

    def test_portable_json_and_polygon_fill(self):
        value = adapter.json_value({'ceiling': float('inf'), 'mask': np.uint64(15),
                                    'vertices': np.array([[1, 2, 3]])})
        self.assertEqual(json.loads(json.dumps(value, allow_nan=False))['ceiling'], None)
        self.assertEqual(len(adapter.polygon_triangles([[0, 0, 0], [1, 0, 0],
                                                       [1, 1, 0], [0, 1, 0]])), 6)

    def test_empty_triangle_classes_delete_previous_geometry_without_rviz_error(self):
        n = make_node()
        filled = n.render_geometry(n.engine.snapshot())
        green = next(m for m in filled.markers if m.ns == 'all_yaw')
        self.assertEqual(green.action, Marker.ADD)
        self.assertEqual(len(green.points), 3)
        amber = next(m for m in filled.markers if m.ns == 'restricted_yaw')
        self.assertEqual(amber.action, Marker.DELETE)
        empty = n.render_geometry(dict(polygons=[]))
        self.assertTrue(all(m.action == Marker.DELETE for m in empty.markers))


if __name__ == '__main__':
    unittest.main()
