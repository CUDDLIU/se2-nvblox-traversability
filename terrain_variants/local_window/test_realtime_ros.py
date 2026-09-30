import json
import queue
import time
import unittest
from unittest.mock import patch

from local_updates import LocalUpdates
from local_display import LocalDisplay
from paper_pipeline import Engine, Config
from test_paper_ros import load_adapter, adapter, make_node, Message, Publisher, mesh

realtime = load_adapter('realtime_node.py', {'paper_node': adapter})


def node():
    n = make_node()
    n.__class__ = realtime.RealtimeNode
    n.engine = Engine()
    n.config = Config()
    n.options.update(compute_budget_ms=1000., global_period_s=1., publish_rejected_cells=False)
    n.provenance = LocalUpdates()
    n.receipts = {}
    n.global_queue = queue.Queue(maxsize=1)
    n.last_global_submit = n.last_status = -float('inf')
    n.local_pub, n.local_markers_pub = Publisher(), Publisher()
    n.display_pub, n.validity_display_pub = Publisher(), Publisher()
    n.display = LocalDisplay()
    n.last_display = -float('inf')
    n.local_events = 0
    n.live_ready=True
    n.config_version=0
    n.admit_turn=0
    n.map_session='test'
    n.history_document=None
    n.last_history_save=-float('inf')
    n.last_local=None
    n.get_parameter=lambda name: Message(value=getattr(n.config,name))
    return n


def delta(key=0, clear=False):
    m = mesh(key, clear=clear)
    m.header.stamp = Message(sec=10, nanosec=123)
    m.blocks[0].vertices = [Message(x=0., y=0., z=0.), Message(x=.3, y=0., z=0.),
                            Message(x=0., y=.3, z=0.)]
    return m


class RealtimeAdapterTests(unittest.TestCase):
    def test_local_publishes_before_global_and_idle_is_not_effective_output(self):
        n = node()
        n.receive_mesh(delta())
        n.tick()
        events = [json.loads(m.data) for m in n.local_pub.messages]
        self.assertEqual([e['event'] for e in events][:2], ['invalidate', 'replace'])
        output = events[1]
        # A local replacement can be emitted while other affected tiles are
        # still queued; complete_input is reserved for a quiet full revision.
        self.assertFalse(output['complete_input'])
        self.assertTrue(output['global_updating'])
        self.assertGreater(output['latency_oldest_ms'], 0.)
        self.assertFalse(n.completed_pub.messages)
        self.assertEqual(n.global_queue.qsize(), 0)
        n.tick()
        self.assertGreaterEqual(n.local_events, 1)

    def test_reset_during_compute_cannot_publish_previous_epoch(self):
        n = node()
        n.receive_mesh(delta())
        original = n.engine.process
        def process(*args, **kwargs):
            result = original(*args, **kwargs)
            n.receive_mesh(delta(1, clear=True))
            return result
        with patch.object(n.engine, 'process', process):
            n.tick()
        events = [json.loads(m.data) for m in n.local_pub.messages]
        self.assertFalse(any(e['event'] == 'replace' for e in events))
        self.assertTrue(any(e['event'] == 'reset' for e in events))
        n.tick()
        self.assertEqual(n.last_local['input_epoch'], 1)

    def test_newer_input_during_compute_is_not_labeled_current(self):
        n = node()
        n.receive_mesh(delta())
        original = n.engine.process
        def process(*args, **kwargs):
            result = original(*args, **kwargs)
            n.receive_mesh(delta(1))
            return result
        with patch.object(n.engine, 'process', process):
            n.tick()
        self.assertFalse(n.last_local['current_input'])
        self.assertFalse(n.last_local['complete_input'])
        self.assertEqual(n.last_local['input_sequence'], 1)

    def test_coalescing_keeps_earliest_wait_and_latest_timestamp(self):
        n = node()
        n.receive_mesh(delta())
        first = n.receipts[(0, 0, 0)][0]
        n.receive_mesh(delta())
        self.assertEqual(n.receipts[(0, 0, 0)][0], first)
        self.assertEqual(n.receipts[(0, 0, 0)][2], 2)

    def test_stale_or_failed_input_emits_invalid_status(self):
        n = node()
        n.compute_error = 'failed'
        n.tick()
        status = json.loads(n.status_pub.messages[-1].data)
        self.assertTrue(status['compute_error'])

    def test_retained_display_survives_pending_and_replaces_deleted_cells(self):
        n = node()
        key = (1, 0, 0)
        full = (1 << n.config.yaw_bins)-1
        slab = dict(key=key, cells=[[16, 0, 1, full], [17, 0, 1, 1]])
        n.display.replace([slab])
        n.display.invalidate([key])
        n.publish_display()
        history = n.display_pub.messages[-1].markers
        current = n.validity_display_pub.messages[-1].markers
        self.assertEqual([len(m.points) for m in history], [1, 1, 0])
        self.assertEqual([len(m.points) for m in current], [0, 0, 2, 0])
        self.assertTrue(all(m.action == adapter.Marker.DELETE for m in current[:2]))
        # A complete retained sample includes unchanged tiles for late RViz.
        n.publish_display()
        self.assertEqual([len(m.points) for m in n.display_pub.messages[-1].markers], [1, 1, 0])
        n.display.replace([dict(key=key, cells=[])])
        n.publish_display()
        self.assertEqual([len(m.points) for m in n.display_pub.messages[-1].markers], [0,0,2])
        self.assertFalse(n.local_pub.messages)
        self.assertEqual(n.local_events, 0)

    def test_reset_removes_retained_display_and_stale_never_claims_current(self):
        n = node()
        n.display.replace([dict(key=[0, 0, 0], cells=[[0, 0, 1, (1 << 40)-1]])])
        n.publish_display(stale=True)
        self.assertEqual([len(m.points) for m in n.validity_display_pub.messages[-1].markers], [0, 0, 1, 0])
        n.clear_outputs_locked('test reset')
        self.assertFalse(n.display.cells)
        self.assertEqual(len(n.display_pub.messages[-1].markers[-1].points),1)


if __name__ == '__main__':
    unittest.main()
