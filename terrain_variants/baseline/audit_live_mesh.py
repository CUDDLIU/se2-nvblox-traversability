"""Bounded read-only audit; raw Mesh transport and current checker results."""
import argparse
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from nvblox_msgs.msg import Mesh
from std_msgs.msg import String

from mesh_wire import decode_mesh,mesh_summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=15.)
    parser.add_argument('--capture', default='/tmp/se2_mesh_wire.bin')
    parser.add_argument('--journal',help='Append event JSONL and flush every second for reboot recovery')
    parser.add_argument('--verify-wire-samples', type=int, default=0,
                        help='Optional expensive independent ROS decode comparison; zero for latency measurement')
    args = parser.parse_args()
    rclpy.init()
    node = Node('se2_bounded_latency_audit')
    rows, arrivals, raw_samples, outputs = [], [], [], []
    largest = b''
    mesh_samples=[]
    journal=open(args.journal,'a',buffering=65536) if args.journal else None
    last_flush=time.monotonic()
    def record(kind,row):
        nonlocal last_flush
        if journal is not None:
            journal.write(json.dumps(dict(kind=kind,received_at=time.monotonic(),data=row),separators=(',',':'))+'\n')
            if time.monotonic()-last_flush>=1.:
                journal.flush();last_flush=time.monotonic()

    def mesh(raw):
        nonlocal largest
        arrivals.append(time.monotonic())
        mesh_samples.append(dict(received_at=arrivals[-1],**mesh_summary(raw)))
        record('mesh',mesh_samples[-1])
        if len(raw) > len(largest):
            largest = raw
        if len(raw_samples) < args.verify_wire_samples and len(raw) > 100000:
            start = time.monotonic()
            decoded = decode_mesh(raw)
            wire_ms = (time.monotonic()-start)*1000
            start = time.monotonic()
            standard = deserialize_message(raw, Mesh)
            deserialize_ms = (time.monotonic()-start)*1000
            assert decoded.header.frame_id == standard.header.frame_id
            assert decoded.clear == standard.clear
            assert decoded.block_size_m == standard.block_size_m
            assert len(decoded.blocks) == len(standard.blocks)
            for fast, normal in zip(decoded.blocks, standard.blocks):
                np.testing.assert_array_equal(fast.vertices_array,
                    np.array([(p.x,p.y,p.z) for p in normal.vertices], dtype=np.float32).reshape(-1,3))
                np.testing.assert_array_equal(fast.triangles_array,
                    np.array(normal.triangles).reshape(-1,3))
            raw_samples.append(dict(bytes=len(raw), blocks=len(decoded.blocks),
                                    wire_ms=wire_ms, deserialize_ms=deserialize_ms))

    def status(msg):
        row=json.loads(msg.data)
        if 'metrics' in row:
            row['audit_received_at']=time.monotonic()
            rows.append(row)
            record('status',row)

    node.create_subscription(Mesh, '/nvblox_node/mesh', mesh, 100, raw=True)
    node.create_subscription(String, '/se2_terrain/status', status, 10)
    def local(msg):
        row=json.loads(msg.data)
        if row.get('event')=='replace':
            # Empty invalidation/status heartbeats are deliberately excluded.
            outputs.append({k:v for k,v in row.items() if k!='updated_slabs'} | dict(
                received_at=time.monotonic(),slabs=len(row['updated_slabs']),
                slab_timings=[{k:s[k] for k in ('key','latency_ms','input_sequence')} | dict(cells=len(s['cells']))
                              for s in row['updated_slabs']],
                cells=sum(len(s['cells']) for s in row['updated_slabs'])))
            record('output',outputs[-1])
    node.create_subscription(String, '/se2_navmesh/local_result', local, 100)
    start=time.monotonic()
    try:
        while time.monotonic()-start < args.seconds:
            rclpy.spin_once(node, timeout_sec=.1)
        if largest:
            Path(args.capture).write_bytes(largest)
        geometry=[r for r in mesh_samples if r['triangles']]
        stamps=sorted(set(r['stamp'] for r in geometry))
        print(json.dumps(dict(seconds=time.monotonic()-start, mesh_messages=len(arrivals),
            mesh_hz=(len(arrivals)-1)/(arrivals[-1]-arrivals[0]) if len(arrivals)>1 else 0,
            geometry_messages=len(geometry),unique_geometry_stamps=len(stamps),
            geometry_source_hz=(len(stamps)-1)/(stamps[-1]-stamps[0]) if len(stamps)>1 else 0,
            wire_samples=raw_samples, mesh_samples=mesh_samples,status=rows, outputs=outputs), separators=(',', ':')))
    finally:
        if journal is not None:journal.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
