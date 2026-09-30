"""Read-only audit of physical sensor availability in each raw bag cloud."""
import argparse
import json
from pathlib import Path
import sqlite3
import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2


def main():
    p = argparse.ArgumentParser();p.add_argument('bag', type=Path)
    args = p.parse_args();data = args.bag / 'data'
    meta = yaml.safe_load((data / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    begin = meta['starting_time']['nanoseconds_since_epoch'] * 1e-9
    rows = []
    for name in meta['relative_file_paths']:
        with sqlite3.connect('file:' + str(data / name) + '?mode=ro', uri=True) as c:
            tid = c.execute('SELECT id FROM topics WHERE name=?', ('/LIDAR/POINTS_NX',)).fetchone()
            if tid is None:continue
            for timestamp, raw in c.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',tid):
                msg = deserialize_message(raw, PointCloud2)
                f = {v.name:v for v in msg.fields}
                if msg.point_step != 26 or msg.is_bigendian:raise ValueError('unverified cloud layout')
                names = ['x','y','z','ring']
                dtype = np.dtype(dict(names=names, formats=['<f4']*3+['<u2'],
                    offsets=[f[n].offset for n in names],itemsize=msg.point_step))
                a = np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,
                    strides=(msg.row_step,msg.point_step)).ravel()
                xyz = np.column_stack([a[n] for n in names[:3]])
                valid = np.isfinite(xyz).all(axis=1)
                ring = a['ring']
                groups = []
                for lo,hi,ox in ((0,96,.32028),(96,192,-.32028)):
                    mask = valid & (ring >= lo) & (ring < hi)
                    v = xyz[mask] - np.array([ox,0.,-.013])
                    groups.append(dict(points=int(np.count_nonzero(mask)),
                        within4=int(np.count_nonzero(np.linalg.norm(v,axis=1)<4))))
                rows.append(dict(elapsed=timestamp*1e-9-begin,points=len(a),groups=groups,
                    unknown_rings=int(np.count_nonzero(ring>=192))))
        print(name, len(rows), flush=True)
    summary = dict(frames=len(rows),unknown_ring_points=sum(v['unknown_rings'] for v in rows),
        front_frames=sum(v['groups'][0]['points']>0 for v in rows),
        rear_frames=sum(v['groups'][1]['points']>0 for v in rows),
        front_within4_frames=sum(v['groups'][0]['within4']>0 for v in rows),
        rear_within4_frames=sum(v['groups'][1]['within4']>0 for v in rows),
        rows=rows)
    out=args.bag/'diagnostics/sensor_alignment/ring_availability.json'
    out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(summary,indent=2))
    print(json.dumps({k:v for k,v in summary.items() if k!='rows'}))


if __name__=='__main__':main()
