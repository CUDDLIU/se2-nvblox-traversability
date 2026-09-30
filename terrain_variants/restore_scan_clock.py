"""Offline audit of the vendor rolling-window timestamp compression.

The locally archived m20_lidar_timestamp_guard.cpp linearly maps a rolling
~200 ms cloud to 100 ms, anchoring its end. Detect its factor using raw encoder
azimuth progression and the measured 10 Hz rotation, then invert for a
diagnostic copy only. Never modifies raw bags, drivers, TF or original samples.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp


def infer_factor(points,rings,times):
    evidence=[]
    rates=[]
    for sensor in range(2):
        values=[]
        for ring in (24+sensor*96,48+sensor*96,72+sensor*96):
            mask=rings==ring;q=points[mask]-[.32028 if sensor==0 else -.32028,0,-.013]
            ts=times[mask];phi=np.arctan2(q[:,1],q[:,2])
            d=(np.diff(phi)+np.pi)%(2*np.pi)-np.pi;dt=np.diff(ts)
            good=(abs(d)>.001)&(abs(d)<.02)&(dt>1e-6)&(dt<.002)
            values.extend((abs(d[good])/dt[good]/(20*np.pi)).tolist())
        if values:
            quant=np.percentile(values,[10,50,90])
            evidence.append(dict(sensor=sensor,pairs=len(values),rotation_rate_ratio=quant.tolist()))
            rates.extend(values)
    if len(rates)<60:raise ValueError('insufficient angular/timestamp evidence')
    lo,median,hi=np.percentile(rates,[10,50,90])
    if hi-lo>.15:raise ValueError('inconsistent scan clock: broad angular rate distribution')
    if abs(median-1)<.05:factor=1
    elif abs(median-2)<.05:factor=2
    else:raise ValueError('unsupported scan clock factor '+str(median))
    return factor,evidence


def main():
    p=argparse.ArgumentParser();p.add_argument('samples',type=Path);p.add_argument('output',type=Path)
    args=p.parse_args()
    if args.samples.resolve()==args.output.resolve():raise ValueError('output must be a diagnostic copy')
    args.output.mkdir(parents=True,exist_ok=True);rows=[]
    for path in sorted(args.samples.glob('sensors_*.npz')):
        with np.load(path) as z:a={k:z[k] for k in z.files}
        meta=json.loads(path.with_suffix('.json').read_text())
        factor,evidence=infer_factor(a['lidar_body'],a['lidar_rings'],a['lidar_times'])
        # These samples were range filtered; full-cloud runtime must use the
        # unfiltered maximum. The possible endpoint difference is disclosed.
        end=a['lidar_times'].max();restored=end+(a['lidar_times']-end)*factor
        a['lidar_times_restored']=restored
        if factor!=1:
            if restored.min()<a['pose_times'][0] or restored.max()>a['pose_times'][-1]:
                raise ValueError('restored timestamp not bracketed by odometry')
            r=Slerp(a['pose_times'],Rotation.from_quat(a['pose_quaternions']))(restored).as_matrix()
            t=np.column_stack([np.interp(restored,a['pose_times'],a['pose_positions'][:,j]) for j in range(3)])
            corrected=np.einsum('nij,nj->ni',r,a['lidar_body'])+t
            movement=np.linalg.norm(corrected-a['lidar_world'],axis=1)
            a['lidar_world']=corrected
        else:movement=np.zeros(len(restored))
        row=dict(sample=path.name,factor=factor,evidence=evidence,
            end_anchor='maximum timestamp of range-filtered diagnostic sample',
            timestamp_shift_min_s=float((restored-a['lidar_times']).min()),
            world_shift_p50_m=float(np.median(movement)),world_shift_p95_m=float(np.percentile(movement,95)))
        rows.append(row);meta['scan_clock_diagnostic']=row
        np.savez(args.output/path.name,**a)
        (args.output/path.with_suffix('.json').name).write_text(json.dumps(meta,indent=2))
        print(path.name,'factor',factor,'world shift p50/p95',row['world_shift_p50_m'],row['world_shift_p95_m'])
    (args.output/'scan_clock_audit.json').write_text(json.dumps(dict(scope=__doc__,samples=rows),indent=2))


if __name__=='__main__':main()
