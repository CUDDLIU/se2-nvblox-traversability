#!/usr/bin/env python3
import json
from pathlib import Path
import signal
import sys
import time
import numpy as np
import rclpy
from std_msgs.msg import Float64MultiArray, String
from scan_planner_msgs.msg import Bspline
from nav_msgs.msg import Path as RosPath
from rclpy.qos import qos_profile_sensor_data

def main():
    out=Path(sys.argv[1]);rclpy.init();node=rclpy.create_node('se2_scan_metrics')
    data={'perception':[],'control':[],'verification':[],'optimization':[],'candidates':[],'verified':[],'status':[]}
    for name,topic in [('perception','timing'),('control','control_timing'),('verification','optimizer_timing'),('optimization','planning/replan_timing')]:
        node.create_subscription(Float64MultiArray,'/se2_scan/'+topic,lambda m,n=name:data[n].append(list(m.data)),qos_profile_sensor_data)
    for name,topic in [('candidates','planning/bspline'),('verified','verified_bspline')]:
        node.create_subscription(Bspline,'/se2_scan/'+topic,lambda m,n=name:data[n].append([time.monotonic(),m.traj_id]),100)
    node.create_subscription(String,'/se2_scan/route_status',lambda m:data['status'].append(m.data),100)
    running=[True]
    signal.signal(signal.SIGINT,lambda *_:running.__setitem__(0,False));signal.signal(signal.SIGTERM,lambda *_:running.__setitem__(0,False))
    def stats(a):
        return dict(count=len(a),p50=float(np.percentile(a,50)),p95=float(np.percentile(a,95)),p99=float(np.percentile(a,99)),max=float(np.max(a))) if len(a) else None
    def save():
        report={'units':'milliseconds unless otherwise stated','counts':{k:len(v) for k,v in data.items()}}
        p=np.asarray(data['perception']);c=np.asarray(data['control']);v=np.asarray(data['verification']);o=np.asarray(data['optimization'])
        if len(p):
            report.update(ingress_to_decision_ms=stats(p[:,6]),preprocess_including_pose_wait_ms=stats(p[:,3]),map_update_ms=stats(p[:,4]),collision_check_ms=stats(p[:,5]),sensor_age_ms=stats(p[:,7]),replaced_frames=int(p[-1,10]),dropped_frames=int(p[-1,11]))
        if len(p) and p.shape[1]>13:
            active=p[p[:,13]>0]
            report['active_trajectory_ingress_to_decision_ms']=stats(active[:,6])
        if len(c)>1:
            periods=c[1:,1];report.update(control_period_ms=stats(periods),control_mean_hz=1000/float(periods.mean()),periods_over_50_ms=int(sum(periods>50)),nonzero_commands=int(np.any(np.abs(c[:,2:5])>1e-6,axis=1).sum()))
            if c.shape[1]>=7:report.update(control_compute_ms=stats(c[:,5]),control_publish_ms=stats(c[:,6]))
        if len(v):
            report.update(candidate_verification_ms=stats(v[:,1]),geometrically_valid_candidates=int(v[:,2].sum()))
            if v.shape[1]>=7:
                report.update(accepted_candidates=int(v[:,5].sum()),published_splines=int(sum(v[:,6]==1)),
                              expired_or_rejected_candidates=int(sum(v[:,5]==0)))
        if len(o):report.update(optimizer_ms=stats(o[:,0]),optimizer_successes=int(o[:,1].sum()))
        if len(o) and len(v) and o.shape[1]>=5 and v.shape[1]>=5:
            source={round(row[0],6):row[1] for row in p}
            planned={int(row[4]):row for row in o if row[1]>0}
            e2e=[]
            for row in v:
                plan=planned.get(int(row[4]))
                # Only an actually published SCAN spline qualifies. A slow
                # discarded result or a different fallback maneuver does not.
                if len(row)>=7 and row[5]>0 and row[6]==1 and plan is not None:
                    ingress=source.get(round(plan[2],6))
                    if ingress is not None:e2e.append((row[3]-ingress)*1000)
            report['cloud_to_verified_trajectory_ms']=stats(e2e)
        (out/'metrics.json').write_text(json.dumps(report,indent=2));(out/'samples.json').write_text(json.dumps(data))
    last=time.monotonic()
    try:
        while running[0]:
            rclpy.spin_once(node,timeout_sec=.1)
            if time.monotonic()-last>10:save();last=time.monotonic()
    finally:save();node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
