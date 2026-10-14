"""Recorded-cloud equivalence and stage timings; does not open a ROS node."""
import argparse,json,time
from pathlib import Path
import numpy as np
from core import Core
ap=argparse.ArgumentParser();ap.add_argument('samples',type=Path);ap.add_argument('output',type=Path);args=ap.parse_args()
rows=[];core=Core()
try:
 with np.load(args.samples) as inputs:
  for name in inputs.files:
   results=[];timings=[];velocities=[]
   for reference in (True,False):
    samples=[]
    for i in range(5):
     points=core.cloud(inputs[name],reference=reference)
     if i:samples.append(core.cloud_timing())
    results.append(points);timings.append({k:float(np.median([s[k] for s in samples])) for k in samples[0]})
    core.goal();v=core.plan((1.5,0,0),(5,0,0));velocities.append([v.x,v.y,v.w,v.id,v.status])
   assert np.array_equal(*results) and velocities[0]==velocities[1],name
   rows.append(dict(frame=name,exact_cloud_equal=True,planner_equal=True,reference=timings[0],optimized=timings[1]))
finally:core.close()
args.output.write_text(json.dumps(rows,indent=2)+'\n');print(json.dumps(rows))
