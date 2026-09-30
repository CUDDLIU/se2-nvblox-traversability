"""Measure surface removal from actual mapper deltas in the stair ROI."""
import json,sys
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
p=Path(sys.argv[1]);rows=[];snapshots={}
for f in sorted((p/'mesh_trace').glob('mesh_*.npz')):
 a=np.load(f);centers=[];areas=[]
 for i,k in enumerate(a['keys']):
  v=a['v'+str(i)];tri=a['t'+str(i)];pts=v[tri]
  normal=np.cross(pts[:,1]-pts[:,0],pts[:,2]-pts[:,0]);norm=np.linalg.norm(normal,axis=1)
  center=pts.mean(axis=1)
  # Approximate tread surfaces in the descent height range, excluding initial
  # and lower flat floors. No triangulation-density assumption.
  keep=(norm>1e-8)&(np.abs(normal[:,2])>.7*norm)&(center[:,2]<-1.5)&(center[:,2]>-7.)
  centers.append(center[keep]);areas.append(norm[keep]/2)
 c=np.concatenate(centers) if centers else np.empty((0,3));area=np.concatenate(areas) if areas else np.empty(0)
 snapshots[f.stem]=(c,area)
 rows.append(dict(name=f.stem,t=float(a['clock']),blocks=len(a['keys']),tread_triangles=len(c),tread_area_m2=float(area.sum())))
final=snapshots.get('mesh_300')
if final is not None and len(final[0]):
 tree=cKDTree(final[0])
 for row in rows:
  c,area=snapshots[row['name']]
  if len(c):
   dist,_=tree.query(c);row['area_without_surface_within_14cm_at_300_m2']=float(area[dist>.14].sum())
changes=[];clear=[]
for l in (p/'mesh_trace/changes.jsonl').open():
 e=json.loads(l)
 if e.get('clear'):clear.append(e['t'])
 for x,y,z,old,new in e.get('changes',[]):
  if old>0 and new==0:changes.append(dict(t=e['t'],key=[x,y,z],previous_triangles=old))
result=dict(scope='ROI x42..53 y-24..-12; tread proxy abs(normal_z)>0.7, z -7..-1.5; centroid proximity, not exact surface distance',snapshots=rows,clear_events=clear,emptied_previously_nonempty_blocks=len(changes),empty_events=changes)
(p/'mesh_trace/analysis.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='empty_events'},indent=2))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
selected=[k for k in ('mesh_225','mesh_245','mesh_265','mesh_300') if k in snapshots]
fig,axes=plt.subplots(1,len(selected),figsize=(4*len(selected),5),layout='constrained',squeeze=False)
for ax,key in zip(axes[0],selected):
 c,area=snapshots[key]
 if len(c):im=ax.scatter(c[:,0],c[:,1],c=c[:,2],vmin=-7,vmax=-1.5,s=3,cmap='turbo')
 ax.set(xlim=(43,52),ylim=(-23,-13),aspect='equal',title=key,xlabel='World X (m)',ylabel='World Y (m)')
fig.savefig(p/'mesh_trace/tread_surfaces.png',dpi=170)
