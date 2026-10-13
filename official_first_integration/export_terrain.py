#!/usr/bin/env python3
"""Rasterize measured upward/horizontal mesh triangles, retaining separate floors.
No hole fill, obstacle deletion, erosion, or yaw-clearance mask is applied.
"""
import argparse, json, hashlib, math
from pathlib import Path
import numpy as np

def export(directory, output):
    directory=Path(directory);output=Path(output)
    doc=json.loads((directory/'navmesh.json').read_text());source=directory/doc['source_mesh']
    r=.05; supports={}; measured_triangles=[]
    with np.load(source,allow_pickle=False) as data:
        for i in range(len(data['keys'])):
            vertices=data['v'+str(i)];triangles=vertices[data['t'+str(i)]]
            normals=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0]);norm=np.linalg.norm(normals,axis=1)
            # Some exported meshes have inconsistent winding. Magnitude selects
            # slope, but this remains a surface model, not a certified footstep map.
            good=(norm>1e-9)&(np.abs(normals[:,2])>=np.cos(np.deg2rad(40))*norm)
            for t in triangles[good]:
                measured_triangles.append(t.copy())
                low=np.floor(t[:,:2].min(0)/r).astype(int);high=np.floor(t[:,:2].max(0)/r).astype(int)
                x,y=np.meshgrid(np.arange(low[0],high[0]+1),np.arange(low[1],high[1]+1),indexing='ij');keys=np.column_stack([x.ravel(),y.ravel()]);xy=(keys+.5)*r
                matrix=(t[1:,:2]-t[0,:2]).T
                if abs(np.linalg.det(matrix))<1e-10:continue
                uv=np.linalg.solve(matrix,(xy-t[0,:2]).T).T
                keep=(uv[:,0]>=-1e-6)&(uv[:,1]>=-1e-6)&(uv.sum(1)<=1+1e-6)
                for k,v in zip(keys[keep],uv[keep]):
                    z=float(t[0,2]+v@(t[1:,2]-t[0,2]));key=tuple(map(int,k));supports.setdefault(key,{}).setdefault(round(z/.01),[]).append(z)
    rows=[];by_xy={}
    for key,heights in sorted(supports.items()):
        by_xy[key]=[]
        for _,zs in sorted(heights.items()):
            by_xy[key].append(len(rows));rows.append((*key,float(np.mean(zs))))
    parent=list(range(len(rows)))
    def find(i):
        while parent[i]!=i:parent[i]=parent[parent[i]];i=parent[i]
        return i
    for i,(x,y,z) in enumerate(rows):
        for key in [(x+1,y),(x,y+1),(x,y)]:
            for j in by_xy.get(key,[]):
                if abs(rows[j][2]-z)<=.2:parent[find(j)]=find(i)
    labels={};output.parent.mkdir(parents=True,exist_ok=True)
    with output.open('w') as f:
        f.write('FIRST_TERRAIN 1 0.05\n')
        for i,(x,y,z) in enumerate(rows):
            component=labels.setdefault(find(i),len(labels));f.write(f'{x} {y} {z:.6f} {component}\n')
    # Retain triangle boundaries for exact point support queries; a supported
    # point near a cell edge must not depend on that cell's center being covered.
    triangle_path=Path(str(output)+'.triangles');written=0
    with triangle_path.open('w') as f:
        f.write('FIRST_TRIANGLES 1\n')
        for t in measured_triangles:
            center=t.mean(0);key=tuple(np.floor(center[:2]/r).astype(int));candidate=[]
            for dx in (-1,0,1):
                for dy in (-1,0,1):candidate.extend(by_xy.get((key[0]+dx,key[1]+dy),[]))
            candidate=[j for j in candidate if abs(rows[j][2]-center[2])<.2]
            if not candidate:continue
            j=min(candidate,key=lambda j:abs(rows[j][2]-center[2]));component=labels[find(j)]
            f.write(' '.join(f'{float(v):.9f}' for v in t.reshape(-1))+f' {component}\n');written+=1
    report=dict(triangles=written,triangle_sha256=hashlib.sha256(triangle_path.read_bytes()).hexdigest(),source=str(source),mesh_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),terrain_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),cells=len(rows),components=len(labels),resolution_m=r,max_step_m=.2,slope_deg=40,hole_fill=False)
    output.with_suffix('.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('map');p.add_argument('output');a=p.parse_args();export(a.map,a.output)
