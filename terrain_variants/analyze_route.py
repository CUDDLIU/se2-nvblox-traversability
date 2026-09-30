"""Offline SE(2) connectivity of completed historical cells and recorded poses.

This is a static geometry regression, not a robot motion or gait validation.
Archived cells are identified separately; historical aggregation is never
reported as a current collision-free map. A gap is never interpolated away.
"""
import argparse
import csv
from collections import Counter, defaultdict
import json
import math
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree


def components(cells, yaw_bins, step_units):
    """Same-yaw 4-neighbor translations and adjacent feasible yaw rotations."""
    states={};columns=defaultdict(list)
    for i,(x,y,z,mask) in enumerate(cells):
        columns[x,y].append(i)
        for yaw in range(yaw_bins):
            if int(mask)&(1<<yaw):states[i,yaw]=len(states)
    parent=list(range(len(states)));size=[1]*len(states)
    def find(i):
        while i!=parent[i]:parent[i]=parent[parent[i]];i=parent[i]
        return i
    def join(a,b):
        a,b=find(a),find(b)
        if a==b:return
        if size[a]<size[b]:a,b=b,a
        parent[b]=a;size[a]+=size[b]
    for (i,yaw),s in states.items():
        neighbor=states.get((i,(yaw+1)%yaw_bins))
        if neighbor is not None:join(s,neighbor)
        x,y,z,_=cells[i]
        for xy in ((x+1,y),(x,y+1)):
            for j in columns.get(xy,[]):
                if abs(cells[j][2]-z)>step_units+1e-6:continue
                neighbor=states.get((j,yaw))
                if neighbor is not None:join(s,neighbor)
    labels={key:find(value) for key,value in states.items()}
    counts=Counter(labels.values())
    return labels,counts


def component_cover(candidate_components):
    """Count route support per connected component without a nearest-cell bias."""
    component_samples=defaultdict(set)
    for i,candidates in enumerate(candidate_components):
        for label in candidates:component_samples[label].add(i)
    best_count=max(map(len,component_samples.values()),default=0)
    remaining={i for i,candidates in enumerate(candidate_components) if candidates};cover=[]
    while remaining:
        label=max(component_samples,key=lambda c:len(component_samples[c]&remaining))
        added=component_samples[label]&remaining
        if not added:break
        cover.append(dict(component=int(label),newly_covered_samples=len(added)))
        remaining-=added
    return best_count,cover


def analyze(path,make_plot=True,body_ground_offset=None):
    history=json.loads((path/'local_history.json').read_text())
    cfg=history['config'];res=cfg['resolution'];dz=cfg['vertical_resolution'];bins=cfg['yaw_bins']
    current={tuple(s['key']):s['cells'] for s in history['slabs']}
    archived={tuple(s['key']):s['cells'] for s in history.get('archived',[])}
    merged={**archived,**current}
    cells=[tuple(map(int,row)) for rows in merged.values() for row in rows if row[3]]
    labels,counts=components(cells,bins,cfg['max_step']/dz)
    poses=[]
    for line in (path/'metrics.jsonl').open():
        event=json.loads(line)
        if event['kind']=='odom':poses.append(event['data'])
    xyz=np.asarray([[ (x+.5)*res,(y+.5)*res,z*dz] for x,y,z,_ in cells]).reshape(-1,3)
    trajectory=np.asarray([p['position'] for p in poses]).reshape(-1,3)
    summary=dict(scope='completed static historical SE(2) cells, including explicitly archived geometry',
        cells=len(cells),archived_cells=sum(len(v) for k,v in archived.items() if k not in current),
        states=sum(counts.values()),components=len(counts),largest_component_states=max(counts.values(),default=0),
        max_step=cfg['max_step'],yaw_bins=bins,route_samples=len(poses),control_outputs='none')
    if not len(xyz) or not len(trajectory):
        summary['route_coverage']=0.;return summary
    # Infer body-to-ground offset from observed surfaces near the initial flat
    # segment, bounded to the robot's plausible mounting height. Report it.
    tree_xy=cKDTree(xyz[:,:2]);offsets=[]
    for p in trajectory[:min(100,len(trajectory))]:
        for j in tree_xy.query_ball_point(p[:2],.2):
            h=p[2]-xyz[j,2]
            if .15<h<.9:offsets.append(h)
    if offsets:
        hist,edges=np.histogram(offsets,bins=np.arange(.15,.92,.02))
        j=int(hist.argmax());near=[h for h in offsets if edges[j]<=h<=edges[j+1]]
        offset=float(np.median(near))
    else:offset=.45
    inferred_offset=offset
    if body_ground_offset is not None:
        if not .15<body_ground_offset<.9:raise ValueError('Invalid body-ground offset')
        offset=float(body_ground_offset)
    ground=trajectory.copy();ground[:,2]-=offset
    tree=cKDTree(xyz);dist,nearest=tree.query(ground)
    covered=[];component=[];any_coverage=[];candidate_components=[];matched=[]
    for p,pose,near in zip(ground,poses,nearest):
        qx,qy,qz,qw=pose['rotation'];yaw=math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
        yaw_bin=round(yaw/(2*math.pi)*bins)%bins
        candidates=tree.query_ball_point(p,.30)
        valid=[j for j in candidates if abs(xyz[j,2]-p[2])<=.20 and (j,yaw_bin) in labels]
        valid.sort(key=lambda j:np.linalg.norm(xyz[j]-p))
        covered.append(bool(valid));component.append(labels[valid[0],yaw_bin] if valid else -1)
        matched.append(valid[0] if valid else -1)
        candidate_components.append({labels[j,yaw_bin] for j in valid})
        any_coverage.append(any(abs(xyz[j,2]-p[2])<=.20 for j in candidates))
    covered=np.asarray(covered);comp_counts=Counter(c for c in component if c>=0)
    dominant=comp_counts.most_common(1)[0][0] if comp_counts else -2
    # A nearest-cell choice can prefer a tiny fragment even when the same
    # pose also matches a large connected region. Report both metrics.
    best_count,cover=component_cover(candidate_components)
    upper=trajectory[:,2]>trajectory[0,2]+1.
    lower=trajectory[:,2]<trajectory[0,2]-1.
    lowest=trajectory[:,2]<trajectory[:,2].min()+.5
    between=(trajectory[:,2]<trajectory[0,2]-1.) & (trajectory[:,2]>trajectory[:,2].min()+1.)
    t=np.asarray([p['stamp'] for p in poses]);t-=t[0]
    # Keep every sample available for diagnosis without replaying or loading
    # the large metrics stream again. These are historical map matches, not
    # measured contact points or current collision permissions.
    suffix='_common_offset' if body_ground_offset is not None else ''
    with (path/('route_samples'+suffix+'.csv')).open('w',newline='') as stream:
        writer=csv.writer(stream)
        writer.writerow(['elapsed_from_first_pose_s','body_x','body_y','body_z',
                         'estimated_ground_z','covered','component','any_yaw_covered',
                         'nearest_geometry_distance_m','matched_surface_z'])
        for i,p in enumerate(trajectory):
            writer.writerow([t[i],*p,ground[i,2],int(covered[i]),component[i],
                             int(any_coverage[i]),dist[i],xyz[matched[i],2] if matched[i]>=0 else ''])
    intervals=[];start=None
    for i,missing in enumerate(list(~covered)+[False]):
        if missing and start is None:start=i
        if not missing and start is not None:
            end=i-1
            intervals.append(dict(start_s=float(t[start]),end_s=float(t[end]),
                sample_span_s=float(t[end]-t[start]),samples=end-start+1,
                body_z_min=float(trajectory[start:i,2].min()),
                body_z_max=float(trajectory[start:i,2].max())))
            start=None
    summary.update(inferred_body_ground_offset_m=inferred_offset,body_ground_offset_used_m=offset,
        body_ground_offset_source='common external estimate' if body_ground_offset is not None else 'individual output map',
        offset_evidence_count=len(offsets),
        route_position_tolerance_m=.30,route_height_tolerance_m=.20,
        route_coverage=float(covered.mean()),route_any_yaw_coverage=float(np.mean(any_coverage)),
        route_dominant_component_fraction=float(np.mean(np.asarray(component)==dominant)),
        route_components=len(comp_counts),upper_floor_samples=int(upper.sum()),
        best_single_component_route_coverage=best_count/len(poses),
        greedy_components_to_cover_observed_route=len(cover),greedy_component_cover=cover,
        upper_floor_coverage=float(covered[upper].mean()) if upper.any() else None,
        lower_than_start_samples=int(lower.sum()),
        lower_than_start_coverage=float(covered[lower].mean()) if lower.any() else None,
        lowest_altitude_coverage=float(covered[lowest].mean()) if lowest.any() else None,
        between_altitudes_coverage=float(covered[between].mean()) if between.any() else None,
        altitude_groups='lower: start_z-1m; lowest: min_z+0.5m; between: min_z+1m < z < start_z-1m; these are altitude groups, not verified stair labels',
        nearest_geometry_distance_m=dict(p50=float(np.median(dist)),p95=float(np.percentile(dist,95))))
    summary['uncovered_route_intervals']=intervals
    summary['uncovered_time_reference']='seconds from first recorded pose; sample spans, not continuous-time proof'
    if make_plot:
        import matplotlib;matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(1,2,figsize=(15,6),layout='constrained')
        h=axes[0].scatter(xyz[:,0],xyz[:,1],c=xyz[:,2],s=3,cmap='viridis')
        axes[0].plot(trajectory[:,0],trajectory[:,1],color='black',lw=.6,alpha=.45)
        axes[0].scatter(trajectory[~covered,0],trajectory[~covered,1],s=5,c='#e03333',label='No matching yaw state within tolerance')
        axes[0].set(aspect='equal',xlabel='World X (m)',ylabel='World Y (m)',title='Historical feasible cells and recorded route')
        axes[0].legend(loc='best',fontsize=7);fig.colorbar(h,ax=axes[0],label='Surface Z (m)')
        axes[1].plot(t,ground[:,2],color='black',lw=1,label='Estimated ground under recorded body')
        axes[1].scatter(t[covered],xyz[np.asarray(matched)[covered],2],s=4,c='#1a8d3d',label='Matching yaw state nearby')
        axes[1].scatter(t[~covered],ground[~covered,2],s=5,c='#e03333',label='Uncovered route')
        axes[1].set(xlabel='Bag elapsed (s)',ylabel='World Z (m)',title=f'Route coverage: {covered.mean():.1%}; components: {len(comp_counts)}')
        axes[1].legend(fontsize=8)
        fig.savefig(path/('route_connectivity_common_offset.png' if body_ground_offset is not None else 'route_connectivity.png'),dpi=170);plt.close(fig)
    return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('experiment',type=Path);parser.add_argument('--no-plot',action='store_true')
    parser.add_argument('--body-ground-offset',type=float)
    args=parser.parse_args();result=analyze(args.experiment,not args.no_plot,args.body_ground_offset)
    filename='route_analysis_common_offset.json' if args.body_ground_offset is not None else 'route_analysis.json'
    (args.experiment/filename).write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
