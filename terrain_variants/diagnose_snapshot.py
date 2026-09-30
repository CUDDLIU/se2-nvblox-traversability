"""Offline repeatable geometry/threshold profiling from captured Mesh blocks."""
import argparse
from collections import Counter
from dataclasses import replace
import json
from pathlib import Path
import sys
import time
import numpy as np
sys.path.insert(0,str(Path(__file__).parent/'local_window'))
from paper_pipeline import Config,Engine

def run(snapshot,plot=False):
    meta=json.loads(snapshot.with_suffix('.json').read_text());pose=meta['pose']['position']
    saved=np.load(snapshot);blocks={tuple(k): (saved['v'+str(i)],saved['t'+str(i)]) for i,k in enumerate(saved['keys'])}
    results=[]
    for flag in (False,True):
        c=Config(tile_cells=8,max_step=.20,max_slope_deg=35.,stair_riser_filter=flag,surface_normal_filter=flag)
        e=Engine(c,native_mesh=True);begin=time.monotonic();e.update(blocks)
        e.process(budget_ms=35,defer_polygons=True,priority_position=pose)
        spans=[(s,int(m),reason) for r in e.results.values() for s,m,reason in zip(r.spans,r.masks,r.reasons)]
        near=[(s,m,reason) for s,m,reason in spans if (s.ix*.1-pose[0])**2+(s.iy*.1-pose[1])**2<2.25 and pose[2]-.9<s.z<pose[2]+.3]
        report=dict(stair_riser_filter=flag,elapsed_ms=(time.monotonic()-begin)*1000,
            promoted=e.classifier.promoted_risers,spans=len(spans),near_spans=len(near),
            near_permitted=sum(bool(m) for s,m,r in near),near_reasons=dict(Counter(r for s,m,r in near)),
            metrics=e.last_metrics)
        results.append(report)
        if plot:
            import matplotlib;matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            fig,axes=plt.subplots(1,3,figsize=(17,5),layout='constrained')
            for ax,pair in zip(axes,((0,1),(0,2),(1,2))):
                xyz=np.array([[(s.ix+.5)*.1,(s.iy+.5)*.1,s.z] for s,m,r in near])
                colors=['#249d43' if m else '#b7b7b7' if r=='slope' else '#e57b32' for s,m,r in near]
                if len(xyz):ax.scatter(xyz[:,pair[0]],xyz[:,pair[1]],c=colors,s=12,marker='s')
                ax.scatter(pose[pair[0]],pose[pair[1]],c='black',marker='x',s=55)
                ax.set(xlabel='XYZ'[pair[0]]+' (m)',ylabel='XYZ'[pair[1]]+' (m)',aspect='equal')
            fig.suptitle(f'{snapshot.stem}: riser filter={flag}, near permitted={report["near_permitted"]}, promoted={report["promoted"]}')
            fig.savefig(snapshot.with_name(snapshot.stem+f'_riser_{int(flag)}.png'),dpi=140);plt.close(fig)
    output=dict(snapshot=str(snapshot),pose=pose,results=results)
    snapshot.with_name(snapshot.stem+'_analysis.json').write_text(json.dumps(output,indent=2))
    return output

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('snapshot',type=Path);p.add_argument('--plot',action='store_true');args=p.parse_args()
    print(json.dumps(run(args.snapshot,args.plot),indent=2))
