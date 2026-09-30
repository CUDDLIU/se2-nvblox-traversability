"""Hash complete per-cell masks/reasons/distances under interchangeable native libraries."""
import argparse
import ctypes
import hashlib
import json
from pathlib import Path
import sys
import time
import numpy as np

def main():
    p=argparse.ArgumentParser();p.add_argument('library',type=Path);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    root=Path(__file__).resolve().parent;sys.path.insert(0,str(root/'local_window'))
    original=ctypes.CDLL
    ctypes.CDLL=lambda path,*a,**k:original(str(args.library.resolve()) if Path(path).name=='raster.so' else path,*a,**k)
    from paper_pipeline import Config,Engine
    result={}
    for path in sorted((root/'snapshots').glob('*/mesh_*.npz')):
        meta=json.loads(path.with_suffix('.json').read_text());pose=meta['pose']['position'];data=np.load(path)
        blocks={tuple(k):(data['v'+str(i)],data['t'+str(i)]) for i,k in enumerate(data['keys'])}
        c=Config(tile_cells=8,max_step=.2,max_slope_deg=40.,surface_normal_filter=True,stair_riser_filter=True)
        engine=Engine(c,native_mesh=True);engine.current_position=pose
        start=time.monotonic();engine.update(blocks);engine.process(budget_ms=35,defer_polygons=True,priority_position=pose)
        rows=[]
        for key,r in sorted(engine.results.items()):
            rows.extend([s.ix,s.iy,s.iz,int(m),float(d),reason] for s,m,d,reason in zip(r.spans,r.masks,r.distances,r.reasons))
        result[str(path.relative_to(root))]=dict(sha256=hashlib.sha256(json.dumps(rows).encode()).hexdigest(),
            cells=len(rows),classify_ms=engine.last_metrics['classify_ms'],elapsed_ms=(time.monotonic()-start)*1000)
    args.output.write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))

if __name__=='__main__':main()
