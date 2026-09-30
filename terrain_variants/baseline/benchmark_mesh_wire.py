"""Replay identical captured CDR for a reproducible worker-count comparison."""
import argparse
import json
import time
from pathlib import Path
import numpy as np
from mesh_wire import decode_mesh,block_arrays
from paper_pipeline import Engine


def main():
    p=argparse.ArgumentParser();p.add_argument('wire');p.add_argument('--repeat',type=int,default=5)
    a=p.parse_args();m=decode_mesh(Path(a.wire).read_bytes())
    blocks={(k.x,k.y,k.z):block_arrays(b) for k,b in zip(m.block_indices,m.blocks)}
    timings=[]
    for _ in range(a.repeat):
        e=Engine(native_mesh=True);start=time.monotonic();e.update(blocks)
        updated=time.monotonic();ticks=[]
        while e.pending:
            e.process(budget_ms=35,defer_polygons=True)
            ticks.append(e.last_metrics['process_ms'])
        timings.append(dict(update_ms=(updated-start)*1000,total_ms=(time.monotonic()-start)*1000,
                            tick_p50_p95_max=np.percentile(ticks,[50,95,100]).tolist()))
    print(json.dumps(dict(blocks=len(blocks),timings=timings)))


if __name__=='__main__':main()
