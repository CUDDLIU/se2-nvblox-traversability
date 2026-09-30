"""Track raw mapper block changes in a fixed staircase ROI; never mask deletions."""
import json
from pathlib import Path
import numpy as np
from mesh_wire import decode_mesh

class MeshTrace:
    def __init__(self,out,start,roi=None,times=None):
        self.out=Path(out)/'mesh_trace';self.out.mkdir();self.start=start
        self.file=(self.out/'changes.jsonl').open('w');self.blocks={};self.next=0
        self.roi=roi or [42,53,-24,-12,-9,2]
        self.times=sorted(times or [190,205,215,225,235,245,255,265,280,300,340,360,380,400,410])
    def update(self,raw,clock,pose=None):
        msg=decode_mesh(raw);t=clock-self.start if clock is not None else None
        if t is None:return
        if msg.clear:self.blocks.clear();self.file.write(json.dumps(dict(t=t,clear=True))+'\n')
        changes=[]
        for k,b in zip(msg.block_indices,msg.blocks):
            key=(k.x,k.y,k.z);center=(np.array(key)+.5)*msg.block_size_m
            if not all(self.roi[2*i]<=center[i]<=self.roi[2*i+1] for i in range(3)):continue
            old=self.blocks.get(key);old_count=len(old[1]) if old is not None else 0
            new_count=len(b.triangles_array)
            changes.append([*key,old_count,new_count])
            if new_count:self.blocks[key]=(b.vertices_array,b.triangles_array)
            else:self.blocks.pop(key,None)
        if changes:self.file.write(json.dumps(dict(t=t,changes=changes),separators=(',',':'))+'\n')
        if self.next<len(self.times) and t>=self.times[self.next]:
            target=self.times[self.next];self.next+=1
            arrays={'keys':np.array(list(self.blocks),dtype=np.int32),'clock':np.array(t)}
            for i,(v,tri) in enumerate(self.blocks.values()):arrays['v'+str(i)]=v;arrays['t'+str(i)]=tri
            path=self.out/f'mesh_{target:03d}.npz'
            np.savez(path,**arrays)
            path.with_suffix('.json').write_text(json.dumps(dict(bag_elapsed=t,pose=pose,
                roi=self.roi,block_size=msg.block_size_m),indent=2))
            self.file.flush()
    def close(self):self.file.close()
