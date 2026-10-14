"""Run original computeHeightMap instructions offline with libc/container I/O.

No geometric branch is stubbed. OpenCV filtering is disabled for this oracle;
the native filter's medianBlur(3) call is checked separately from its wrapper.
"""
import argparse,json,math,struct
from pathlib import Path
import numpy as np
from unicorn.arm64_const import *
from differential_heightmap import Oracle,IDENTITY
from heightmap import build_height_map


class BuilderOracle(Oracle):
    def __init__(self,elf):
        super().__init__(elf)
        self.uc.mem_map(0x800000,0x4000000)
        self.heap=0x800000

    def alloc(self,n):
        p=self.heap;self.heap+=(n+15)//16*16
        if self.heap>=0x4800000:raise AssertionError('oracle heap exhausted')
        return p

    def hook(self,uc,a,size,context):
        x=[uc.reg_read(r) for r in (UC_ARM64_REG_X0,UC_ARM64_REG_X1,UC_ARM64_REG_X2)]
        ret=True
        if a in (0x873b0,0x89260):uc.reg_write(UC_ARM64_REG_X0,self.alloc(x[0]))
        elif a in (0x873a0,0x89020):pass
        elif a==0x87210:
            if x[2]:uc.mem_write(x[0],bytes([x[1]&255])*x[2])
        elif a in (0x87340,0x86ff0):
            if x[2]:uc.mem_write(x[0],bytes(uc.mem_read(x[1],x[2])))
        elif a==0xba8d0:
            # unordered_map access is bookkeeping only in this function;
            # reserve a vector so the original point insertion executes.
            key=bytes(uc.mem_read(x[1],8))
            if key not in self.cells:
                v=self.alloc(24);data=self.alloc(65536);self.put(v,'QQQ',data,data,data+65536);self.cells[key]=v
            uc.reg_write(UC_ARM64_REG_X0,self.cells[key])
        elif a==0xb21e8:
            value=bytes(uc.mem_read(x[2],4))
            begin,end=struct.unpack('<QQ',uc.mem_read(x[0],16));n=end-begin
            dest=self.alloc(max(64,n*2+4))
            if n:uc.mem_write(dest,bytes(uc.mem_read(begin,n)))
            uc.mem_write(dest+n,value);self.put(x[0],'QQQ',dest,dest+n+4,dest+max(64,n*2+4))
        elif a in (0x886c0,0x87830,0x87950):
            def s(reg):return struct.unpack('<f',struct.pack('<I',uc.reg_read(reg)))[0]
            f=s(UC_ARM64_REG_S0)
            if a==0x886c0:f=math.atan2(f,s(UC_ARM64_REG_S1))
            elif a==0x87830:f=math.cos(f)
            else:
                self.put(x[0],'f',math.sin(f));self.put(x[1],'f',math.cos(f))
            uc.reg_write(UC_ARM64_REG_S0,struct.unpack('<I',struct.pack('<f',f))[0])
        else:ret=False;super().hook(uc,a,size,context)
        if ret:uc.reg_write(UC_ARM64_REG_PC,uc.reg_read(UC_ARM64_REG_LR))

    def build(self,points,shape=(9,11),retained=6,pitch=0.):
        self.heap=0x800000;self.cells={}
        self.uc.mem_write(self.obj,bytes(0x1000))
        nx,ny=shape
        self.put(self.obj+0x348,'f',.55)
        self.put(self.obj+0x3cc,'ff',-4.,-4.)
        self.put(self.obj+0x3ec,'f',.25)
        self.put(self.obj+0x400,'ffff',0.,math.sin(pitch/2),0.,math.cos(pitch/2))
        self.put(self.obj+0x410,'fiii',.1,nx,ny,retained)
        for off,ptr in ((0x420,0x410000),(0x438,0x420000),(0x450,0x430000)):
            self.put(self.obj+off,'QQQ',ptr,nx,ny)
        self.put(self.obj+0x480,'QQ',0x440000,1)
        self.put(self.obj+0x520,'Q',0x650000)
        self.put(0x650030,'QQQ',0x660000,0x660000,0x670000)
        self.put(0x1fd000,'Q',0x670000)
        p=np.column_stack([points,np.ones(len(points))]).astype('<f4')
        self.put(0x670030,'QQQ',0x680000,0x680000+len(p)*16,0x680000+len(p)*16)
        if len(p):self.uc.mem_write(0x680000,p.tobytes())
        self.run(0x8f5d8)
        def matrix(ptr,dtype):return np.frombuffer(self.uc.mem_read(ptr,nx*ny*4),dtype).reshape(shape,order='F').copy()
        height=matrix(0x410000,'<f4');counts=matrix(0x420000,'<i4');raw=matrix(0x430000,'<f4')
        layers=self.get(self.obj+0x468,'Q')
        samples=np.array([matrix(self.get(layers+24*k,'Q'),'<f4') for k in range(retained)])
        return height,samples,counts,raw


def compare(elf):
    ref=BuilderOracle(elf);rng=np.random.default_rng(20261014)
    for scene in range(40):
        shape=(9,11);n=scene*17
        xy=rng.integers(0,shape,(n,2))
        p=np.column_stack([xy*.1-3.97,rng.choice([-.8,-.55,-.4,-.2,.1],n)]).astype('f4')
        pitch=(0.,.5,-.5)[scene%3]
        actual=ref.build(p,shape,pitch=pitch)
        expected=build_height_map(p,shape=shape,retained=6,pitch=pitch,filtered=False)
        for name,a,b in zip(('height','samples','counts','raw'),actual,expected):
            if not np.array_equal(a,b,equal_nan=True):
                bad=~((a==b)|(np.isnan(a)&np.isnan(b)))
                raise AssertionError((scene,name,np.argwhere(bad)[:10].tolist(),a[bad][:10],b[bad][:10]))
    return dict(passed=True,scenes=40,reference_sha256=IDENTITY,
                scope='finite input computeHeightMap; sample ring, cardinal missing-cell fill; filter/changeHeight disabled')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--elf',required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();result=compare(args.elf);args.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
