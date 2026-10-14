"""Offline original AArch64 IMU/ray decisions vs reconstructed source.

PCL octree calls use the same installed PCL query on both sides. This checks
the proprietary ray geometry and erase decisions, not PCL-version equivalence.
No original binary is executed as a process, and there is no DDS/robot I/O.
"""
import argparse,ctypes as C,json,math,struct
from pathlib import Path
import numpy as np
from unicorn.arm64_const import *
from differential_height_builder import BuilderOracle
from differential_heightmap import IDENTITY
from height_frontend import library,ptr,FLOAT,HeightFrontend

class FrontendOracle(BuilderOracle):
    def __init__(self,elf):
        super().__init__(elf);self.lib=library();self.removed=set();self.query_count=0
        self.libm=C.CDLL('libm.so.6')
        for name in ('acosf','sqrtf','sinf','cosf'):
            f=getattr(self.libm,name);f.argtypes=[C.c_float];f.restype=C.c_float
    def hook(self,uc,a,size,context):
        x=[uc.reg_read(r) for r in (UC_ARM64_REG_X0,UC_ARM64_REG_X1,UC_ARM64_REG_X2,UC_ARM64_REG_X3)]
        ret=True
        if a==0x87cc0:
            o=np.frombuffer(uc.mem_read(x[1],12),'<f4').copy();d=np.frombuffer(uc.mem_read(x[2],12),'<f4').copy()
            ids=np.empty(len(self.old),np.int32)
            n=self.lib.aos_ray_query(ptr(self.old),len(self.old),ptr(o),ptr(d),ids.ctypes.data_as(C.POINTER(C.c_int)))
            buf=self.alloc(max(4,4*n));self.put(x[3],'QQQ',buf,buf+4*n,buf+4*n)
            if n:uc.mem_write(buf,ids[:n].tobytes())
            uc.reg_write(UC_ARM64_REG_X0,n);self.query_count+=1
        elif a==0xb2358:
            self.removed.add(self.get(x[1],'i'))
            # Ordered-set storage only. The original membership search executes.
            def tree(items):
                if not items:return 0
                mid=len(items)//2;node=self.alloc(40)
                self.put(node+16,'QQi',tree(items[:mid]),tree(items[mid+1:]),items[mid]);return node
            self.put(x[0]+16,'Q',tree(sorted(self.removed)))
        elif a==0x87350:uc.mem_write(x[0],bytes(0xa0))
        elif a in (0x87170,0x89600,0xb2310,0xa6340):pass
        elif a==0x88070:uc.reg_write(UC_ARM64_REG_X0,1000000000)
        elif a in (0x898b0,0x89650,0x87930,0x88c20,0xb28d0):uc.reg_write(UC_ARM64_REG_X0,0)
        elif a==0x887a0:uc.reg_write(UC_ARM64_REG_X0,0)
        elif a in (0x87c70,0x88cd0,0x88340):
            f=struct.unpack('<f',struct.pack('<I',uc.reg_read(UC_ARM64_REG_S0)))[0]
            f=getattr(self.libm,{0x87c70:'acosf',0x88cd0:'sqrtf',0x88340:'sinf'}[a])(f)
            uc.reg_write(UC_ARM64_REG_S0,struct.unpack('<I',struct.pack('<f',f))[0])
        else:ret=False;super().hook(uc,a,size,context)
        if ret:uc.reg_write(UC_ARM64_REG_PC,uc.reg_read(UC_ARM64_REG_LR))
    def cloud(self,shared,points):
        cloud=self.alloc(144);data=self.alloc(max(16,16*len(points)))
        self.put(shared,'QQ',cloud,0);self.put(cloud+48,'QQQ',data,data+len(points)*16,data+len(points)*16)
        if len(points):self.uc.mem_write(data,np.column_stack([points,np.ones(len(points))]).astype('<f4').tobytes())
    def parallel(self,now,old):
        self.heap=0x800000;self.removed=set();self.old=np.ascontiguousarray(old,np.float32)
        self.cloud(0x1fd000,now);self.cloud(0x1fd100,old)
        self.put(0x1fd200,'Q',0x1fd400)
        self.uc.reg_write(UC_ARM64_REG_X3,0x1fd200);self.uc.reg_write(UC_ARM64_REG_X4,0x1fd300)
        self.uc.reg_write(UC_ARM64_REG_X5,0);self.uc.reg_write(UC_ARM64_REG_X6,len(now))
        self.run(0x8d1f8)
        return np.array([p for i,p in enumerate(old) if i not in self.removed],np.float32).reshape(-1,3)
    def serial(self,now,old):
        self.heap=0x800000;self.removed=set();self.old=np.ascontiguousarray(old,np.float32)
        self.cloud(0x1fd000,now);self.cloud(0x1fd100,old);self.cloud(0x1fd200,[])
        output=self.get(0x1fd200,'Q');data=self.alloc(max(16,len(old)*16))
        self.put(output+48,'QQQ',data,data,data+len(old)*16)
        self.put(self.obj+0x388,'f',.1);self.uc.reg_write(UC_ARM64_REG_X3,0x1fd200)
        self.run(0x8d5f8);n=(self.get(output+56,'Q')-data)//16
        return np.frombuffer(self.uc.mem_read(data,16*n),'<f4').reshape(-1,4)[:,:3].copy()
    def imu(self,samples,t):
        self.heap=0x800000;self.uc.mem_write(self.obj,bytes(0x1000))
        block_count=len(samples)//4+1;table=self.alloc(block_count*8)
        blocks=[self.alloc(512) for _ in range(block_count)]
        for i,b in enumerate(blocks):self.put(table+i*8,'Q',b)
        # libstdc++ deque layout: map,size,start iterator,finish iterator.
        self.put(self.obj+0x1f8,'QQ',table,block_count)
        self.put(self.obj+0x208,'QQQQ',blocks[0],blocks[0],blocks[0]+512,table)
        j=len(samples)//4;end=blocks[j]+128*(len(samples)%4)
        self.put(self.obj+0x228,'QQQQ',end,blocks[j],blocks[j]+512,table+8*j)
        for i,(stamp,q) in enumerate(samples):
            b=blocks[i//4]+128*(i%4);sec=int(stamp);nano=round((stamp-sec)*1e9)
            self.put(b,'qq',sec,nano);self.put(b+0x60,'dddd',q[3],q[0],q[1],q[2])
        self.put(self.obj+0x550,'Q',0x600000)
        self.uc.reg_write(UC_ARM64_REG_D0,struct.unpack('<Q',struct.pack('<d',t))[0])
        self.run(0x8e148)
        result=self.uc.reg_read(UC_ARM64_REG_X0)
        return bool(result),np.frombuffer(self.uc.mem_read(self.obj+0x310,16),'<f4').copy()

def compare(elf):
    ref=FrontendOracle(elf);lib=library();rng=np.random.default_rng(61014)
    lib.aos_interpolate.argtypes=[C.c_void_p,C.c_double,FLOAT];lib.aos_interpolate.restype=C.c_int
    rays=0;erased=0
    for i in range(100):
        old=rng.uniform([-2,-1,-.8],[2,1,.4],(80,3)).astype('f4')
        # Collinear farther returns exercise real ray hits and stale-obstacle removal.
        now=np.vstack([old*2,rng.uniform([-3,-2,-1],[3,2,.5],(20,3))]).astype('f4')
        if i%3==0:old[:6]=[[.4,0,0],[-.4,0,0],[.5,.01,0],[-.5,-.01,0],[0,.2,0],[0,-.2,0]]
        for parallel in (False,True):
            actual=(ref.parallel if parallel else ref.serial)(now,old);out=np.empty_like(old)
            n=lib.aos_clear(ptr(now),len(now),ptr(old),len(old),parallel,ptr(out))
            assert n>=0 and np.array_equal(actual,out[:n]),(parallel,i,actual.shape,n)
            erased+=len(old)-n;rays+=len(now)
    imu_cases=0;max_error=0.
    for i in range(60):
        qs=rng.normal(size=(8,4));qs/=np.linalg.norm(qs,axis=1)[:,None]
        # Include normalization and same/antipodal quaternion branches.
        if i%4==0:qs[:]=qs[0]
        if i%4==1:qs[1::2]*=-1
        qs*=rng.uniform(.8,1.2,(8,1))
        samples=[(10.+j*.1,q) for j,q in enumerate(qs)]
        t=9.9 if i%10==0 else 10.8 if i%10==1 else 10.+(i%7)*.1+.037
        ok,q=ref.imu(samples,t)
        with HeightFrontend() as f:
            for s,v in samples:f.add_imu(s,v)
            out=np.empty(4,np.float32);got=bool(lib.aos_interpolate(f.handle,t,ptr(out)))
            assert ok==got,(i,ok,got)
            if ok:
                error=float(np.max(np.abs(q-out)));max_error=max(max_error,error)
                assert error<2e-6,(i,q,out,error)
        imu_cases+=1
    return dict(reference_sha256=IDENTITY,parallel_cloud_pairs=100,serial_cloud_pairs=100,rays=rays,
                octree_queries=ref.query_count,erased_points=erased,
                imu_cases=imu_cases,imu_max_abs_error=max_error,
                scope='original ray decisions and IMU interpolation; PCL queries delegated to installed library; full sync callback not instruction-differential verified')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--elf',required=True);p.add_argument('--output',required=True);a=p.parse_args()
    result=compare(a.elf);Path(a.output).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
