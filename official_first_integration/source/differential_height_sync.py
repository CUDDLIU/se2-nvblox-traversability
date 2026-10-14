"""Original syncCallback control/transformation sequence, offline.

PCL, allocation and cloud containers are host shims. findImu and ray clearing
are covered by differential_height_frontend and stubbed here. The original
SE(3) transforms, quaternion math, ordering, flags and copies execute in Unicorn.
"""
import argparse,ctypes as C,json,struct
from pathlib import Path
import numpy as np
from unicorn.arm64_const import *
from differential_height_frontend import FrontendOracle,IDENTITY
from height_frontend import HeightFrontend,ptr,FLOAT
from heightmap import build_height_map,obstacle_mask

class SyncOracle(FrontendOracle):
    def __init__(self,elf):
        super().__init__(elf)
        self.lib.aos_pcl_filter.argtypes=[FLOAT,C.c_int,C.c_int,FLOAT,FLOAT];self.lib.aos_pcl_filter.restype=C.c_int
        self.lib.aos_pcl_transform.argtypes=[FLOAT,C.c_int,FLOAT,FLOAT];self.lib.aos_pcl_transform.restype=C.c_int
        self.crop_input={};self.imu_q=None
        self.uc.mem_write(self.obj,bytes(0x1000))
        self.put(self.obj+0x188,'BB',1,1);self.put(self.obj+0x1c0,'BB',1,1)
        self.put(self.obj+0x350,'6f',4,-4,4,-4,.5,-1.5)
        self.put(self.obj+0x368,'6f',.4,-.4,.2,-.2,.4,-.5)
        self.put(self.obj+0x380,'5f',.05,.05,.1,.05,.05)
        for off in (0x500,0x510):self.cloud(self.obj+off,[])
    def read_cloud(self,p):
        start,end=struct.unpack('<QQ',self.uc.mem_read(p+48,16))
        return np.frombuffer(self.uc.mem_read(start,end-start),'<f4').reshape(-1,4)[:,:3].copy()
    def write_cloud(self,p,xyz):
        data=self.alloc(max(16,len(xyz)*16));self.put(p+48,'QQQ',data,data+len(xyz)*16,data+len(xyz)*16)
        if len(xyz):self.uc.mem_write(data,np.column_stack([xyz,np.ones(len(xyz))]).astype('<f4').tobytes())
    def hook(self,uc,a,size,context):
        x=[uc.reg_read(r) for r in (UC_ARM64_REG_X0,UC_ARM64_REG_X1,UC_ARM64_REG_X2)]
        ret=True
        def singles(n):return np.array([struct.unpack('<f',struct.pack('<I',uc.reg_read(UC_ARM64_REG_S0+i)))[0] for i in range(n)],np.float32)
        if a in (0xa63e8,0x893c0):uc.mem_write(x[0],bytes(144))
        elif a in (0xab418,0xb1a98,0x9eb88,0x87d80,0x90f50,0xa6340,0xace70):pass
        elif a==0x88070:uc.reg_write(UC_ARM64_REG_X0,1000000000)
        elif a==0xab6e8:self.put(x[0],'QQ',x[1],0)
        elif a in (0xab1a0,0xab4e0):self.write_cloud(x[0],self.read_cloud(x[1]))
        elif a==0xbc7d8:self.write_cloud(uc.reg_read(UC_ARM64_REG_X8),np.vstack([self.read_cloud(x[0]),self.read_cloud(x[1])]))
        elif a==0x899a0:self.crop_input[x[0]]=self.get(x[1],'Q')
        elif a in (0xb9a10,0xba128,0xb9d20,0xb88b0):
            if a==0xb88b0:
                p=self.read_cloud(self.crop_input[x[0]]);dest=x[1];mode=1
                args=np.frombuffer(uc.mem_read(x[0]+0x70,32),'<f4').copy()
            else:
                p=self.read_cloud(self.get(x[1],'Q'));dest=self.get(x[2] if a==0xba128 else x[1],'Q')
                mode=0 if a==0xb9a10 else 2;args=singles(3 if mode==0 else 6)
            out=np.empty_like(p);n=self.lib.aos_pcl_filter(ptr(p),len(p),mode,ptr(args),ptr(out));self.write_cloud(dest,out[:n])
        elif a==0xb8a80:
            p=self.read_cloud(x[0]);matrix=np.frombuffer(uc.mem_read(x[2],64),'<f4').copy();out=np.empty_like(p)
            n=self.lib.aos_pcl_transform(ptr(p),len(p),ptr(matrix),ptr(out));self.write_cloud(x[1],out[:n])
        elif a in (0x8d5f8,0x8db70):
            old=self.read_cloud(self.get(x[2],'Q'));now=self.read_cloud(self.get(x[1],'Q'));out=np.empty_like(old)
            n=self.lib.aos_clear(ptr(now),len(now),ptr(old),len(old),a==0x8db70,ptr(out))
            self.write_cloud(self.get(uc.reg_read(UC_ARM64_REG_X3),'Q'),out[:n])
        elif a==0x8e148:
            if self.imu_q is not None:self.put(self.obj+0x310,'4f',*self.imu_q)
            uc.reg_write(UC_ARM64_REG_X0,int(self.imu_q is not None))
        else:ret=False;super().hook(uc,a,size,context)
        if ret:uc.reg_write(UC_ARM64_REG_PC,uc.reg_read(UC_ARM64_REG_LR))
    def process(self,points,stamp,pose,q):
        self.imu_q=q;cloud=0x1fd000;odom=0x1fd100
        self.cloud(cloud+0x30,points);sec=int(stamp);ns=round((stamp-sec)*1e9)
        self.put(cloud,'qq',sec,ns);self.put(odom,'qq',sec,ns)
        self.put(odom+0x30,'7d',*pose[:3],pose[6],*pose[3:6]);self.run(0x92a00)
        return self.read_cloud(self.get(self.obj+0x500,'Q')),self.read_cloud(self.get(self.obj+0x510,'Q'))

def compare(elf):
    from scipy.spatial.transform import Rotation
    from scipy.spatial import cKDTree
    rng=np.random.default_rng(1410);cases=0;max_error=0.
    for accumulate,casting,parallel in [(False,True,True),(True,False,True),(True,True,True),(True,True,False)]:
        ref=SyncOracle(elf);ref.put(ref.obj+0x188,'B',accumulate);ref.put(ref.obj+0x1c0,'BB',casting,parallel)
        with HeightFrontend(accumulate=accumulate,casting=casting,parallel=parallel) as f:
            for i in range(8):
                p=rng.uniform([-4.5,-4.5,-1.6],[4.5,4.5,.6],(300,3)).astype('f4')
                pose=[.05*i,-.01*i,.01*i,*Rotation.from_euler('ZYX',[.03*i,.01*i,-.02*i]).as_quat()]
                q=Rotation.from_euler('ZYX',[.2,.03*i,-.01*i]).as_quat().astype('f4') if i%3 else None
                if q is not None:f.add_imu(i+1.-.01,q);f.add_imu(i+1.+.01,q)
                body,ground=ref.process(p,i+1.,pose,q);actual=f.process(p,i+1.,pose)
                for a,b in [(body,f.cloud(body=True)),(ground,actual)]:
                    assert len(a)==len(b),(accumulate,casting,parallel,i,len(a),len(b))
                    error=float(np.max(cKDTree(a).query(b)[0])) if len(a) else 0.;max_error=max(max_error,error)
                    assert error<1e-4,(accumulate,casting,parallel,i,error)
                expected=build_height_map(ground,pitch=f.pitch)
                mask=obstacle_mask(*expected[:3],f.policy)
                assert np.array_equal(mask,f.height_map()['mask']),(accumulate,casting,parallel,i,'obstacle cells differ')
                cases+=1
    return dict(reference_sha256=IDENTITY,sync_frames=cases,configurations=4,downstream_obstacle_masks_equal=cases,max_nearest_point_error_m=max_error,
                scope='original syncCallback ordering/transforms; PCL/container I/O delegated; IMU/ray decisions independently differential-tested')
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--elf',required=True);p.add_argument('--output',required=True);a=p.parse_args()
    result=compare(a.elf);Path(a.output).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
