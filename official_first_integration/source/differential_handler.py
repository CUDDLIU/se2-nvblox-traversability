"""Execute original handler AArch64 kernels offline; only libc/libm are stubbed.

No DDS or actuators. Optional Unicorn test dependency, explicit hash-checked ELF.
Full handler timer/transport is outside this kernel comparison.
"""
import argparse,ctypes as C,json,math,struct
from pathlib import Path
import numpy as np
from unicorn.arm64_const import *
from differential_heightmap import Oracle
from height_frontend import library,ptr

IDENTITY='f6310a90732a34f54ace7e211a5deac8dc170481a494a8fc2605b8439454f21d'
D=C.POINTER(C.c_double);F=C.POINTER(C.c_float)
def dp(a):return a.ctypes.data_as(D)

def kernels():
    lib=library()
    lib.handler_boundary.argtypes=[C.c_float]*3+[F]
    lib.handler_step.argtypes=[D,D,C.c_double]
    lib.handler_simpson.argtypes=[D,D,C.c_double,D]
    lib.handler_judge.argtypes=[D,C.c_int,F,C.c_int,D,F,D];lib.handler_judge.restype=C.c_int
    lib.handler_limits.argtypes=[C.c_float,C.c_double,D,D,C.c_double,C.c_double,D,D,D]
    lib.handler_cost_distance.argtypes=[C.c_double];lib.handler_cost_distance.restype=C.c_double
    lib.handler_select.argtypes=[F,C.c_int,D,D,D,D,D,D,D,C.POINTER(C.c_int)];lib.handler_select.restype=C.c_int
    return lib

class HandlerOracle(Oracle):
    def __init__(self,elf):
        super().__init__(elf,IDENTITY,0x106fe8);self.heap=0x500000
        self.uc.mem_map(0x800000,0x4000000)
        self.libm=C.CDLL('libm.so.6')
        for name in ('atan2f','sinf','sqrtf'):
            f=getattr(self.libm,name);f.argtypes=[C.c_float]*(2 if name=='atan2f' else 1);f.restype=C.c_float
    def alloc(self,n):
        p=self.heap;self.heap+=(max(n,16)+15)//16*16
        if self.heap>=0x4800000:raise AssertionError('oracle heap exhausted')
        return p
    def reg(self,index,value,single=False):
        reg=(UC_ARM64_REG_S0 if single else UC_ARM64_REG_D0)+index
        self.uc.reg_write(reg,struct.unpack('<I' if single else '<Q',struct.pack('<f' if single else '<d',value))[0])
    def val(self,index,single=False):
        reg=(UC_ARM64_REG_S0 if single else UC_ARM64_REG_D0)+index
        return struct.unpack('<f' if single else '<d',struct.pack('<I' if single else '<Q',self.uc.reg_read(reg)))[0]
    def hook(self,uc,a,size,context):
        x0=uc.reg_read(UC_ARM64_REG_X0);x1=uc.reg_read(UC_ARM64_REG_X1)
        x2=uc.reg_read(UC_ARM64_REG_X2)
        if a in (0x98b00,0x96b60):uc.reg_write(UC_ARM64_REG_X0,self.alloc(x0))
        elif a in (0x988d0,0x96b50,0xb4168,0x97120,0x96fa0,0x96fc0):pass
        elif a in (0x99190,0x98f40):uc.reg_write(UC_ARM64_REG_X0,0)
        elif a==0x969c0:uc.mem_write(x0,bytes([x1&255])*x2)
        elif a in (0x96b00,0x967d0):
            if x2:uc.mem_write(x0,bytes(uc.mem_read(x1,x2)))
        elif a==0xb5360:uc.mem_write(x0,bytes(144))
        elif a==0xbd380: # Eigen vector push_back: storage only, original arithmetic runs.
            start=self.get(x0,'Q');end=self.get(x0+8,'Q');size=(end-start)//16
            buf=self.alloc((size+1)*16)
            if size:uc.mem_write(buf,bytes(uc.mem_read(start,size*16)))
            src=self.get(x2,'Q');n=self.get(x2+8,'Q');data=self.alloc(n*8)
            uc.mem_write(data,bytes(uc.mem_read(src,n*8)))
            self.put(buf+size*16,'QQ',data,n);self.put(x0,'QQQ',buf,buf+(size+1)*16,buf+(size+1)*16)
        elif a==0xba2d0: # assignment only; old buffers intentionally live for oracle lifetime
            uc.mem_write(x0,bytes(uc.mem_read(x1,24)))
        elif a==0x985b0:
            v=self.val(0);self.put(x0,'d',math.sin(v));self.put(x1,'d',math.cos(v))
        elif a==0x98e00:self.reg(0,math.cos(self.val(0)))
        elif a in (0x97ee0,0x97b60,0x98580):
            name={0x97ee0:'atan2f',0x97b60:'sinf',0x98580:'sqrtf'}[a]
            args=[self.val(0,True)]+([self.val(1,True)] if name=='atan2f' else [])
            self.reg(0,getattr(self.libm,name)(*args),True)
        elif a==0x99440:raise AssertionError('stack guard failed')
        else:return
        uc.reg_write(UC_ARM64_REG_PC,uc.reg_read(UC_ARM64_REG_LR))
    def boundary(self,shape):
        self.heap=0x500000
        for i,v in enumerate(shape[:3]):self.reg(i,v,True)
        self.uc.reg_write(UC_ARM64_REG_X8,0x1fd400);self.run(0x9b100)
        p=self.get(0x1fd400,'Q');return np.frombuffer(self.uc.mem_read(p,1440),'<f4').copy()
    def step(self,state,velocity,dt,simpson=False,measured=None):
        self.put(0x1fd000,'B',int(simpson));self.put(0x1fd100,'QQ',0x600000,6)
        self.uc.mem_write(0x600000,np.asarray(state,'<f8').tobytes())
        self.put(0x1fd200,'QQQ',0x610000,0x610018,0x610018)
        self.uc.mem_write(0x610000,np.asarray(velocity,'<f8').tobytes())
        if measured is not None:self.put(self.obj+0x1d0,'ddd',*measured)
        self.uc.reg_write(UC_ARM64_REG_X3,0x1fd200);self.uc.reg_write(UC_ARM64_REG_X8,0x1fd400)
        self.reg(0,dt);self.run(0x9b670)
        p=self.get(0x1fd400,'Q');return np.frombuffer(self.uc.mem_read(p,48),'<f8').copy()
    def judge(self,states,points,shape,table):
        self.put(self.obj+0x338,'ddd',*shape[:3]);self.put(self.obj+0x3a8,'d',shape[3])
        self.put(self.obj+0x3b0,'QQQ',0x620000,0x6205a0,0x6205a0)
        self.uc.mem_write(0x620000,table.tobytes())
        self.put(0x1fd000,'QQQ',0x600000,0x600000+16*len(states),0x600000+16*len(states))
        for i,s in enumerate(states):
            self.put(0x600000+16*i,'QQ',0x610000+48*i,6)
            self.uc.mem_write(0x610000+48*i,np.asarray(s,'<f8').tobytes())
        self.put(0x1fd100,'QQ',0x630000,0)
        self.put(0x630030,'QQQ',0x640000,0x640000+16*len(points),0x640000+16*len(points))
        if len(points):self.uc.mem_write(0x640000,np.column_stack([points,np.ones(len(points))]).astype('<f4').tobytes())
        self.put(0x1fd200,'B',1);self.put(0x1fd300,'d',1000.)
        self.uc.reg_write(UC_ARM64_REG_X3,0x1fd200);self.uc.reg_write(UC_ARM64_REG_X4,0x1fd300)
        self.run(0x9b7f0);return self.get(0x1fd200,'B'),self.get(0x1fd300,'d')
    def limits(self,d,pitch,maxima,minima,d1,d2,s1,s2):
        self.put(self.obj+0x3c8,'dddd',d1,*s1);self.put(self.obj+0x418,'dddd',d2,*s2)
        pairs=np.column_stack([maxima,minima]).astype('<f8')
        self.uc.mem_write(self.obj+0x468,pairs.tobytes());self.uc.mem_write(self.obj+0x498,pairs.tobytes())
        # Run the actual coefficient arithmetic in InitParams (no YAML stubs).
        self.uc.reg_write(UC_ARM64_REG_X20,self.obj)
        self.uc.emu_start(0xcc64c,0xcc6f0,count=1000)
        self.put(self.obj+0xa0,'d',pitch);self.put(self.obj+0x66c,'B',0)
        self.reg(0,d,True);self.run(0x9bb98)
        return np.frombuffer(self.uc.mem_read(self.obj+0x468,48),'<f8').copy()

    def select(self,points,desired,measured,bounds,cfg):
        self.uc.mem_write(self.obj,bytes(0x1000));self.heap=0x800000
        table=self.boundary(cfg);self.heap=0x800000
        self.put(self.obj+0x338,'ddd',*cfg[:3]);self.put(self.obj+0x3a8,'d',cfg[3])
        self.put(self.obj+0x350,'di',cfg[4],int(cfg[5]));self.put(self.obj+0x4f8,'B',int(cfg[6]))
        self.put(self.obj+0x4c8,'ddd',*cfg[7:10]);self.put(self.obj+0x4e0,'ddd',*cfg[10:13])
        self.put(self.obj+0x500,'dddd',*cfg[13:17]);self.put(self.obj+0xa8,'ddd',*desired)
        self.put(self.obj+0x1d0,'ddd',*measured);self.put(self.obj+0x468,'dddddd',*bounds)
        self.put(self.obj+0x3b0,'Q',0x620000);self.uc.mem_write(0x620000,table.tobytes())
        self.put(self.obj+0x2a8,'Q',0x630000);self.uc.mem_write(0x630000,bytes(144))
        self.put(0x630030,'QQQ',0x640000,0x640000+16*len(points),0x640000+16*len(points))
        if len(points):self.uc.mem_write(0x640000,np.column_stack([points,np.ones(len(points))]).astype('<f4').tobytes())
        self.put(0x1fd000,'QQ',0x650000,0);self.uc.mem_write(0x650000,bytes(144))
        self.put(0x650030,'QQQ',0x4000000,0x4000000,0x4400000)
        self.put(0x1fd100,'QQ',0x610000,3);self.put(0x1fd200,'QQQ',0,0,0)
        self.uc.reg_write(UC_ARM64_REG_X3,0x1fd200);self.uc.reg_write(UC_ARM64_REG_X4,0x1fd300)
        # Same ABI as Oracle.run, with a larger budget for the candidate loops.
        self.uc.reg_write(UC_ARM64_REG_SP,0x3ff000);self.uc.reg_write(UC_ARM64_REG_X0,self.obj)
        self.uc.reg_write(UC_ARM64_REG_X1,0x1fd000);self.uc.reg_write(UC_ARM64_REG_X2,0x1fd100)
        self.uc.reg_write(UC_ARM64_REG_LR,self.end)
        self.uc.emu_start(0x9edb8,self.end,count=50000000)
        if self.uc.reg_read(UC_ARM64_REG_PC)!=self.end:raise AssertionError(hex(self.uc.reg_read(UC_ARM64_REG_PC)))
        valid=self.uc.reg_read(UC_ARM64_REG_X0)
        velocity=np.frombuffer(self.uc.mem_read(0x610000,24),'<f8').copy()
        distance=self.get(0x1fd300,'d')
        accepted=(self.get(0x650038,'Q')-0x4000000)//(16*int(cfg[5]))
        return valid,velocity,distance,accepted

def compare(elf):
    ref=HandlerOracle(elf);lib=kernels();rng=np.random.default_rng(1014)
    boundaries=steps=judgements=limitcases=0;maxstep=0.;maxclear=0.;maxlimits=0.
    for i in range(20):
        shape=np.array([.4,.4,.4,1.5] if i==0 else [*rng.uniform(.2,.7,3),rng.uniform(.8,2.)])
        expected=ref.boundary(shape);table=np.empty(360,np.float32)
        lib.handler_boundary(*shape[:3],ptr(table));np.testing.assert_allclose(table,expected,atol=6e-8,rtol=2e-7);boundaries+=1
        for j in range(15):
            state=rng.uniform(-1,1,6);vel=rng.uniform(-2,2,3);measured=rng.uniform(-2,2,3);dt=rng.uniform(.05,.3)
            for use_simpson in (False,True):
                actual=state.copy()
                if use_simpson:lib.handler_simpson(dp(measured),dp(vel),dt,dp(actual))
                else:lib.handler_step(dp(actual),dp(vel),dt)
                native=ref.step(state,vel,dt,use_simpson,measured)
                np.testing.assert_allclose(actual,native,rtol=3e-14,atol=3e-14);maxstep=max(maxstep,float(np.max(np.abs(actual-native))));steps+=1
            trajectory=np.zeros((6,6),np.float64);state=np.zeros(6)
            for k in range(6):lib.handler_step(dp(state),dp(vel),.15);trajectory[k]=state
            points=rng.uniform(-2,2,(j*2,3)).astype('f4')
            native=ref.judge(trajectory,points,shape,expected);distance=C.c_double(1000.)
            valid=lib.handler_judge(dp(trajectory),6,ptr(points),len(points),dp(shape),ptr(table),C.byref(distance))
            assert valid==native[0],(i,j,'valid',valid,native)
            np.testing.assert_allclose(distance.value,native[1],rtol=1e-6,atol=1.2e-7)
            maxclear=max(maxclear,abs(distance.value-native[1]));judgements+=1
    maxima=np.array([2.,1.,2.]);minima=-maxima;s1=np.array([.6,.4,.6]);s2=np.array([.4,.2,.4])
    for d in [0.,.01,.249999,.25,.250001,.3,.7,.999999,1.,1.001,2.]:
        for pitch in [-45.,-15.,0.,20.,45.]:
            out=np.empty(6);lib.handler_limits(d,pitch,dp(maxima),dp(minima),1.,.25,dp(s1),dp(s2),dp(out))
            native=ref.limits(d,pitch,maxima,minima,1.,.25,s1,s2)
            np.testing.assert_allclose(out,native,rtol=1e-14,atol=1e-14);maxlimits=max(maxlimits,float(np.max(np.abs(out-native))));limitcases+=1
    selection_cases=0;candidate_count=0
    for i in range(24):
        cfg=np.array([.4,.4,.4,1.5,.15,6,i%2,.8,.6,.8,.2,.2,.2,.2,.02,.02,.02])
        maxima=np.array([2.,1.,2.]) if i==0 else rng.uniform(.2,.7,3)
        bounds=np.column_stack([maxima,-maxima]).ravel()
        measured=np.zeros(3) if i==0 else rng.uniform(-1,1,3)
        desired=rng.uniform(-1,1,3)
        points=rng.uniform(-1.5,1.5,(i%8*3,3)).astype('f4')
        if i==23:points=np.array([[0.,0.,0.]],np.float32);bounds[:]=[0.,0.,0.,0.,0.,0.]
        native=ref.select(points,desired,measured,bounds,cfg)
        v=np.empty(3);d=C.c_double();cost=C.c_double();n=C.c_int()
        valid=lib.handler_select(ptr(points),len(points),dp(desired),dp(measured),dp(bounds),dp(cfg),dp(v),C.byref(d),C.byref(cost),C.byref(n))
        assert valid==native[0] and n.value==native[3],(i,valid,n.value,native)
        np.testing.assert_allclose(v,native[1],rtol=1e-13,atol=1e-13)
        np.testing.assert_allclose(d.value,native[2],rtol=1e-13,atol=1e-13)
        candidate_count+=n.value;selection_cases+=1
    return dict(selection_cases=selection_cases,accepted_candidates=candidate_count,reference_sha256=IDENTITY,boundary_tables=boundaries,kinematic_steps=steps,path_judgements=judgements,velocity_limit_cases=limitcases,max_state_error=maxstep,max_clearance_error=maxclear,max_limit_error=maxlimits,passed=True,scope='boundary, Euler/Simpson kinematics, judgePath, InitParams coefficients and ChangeVelBoundary; full CalAvoidance with cloud transform disabled; not timer/transport')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--elf',required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=compare(a.elf);a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
