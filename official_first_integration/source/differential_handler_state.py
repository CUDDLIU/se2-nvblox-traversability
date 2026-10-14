"""Original timer/PostProcess/steer decisions; CalAvoidance is a supplied result.

Candidate math is independently covered by differential_handler.py. Transport,
logging, scheduler and output buffer (captured length=1) are test stubs only.
"""
import argparse,itertools,json
from pathlib import Path
import numpy as np
from unicorn.arm64_const import *
from differential_handler import HandlerOracle,IDENTITY
from handler import Handler,Selection

class StateOracle(HandlerOracle):
    def __init__(self,elf):
        super().__init__(elf);self.sent=[];self.oa_calls=[];self.call_success=False
    def hook(self,uc,a,size,context):
        x=[uc.reg_read(r) for r in (UC_ARM64_REG_X0,UC_ARM64_REG_X1,UC_ARM64_REG_X2,UC_ARM64_REG_X3,UC_ARM64_REG_X4)]
        if a==0x9edb8:
            p=self.alloc(24);self.put(x[2],'QQ',p,3);self.put(p,'ddd',*self.selection.velocity)
            self.put(x[4],'d',self.selection.clearance);uc.reg_write(UC_ARM64_REG_X0,self.selection.valid)
        elif a==0xbc2a0:self.sent.append(np.frombuffer(uc.mem_read(x[1],24),'<f8').copy())
        elif a==0x986c0:uc.reg_write(UC_ARM64_REG_X0,1000000000)
        elif a in (0x9ddf8,0xc59f8,0xba908,0xb9c70,0xb9d48,0xb9e20,0x98130,0x97490,0x98470,0x981c0,0x98790):pass
        elif a in (0x984d0,0x97fe0):uc.reg_write(UC_ARM64_REG_X0,0)
        elif a in (0x96d20,0x96c60):self.put(x[0],'q',0)
        elif a in (0x96f80,0x992c0):self.put(x[0],'i',x[1])
        elif a==0x98250:uc.mem_write(x[0],bytes(uc.mem_read(x[1],8)))
        elif a==0x96d40:pass # getter returns reference to the integer in x0
        elif a==0x982b0:
            self.oa_calls.append(bool(self.get(x[1],'i')));uc.reg_write(UC_ARM64_REG_X0,self.call_success)
        else:return super().hook(uc,a,size,context)
        uc.reg_write(UC_ARM64_REG_PC,uc.reg_read(UC_ARM64_REG_LR))
    def tick(self,h,selection,ready,pitch):
        self.selection=selection;self.sent=[];self.oa_calls=[];self.heap=0x800000
        self.uc.mem_write(self.obj,bytes(0x1000))
        # Initialize actual native slope coefficients, then restore current bounds.
        self.limits(1.,pitch,h.initial_maxima,-h.initial_maxima,1.,.25,h.slow1,h.slow2)
        self.put(self.obj+0x498,'dddddd',*np.column_stack([h.maxima,-h.maxima]).ravel())
        self.put(self.obj+0x468,'dddddd',*h.bounds)
        self.put(self.obj+0x18,'Q',0x720000);self.uc.mem_write(0x720000,bytes(100))
        self.put(self.obj+0x50,'BBBBBBBB',h.enabled,h.idle,h.avoiding,0,h.previous_stair_oa,h.stair_oa,h.previous_steer_missing,h.steer_missing)
        self.put(self.obj+0x94,'BBBB',ready,ready,ready,ready);self.put(self.obj+0x7c,'B',ready)
        self.put(self.obj+0x98,'d',.3);self.put(self.obj+0xa8,'ddd',*h.desired)
        self.run(0xc8220)
        result=dict(sent=self.sent,desired=np.frombuffer(self.uc.mem_read(self.obj+0xa8,24),'<f8').copy(),
                    idle=bool(self.get(self.obj+0x51,'B')),avoiding=bool(self.get(self.obj+0x52,'B')),
                    previous_stair_oa=bool(self.get(self.obj+0x54,'B')),oa_calls=self.oa_calls)
        self.run(0x9afb0);result['state']=self.uc.reg_read(UC_ARM64_REG_X0)
        return result

def compare(elf):
    ref=StateOracle(elf);cases=0
    for enabled,ready,zero,valid in itertools.product((False,True),repeat=4):
        for d in (.1,.3,1.):
            for prev_missing,missing in ((False,False),(False,True),(True,True)):
                h=Handler();h.enabled=enabled;h.idle=True;h.avoiding=True
                h.previous_steer_missing=prev_missing;h.steer_missing=missing
                h.desired[:]=0. if zero else [.7,-.3,.8]
                h.stair_oa=True;h.previous_stair_oa=False
                h.set_parameters([1.7,.8,1.4],True)
                selection=Selection(valid,np.array([.456789,-.765432,1.234567]),d,.1,1)
                native=ref.tick(h,selection,ready,20.)
                h.select=lambda *args:selection;calls=[]
                out=h.tick([],np.zeros(3),20.,ready=ready,stair_oa_call=lambda value:calls.append(value))
                if out is None:assert not native['sent']
                else:np.testing.assert_array_equal(out,native['sent'][0])
                np.testing.assert_array_equal(h.desired,native['desired'])
                assert (h.state,h.idle,h.avoiding,h.previous_stair_oa,calls)==(native['state'],native['idle'],native['avoiding'],native['previous_stair_oa'],native['oa_calls']),native
                cases+=1
    rng=np.random.default_rng(111);steer_cases=0
    for i in range(100):
        h=Handler();h.bounds=np.column_stack([rng.uniform(.1,2,3),-rng.uniform(.1,2,3)]).ravel()
        v=rng.uniform(-2,2,3);h.process_steer(v)
        ref.put(ref.obj+0x468,'dddddd',*h.bounds);ref.put(ref.obj+0x668,'B',0);ref.put(0x1fd000,'ddd',*v)
        ref.run(0x9b2c0)
        np.testing.assert_array_equal(h.desired,np.frombuffer(ref.uc.mem_read(ref.obj+0xa8,24),'<f8'))
        steer_cases+=1
    return dict(reference_sha256=IDENTITY,timer_cases=cases,steer_cases=steer_cases,passed=True,scope='RunTimerFunc, PostProcess, GetState, ProcessVel, CheckStairOaEnabledState; injected CalAvoidance result, transport/scheduler stubs, output buffer length 1')
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--elf',required=True);p.add_argument('--output',required=True,type=Path);a=p.parse_args()
    result=compare(a.elf);a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
