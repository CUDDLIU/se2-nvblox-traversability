"""AOS SwitchGait instruction comparison. Sends/waits are captured stubs only."""
import argparse
import itertools
import json
from pathlib import Path
from unicorn.arm64_const import (UC_ARM64_REG_PC, UC_ARM64_REG_SP,
                                UC_ARM64_REG_X0, UC_ARM64_REG_X1,
                                UC_ARM64_REG_X2, UC_ARM64_REG_LR)
from differential_heightmap import Oracle
from gait import GaitSwitch, SUPPORTED

IDENTITY='2faf5921b6772f0abdea0de178b54e88a17dac54ecb0f0a849bd31a169c33d3e'


class GaitOracle(Oracle):
    def __init__(self,elf):
        super().__init__(elf,IDENTITY,0x228fd0)

    def hook(self,uc,address,size,_):
        if address in (0xd6820,0x1578f8,0xd6f40):
            pass  # Logging only.
        elif address==0x1adb48:
            uc.reg_write(UC_ARM64_REG_X0,int(self.lying))
        elif address in (0xde6d8,0x95e30,0x951b0,0x965d0):
            uc.reg_write(UC_ARM64_REG_X0,0x450000)
        elif address==0x14fd30:
            self.sent.append(uc.reg_read(UC_ARM64_REG_X1))
        elif address==0x14e9d8:
            assert uc.reg_read(UC_ARM64_REG_X2)==3000
            uc.reg_write(UC_ARM64_REG_X0,int(self.responses[len(self.sent)-1]))
        elif address==0x99460:
            raise AssertionError('reference stack guard failed')
        else:
            return
        uc.reg_write(UC_ARM64_REG_PC,uc.reg_read(UC_ARM64_REG_LR))

    def switch(self,target,actual,lying,force,responses):
        self.lying=lying;self.responses=responses;self.sent=[]
        self.put(0x450000,'I',actual)
        self.uc.reg_write(UC_ARM64_REG_SP,0x3ff000)
        self.uc.reg_write(UC_ARM64_REG_X0,self.obj)
        self.uc.reg_write(UC_ARM64_REG_X1,target)
        self.uc.reg_write(UC_ARM64_REG_X2,int(force))
        self.uc.reg_write(UC_ARM64_REG_LR,self.end)
        self.uc.emu_start(0x1adbf0,self.end,count=20000)
        assert self.uc.reg_read(UC_ARM64_REG_PC)==self.end
        return self.sent,bool(self.uc.reg_read(UC_ARM64_REG_X0))


def compare(elf):
    ref=GaitOracle(elf);count=0
    for target,matching,lying,force,outcomes in itertools.product(
            sorted(SUPPORTED|{0,0x3005,0xffff}), (False,True),(False,True),
            (False,True),((False,False,False),(True,False,False),
                          (False,True,False),(False,False,True))):
        actual=target if matching else 0x1000
        sent,ok=ref.switch(target,actual,lying,force,outcomes)
        g=GaitSwitch();event=g.begin(target,actual,0,lying=lying,force=force)
        expected=[];time=0
        while event is not None:
            expected.append(event)
            time+=3
            event=g.update(target if outcomes[len(expected)-1] else 0x1000,time)
        assert (sent,ok)==(expected,g.state=='confirmed'),(target,matching,lying,force,outcomes,sent,ok,expected,g.state)
        count+=1
    return dict(reference_sha256=IDENTITY,cases=count,passed=True,
                scope='SwitchGait branching/retries; transport and confirmation waits stubbed; not a physical gait test')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--elf',required=True)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    result=compare(args.elf);args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result))
