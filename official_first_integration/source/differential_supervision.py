"""AOS continuous-warning instruction comparison, Go2Point=false."""
import argparse,itertools,json,re
from pathlib import Path
from unicorn.arm64_const import UC_ARM64_REG_X0,UC_ARM64_REG_X1,UC_ARM64_REG_PC,UC_ARM64_REG_LR
from differential_heightmap import Oracle
from differential_gait import IDENTITY
from supervision import WarningTimer,ERRORS

class WarningOracle(Oracle):
    def __init__(self,elf,assembly):
        self.calls={int(a,16) for a in re.findall(r'\tbl\s+0x([a-f0-9]+)',Path(assembly).read_text())}
        super().__init__(elf,IDENTITY,0x228fd0)
    def hook(self,uc,address,size,_):
        if address not in self.calls:return
        if address==0x968c0:v=self.now_ms*1000000
        elif address==0x157150:v=0x450000
        elif address==0x19b6e0:v=ERRORS.get(uc.reg_read(UC_ARM64_REG_X1),0xa34c)
        elif address==0x99460:raise AssertionError('stack guard')
        else:v=0 # Only logging, error-description strings and destructors.
        uc.reg_write(UC_ARM64_REG_X0,v)
        uc.reg_write(UC_ARM64_REG_PC,uc.reg_read(UC_ARM64_REG_LR))
    def check(self,timer,now,warning,localized,fresh):
        self.now_ms=now
        self.put(self.obj+0x628,'B',timer.obstacle_wait)
        self.put(self.obj+0x624,'I',warning)
        self.put(self.obj+0x5a8,'Q',timer.last_good_ms)
        self.put(self.obj+0x4b0,'I',timer.obstruction_seconds)
        self.put(self.obj+0x4b4,'I',timer.localization_seconds)
        self.put(self.obj+0x638,'B',1 if localized else 3)
        self.put(self.obj+0x598,'I',0);self.put(self.obj+0x4a4,'B',0)
        self.put(self.obj+0x4d0,'Q',0x460000)
        self.put(0x450190,'B',fresh)
        self.run(0x19b920)
        return self.get(self.obj+0x598,'I'),self.get(self.obj+0x5a8,'Q')

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--elf',required=True);ap.add_argument('--assembly',required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    oracle=WarningOracle(a.elf,a.assembly);count=0
    for oldflag,warning,elapsed,located,fresh,limits in itertools.product((False,True),(0,1001,1002,1003,1101,1102,2004,2005),(0,29999,30000,30001),(False,True),(False,True),((30,30),(10,20))):
        s=WarningTimer(1000,*limits,obstacle_wait=oldflag)
        # recvPlannerStatus precedes CheckContinuousWarningStatus.
        if warning in (2004,2005):s.obstacle_wait=True
        elif warning==0:s.obstacle_wait=False
        result=oracle.check(s,1000+elapsed,warning,located,fresh)
        s.update(1000+elapsed,warning,localized=located,command_fresh=fresh)
        assert result==(s.error,s.last_good_ms),(oldflag,warning,elapsed,located,fresh,limits,result,s)
        count+=1
    result=dict(reference_sha256=IDENTITY,cases=count,passed=True,scope='CheckContinuousWarningStatus with Go2Point=false; clock, strings and error-map lookup stubbed; timer arithmetic executed')
    a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
