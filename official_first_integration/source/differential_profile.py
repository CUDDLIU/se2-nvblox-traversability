"""Execute SetPlannerParam instructions; stub only C++ I/O and DDS storage.

The TOML loader's array layout is populated from audited init instructions:
terrain FLAT/STAIR/ORIENTEER; speed NORMAL/LOW/HIGH (not LOW/NORMAL/HIGH).
"""
import argparse,itertools,json,re
from pathlib import Path
from unicorn.arm64_const import UC_ARM64_REG_X0,UC_ARM64_REG_X1,UC_ARM64_REG_PC,UC_ARM64_REG_LR
from differential_heightmap import Oracle
from differential_gait import IDENTITY
from task_profile import CONFIG,task_parameters

FIELDS=('maxSpeedX','barking_deceleration','stop_distance','dl_min_yaw','dl_min_x','dl_min_y','dl_line_yaw_min')


class ProfileOracle(Oracle):
    def __init__(self,elf,assembly):
        self.calls={int(a,16):name for a,name in re.findall(r'\tbl\s+0x([a-f0-9]+) <([^>]+)>',Path(assembly).read_text())}
        self.slots={}
        super().__init__(elf,IDENTITY,0x228fd0)
        self.put(0x22a958,'B',1)
        self.put(self.obj+0x4b8,'I',3);self.put(self.obj+0x4c8,'I',5)
        self.put(self.obj+0x4d0,'Q',0x450000)
        for i,k in enumerate(('maxSpeedY','maxTheta','yaw_tolerance','xy_tolerance')):
            self.put(self.obj+0x78+8*i,'d',CONFIG['COMMON'][k])
        for m,mode in enumerate(('DIRECTLINE','AUTO')):
            for g,terrain in enumerate(('FLAT','STAIR','ORIENTEER')):
                for s,speed in enumerate(('NORMAL_SPEED','LOW_SPEED','HIGH_SPEED')):
                    for f,name in enumerate(FIELDS):
                        self.put(self.obj+0x98+504*m+168*g+56*s+8*f,'d',CONFIG[mode][terrain][speed][name])

    def hook(self,uc,address,size,_):
        if address not in self.calls:return
        name=self.calls[address]
        match=re.search(r'SetParam_Request\d+([a-zA-Z_]+)Ev',name)
        if match:
            field=match[1]
            slot=self.slots.setdefault(field,0x460000+16*len(self.slots))
            uc.reg_write(UC_ARM64_REG_X0,slot)
        elif address==0x99460:raise AssertionError('stack guard')
        else:
            # All remaining callees are logging/string storage, constructors,
            # destructors or DDS. Planner selection arithmetic runs unchanged.
            uc.reg_write(UC_ARM64_REG_X0,1 if address==0x14ca60 else 0)
        uc.reg_write(UC_ARM64_REG_PC,uc.reg_read(UC_ARM64_REG_LR))

    def parameters(self,task):
        for key,offset in [('Gait',0x1c),('Speed',0x20),('Manner',0x24),('ObsMode',0x28),('NavMode',0x2c)]:
            self.put(0x1fd000+offset,'i',task[key])
        self.run(0x19f0e0)
        assert self.uc.reg_read(UC_ARM64_REG_X0)==1
        return {k:bool(self.get(a,'B')) if k=='directLine_mode' else bool(self.get(a,'d')) if k in ('judge_close','backward_mode') else self.get(a,'d') for k,a in self.slots.items()}


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--elf',required=True);ap.add_argument('--assembly',required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    oracle=ProfileOracle(a.elf,a.assembly);count=0
    for values in itertools.product((0x3002,0x3003),(0,1),(0,1,2),(0,1),(0,1)):
        task=dict(zip(('Gait','NavMode','Speed','Manner','ObsMode'),values))
        got=oracle.parameters(task);want=task_parameters(task)
        assert got==want,(task,got,want)
        count+=1
    result=dict(reference_sha256=IDENTITY,cases=count,passed=True,scope='SetPlannerParam instruction execution, captured TOML values, stubbed C++ I/O and DDS')
    a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
