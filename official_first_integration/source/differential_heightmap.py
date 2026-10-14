"""Offline AArch64 instruction oracle; never starts a vendor executable.

Requires optional test-only unicorn. ELF path is explicit and hash checked.
Only logging, request access and response setters are stubbed for callbacks.
computeCostMap executes its original instructions with preallocated PCL data.
No networking, DDS, robot sender, or production-runtime imports.
"""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import numpy as np
from unicorn import Uc, UC_ARCH_ARM64, UC_MODE_ARM, UC_HOOK_CODE
from unicorn.arm64_const import (UC_ARM64_REG_PC, UC_ARM64_REG_SP,
                                UC_ARM64_REG_X0, UC_ARM64_REG_X1,
                                UC_ARM64_REG_X2, UC_ARM64_REG_LR)
from heightmap import HeightPolicy, obstacle_mask

IDENTITY = '1376c2a75fc3a9f30089df19f3d773be3c019fe53354c11aa89697f6f821aae9'


class Oracle:
    obj = 0x400000
    end = 0x1ff000

    def __init__(self, elf, identity=IDENTITY, guard=0x101fe8):
        b = Path(elf).read_bytes()
        if hashlib.sha256(b).hexdigest() != identity:
            raise ValueError('unexpected reference ELF')
        self.uc = Uc(UC_ARCH_ARM64, UC_MODE_ARM)
        self.uc.mem_map(0, 0x800000)
        off = struct.unpack_from('<Q', b, 32)[0]
        size, n = struct.unpack_from('<HH', b, 54)
        for i in range(n):
            t, _, pos, va, _, length, _, _ = struct.unpack_from('<IIQQQQQQ', b, off + size*i)
            if t == 1:
                self.uc.mem_write(va, b[pos:pos+length])
        self.put(guard, 'Q', 0x1fe000)  # __stack_chk_guard pointer
        self.put(0x1fe000, 'Q', 0x12345678)
        self.uc.hook_add(UC_HOOK_CODE, self.hook)
        self.command = 0
        self.response = None

    def put(self, address, fmt, *values):
        self.uc.mem_write(address, struct.pack('<'+fmt, *values))

    def get(self, address, fmt):
        return struct.unpack('<'+fmt, self.uc.mem_read(address, struct.calcsize('<'+fmt)))[0]

    def hook(self, uc, address, size, _):
        if address in (0xacbf0, 0xac750, 0xac678):  # logging only
            uc.reg_write(UC_ARM64_REG_PC, uc.reg_read(UC_ARM64_REG_LR))
        elif address == 0x878a0:  # Request::command()
            uc.reg_write(UC_ARM64_REG_X0, self.command & 0xffffffff)
            uc.reg_write(UC_ARM64_REG_PC, uc.reg_read(UC_ARM64_REG_LR))
        elif address == 0x87790:  # Response::result(int)
            self.response = uc.reg_read(UC_ARM64_REG_X1)
            uc.reg_write(UC_ARM64_REG_PC, uc.reg_read(UC_ARM64_REG_LR))
        elif address == 0x89b70:
            raise AssertionError('reference stack guard failed')

    def run(self, address):
        self.uc.reg_write(UC_ARM64_REG_SP, 0x3ff000)
        self.uc.reg_write(UC_ARM64_REG_X0, self.obj)
        self.uc.reg_write(UC_ARM64_REG_X1, 0x1fd000)
        self.uc.reg_write(UC_ARM64_REG_X2, 0x1fd100)
        self.uc.reg_write(UC_ARM64_REG_LR, self.end)
        self.uc.emu_start(address, self.end, count=2000000)
        if self.uc.reg_read(UC_ARM64_REG_PC) != self.end:
            raise AssertionError('reference did not return')

    def policy_init(self, p):
        self.put(self.obj+0xf0, 'B', p.stair_oa)
        self.put(self.obj+0xf4, 'i', p.terrain)
        self.put(self.obj+0x3e4, 'ffff', p.single_battery, p.flat, p.stair, p.height_range)

    def callback(self, kind, command):
        self.command = command
        self.response = None
        self.run(0x8c928 if kind == 'oa' else 0x8caa0)
        return (self.get(self.obj+0xf0, 'B'), self.get(self.obj+0xf4, 'i'),
                self.get(self.obj+0x3f0, 'f'), self.response)

    def cost(self, height, samples, counts, policy):
        self.policy_init(policy)
        nx, ny = height.shape
        self.put(self.obj+0x348, 'f', .55)
        self.put(self.obj+0x3cc, 'ff', -4., -4.)
        self.put(self.obj+0x3d4, 'ii', 2, 2)
        self.put(self.obj+0x410, 'fiii', .1, nx, ny, len(samples))
        self.put(self.obj+0x420, 'QQQ', 0x410000, nx, ny)
        self.put(self.obj+0x438, 'QQQ', 0x420000, nx, ny)
        self.uc.mem_write(0x410000, np.asarray(height, '<f4').tobytes(order='F'))
        self.uc.mem_write(0x420000, np.asarray(counts, '<i4').tobytes(order='F'))
        self.put(self.obj+0x468, 'Q', 0x430000)
        for k, layer in enumerate(samples):
            ptr = 0x440000+k*0x10000
            self.put(0x430000+k*24, 'QQQ', ptr, nx, ny)
            self.uc.mem_write(ptr, np.asarray(layer, '<f4').tobytes(order='F'))
        self.put(self.obj+0x4cd, 'B', 1)
        for slot, cloud, data in ((0x530, 0x650000, 0x660000), (0x540, 0x670000, 0x680000)):
            self.put(self.obj+slot, 'Q', cloud)
            self.put(cloud+0x30, 'QQQ', data, data, data+0x10000)
        self.run(0x8eda8)
        n = (self.get(0x650038, 'Q') - 0x660000)//16
        xyz = np.frombuffer(self.uc.mem_read(0x660000, n*16), '<f4').reshape(-1,4)[:,:3]
        out = np.zeros(height.shape, dtype=bool)
        if len(xyz):
            xy = np.rint((xyz[:,:2]+4.)/.1).astype(int)
            out[xy[:,0],xy[:,1]] = True
        return out


def compare(elf):
    ref = Oracle(elf)
    policies = 0
    # Unequal synthetic thresholds distinguish branches hidden by production's
    # equal .25 flat/stair values; sequence includes unknown IDs and disabling.
    for values in ((.08,.25,.25), (.09,.21,.32)):
        for initial in (1,2,7):
            p = HeightPolicy(*values, terrain=initial, height_range=values[1])
            ref.policy_init(p)
            for kind, cmd in [('oa',1),('terrain',2),('oa',0),('terrain',1),
                              ('terrain',7),('oa',1),('oa',-1),('terrain',2),
                              ('oa',2),('terrain',-1),('oa',0)]:
                response = p.set_stair_oa(cmd) if kind == 'oa' else p.set_terrain(cmd)
                actual = ref.callback(kind, cmd)
                expected = (int(p.stair_oa),p.terrain,p.height_range,response)
                assert actual == expected, (kind,cmd,actual,expected)
                policies += 1
    rng = np.random.default_rng(20261014)
    maps = 0
    for i in range(120):
        shape = (13,15)
        heights = rng.choice([-.9,-.8,-.55,-.47,-.3,0.,.3],shape).astype('f4')
        samples = rng.choice([-.9,-.55,-.47,-.3,0.,.3],(6,*shape)).astype('f4')
        counts = rng.integers(0,7,shape,dtype=np.int32)
        p = HeightPolicy(terrain=1+i%3)
        p.set_stair_oa(i%2)
        actual = ref.cost(heights,samples,counts,p)
        expected = obstacle_mask(heights,samples,counts,p)
        assert np.array_equal(actual,expected), (i,np.argwhere(actual!=expected).tolist())
        maps += 1
    return dict(reference_sha256=IDENTITY, callback_transitions=policies,
                cost_maps=maps, passed=True, scope='callbacks and computeCostMap, finite prebuilt heights, remove_cloud_nav=true')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--elf',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    result = compare(args.elf)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result))
