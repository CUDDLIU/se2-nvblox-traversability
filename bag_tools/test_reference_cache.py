import struct
import unittest
from reference_cache import MeshCache
from mesh_wire import decode_mesh


def wire(keys, deleted=(), clear=False, endian='<'):
    def pack(fmt, *args):return struct.pack(endian+fmt,*args)
    raw=b'\0\1\0\0' if endian=='<' else b'\0\0\0\0'
    frame=b'nvblox_odom\0'
    raw+=pack('iII',1,2,len(frame))+frame
    raw+=b'\0'*(-len(raw)%4)
    raw+=pack('fI',.4,len(keys))+b''.join(pack('iii',*k) for k in keys)+pack('I',len(keys))
    for key in keys:
        if key in deleted:
            raw+=pack('IIII',0,0,0,0)
        else:
            raw+=pack('I9f',3,0,0,0,1,0,0,0,1,0)+pack('II',0,0)+pack('I3i',3,0,1,2)
    return raw+bytes([int(clear)])


class CacheTests(unittest.TestCase):
    def test_full_snapshot_includes_old_blocks_and_deletes(self):
        for endian in ('<','>'):
            cache=MeshCache()
            a,b,c=(0,0,0),(1,0,0),(2,0,0)
            cache.apply(wire([a,b],clear=True,endian=endian))
            cache.apply(wire([a,c],deleted=[a],endian=endian))
            snapshot=decode_mesh(cache.snapshot())
            self.assertTrue(snapshot.clear)
            self.assertEqual([(i.x,i.y,i.z) for i in snapshot.block_indices],[b,c])
            self.assertEqual(snapshot.blocks[0].vertices_array.shape,(3,3))

    def test_clear_and_bad_data_do_not_reuse_previous_map(self):
        cache=MeshCache()
        cache.apply(wire([(0,0,0)]))
        cache.apply(wire([],clear=True))
        self.assertFalse(decode_mesh(cache.snapshot()).blocks)
        with self.assertRaises(ValueError):cache.apply(b'bad')


if __name__=='__main__':unittest.main()
