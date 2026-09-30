import struct
import unittest

import numpy as np

from mesh_wire import decode_mesh, block_arrays, recolor_mesh,mesh_summary,Reader


def wire(endian='<',clear=True,frame='nvblox_odom'):
    data=bytearray(b'\x00\x01\x00\x00' if endian=='<' else b'\x00\x00\x00\x00')
    def put(fmt,*args):
        data.extend(b'\x00'*(-(len(data)-4)%min(struct.calcsize(fmt),4)))
        data.extend(struct.pack(endian+fmt,*args))
    put('i',19);put('I',23)
    encoded=frame.encode()+b'\x00'
    put('I',len(encoded));data.extend(encoded)
    put('f',.4)
    put('I',2);put('iii',-1,2,3);put('iii',9,-5,2)
    put('I',2)
    put('I',3);put('9f',0,0,0,1,0,0,0,1,0)
    put('I',3);put('9f',*([0,0,1]*3))
    put('I',3);put('12f',*([1,0,0,1]*3))
    put('I',3);put('3i',0,1,2)
    for _ in range(4):put('I',0)
    put('B',int(clear))
    return bytes(data)


class MeshWireTests(unittest.TestCase):
    def test_recolor_preserves_all_geometry_and_matches_scalar_colors(self):
        import colorsys
        palette=np.array([(*colorsys.hsv_to_rgb(.78-.62*i/255,.9,.95),1.) for i in range(256)],dtype=np.float32)
        for endian in ('<','>'):
            raw=wire(endian)
            colored=recolor_mesh(raw,-.7,1.8,palette)
            original=decode_mesh(raw);result=decode_mesh(colored)
            self.assertEqual(mesh_summary(raw),mesh_summary(colored))
            for a,b in zip(original.blocks,result.blocks):
                np.testing.assert_array_equal(a.vertices_array,b.vertices_array)
                np.testing.assert_array_equal(a.triangles_array,b.triangles_array)
            r=Reader(colored);r.scalar('i');r.scalar('I');r.take(r.scalar('I'),1);r.scalar('f')
            r.array('i4',3,skip=True);self.assertEqual(r.scalar('I'),2)
            for block in result.blocks:
                points=r.array('f4',3);r.array('f4',3,skip=True);colors=r.array('f4',4);r.array('i4',skip=True)
                expected=np.array([palette[max(0,min(255,int(255*(float(p[2])+.7)/2.5)))]
                                   for p in points],dtype=np.float32).reshape(-1,4)
                np.testing.assert_array_equal(colors,expected)

    def test_endianness_alignment_empty_deletion_and_reset(self):
        for endian in ('<','>'):
            for frame in ('a','ab','abc','abcd','nvblox_odom'):
                msg=decode_mesh(wire(endian,frame=frame))
                self.assertEqual(msg.header.frame_id,frame)
                self.assertEqual((msg.header.stamp.sec,msg.header.stamp.nanosec),(19,23))
                self.assertEqual((msg.block_indices[0].x,msg.block_indices[1].y),(-1,-5))
                self.assertTrue(msg.clear)
                v,t=block_arrays(msg.blocks[0])
                np.testing.assert_array_equal(v,[[0,0,0],[1,0,0],[0,1,0]])
                np.testing.assert_array_equal(t,[[0,1,2]])
                self.assertEqual(block_arrays(msg.blocks[1])[1].shape,(0,3))

    def test_invalid_or_truncated_message_rejected(self):
        data=wire()
        for end in range(len(data)):
            with self.assertRaises(ValueError):decode_mesh(data[:end])
        for bad in (b'\x00\x03'+data[2:],data[:-1]+b'\x02',data+b'garbage'):
            with self.assertRaises(ValueError):decode_mesh(bad)

    def test_retained_block_does_not_pin_raw_buffer(self):
        raw=bytearray(wire())
        msg=decode_mesh(raw)
        raw[:]=b'\x00'*len(raw)
        np.testing.assert_array_equal(msg.blocks[0].vertices_array,[[0,0,0],[1,0,0],[0,1,0]])


if __name__=='__main__':unittest.main()
