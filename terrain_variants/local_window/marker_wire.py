"""Bulk CDR1 cube markers without thousands of Python Point allocations."""
import struct
import numpy as np
from angular_display import orange_color

class Writer:
    def __init__(self):self.data=bytearray(b'\x00\x01\x00\x00')
    def align(self,n):self.data.extend(b'\x00'*(-(len(self.data)-4)%n))
    def pack(self,fmt,*values):
        self.align(8 if fmt[-1]=='d' else 1 if fmt[-1]=='B' else 4)
        self.data.extend(struct.pack('<'+fmt,*values))
    def string(self,value):
        data=value.encode()+b'\x00';self.pack('I',len(data));self.data.extend(data)
    def header(self,stamp,frame):self.pack('iI',*stamp);self.string(frame)

def cube_markers(frame,stamp,groups,resolution,dz,modern=True):
    w=Writer();w.pack('I',len(groups))
    for name,rows,color in groups:
        w.header(stamp,frame);w.string(name);w.pack('iii',0,6,0 if rows else 2)
        w.pack('7d',0,0,0,0,0,0,1);w.pack('3d',resolution,resolution,.012)
        w.pack('4f',*color);w.pack('iI',0,0);w.pack('B',0)
        w.pack('I',len(rows))
        if rows:
            xyz=np.asarray([row[:3] for row in rows],dtype='<f8').reshape(-1,3)
            xyz[:,:2]=(xyz[:,:2]+.5)*resolution;xyz[:,2]=xyz[:,2]*dz+.025
            w.align(8);w.data.extend(xyz.tobytes())
        if rows and len(rows[0])>3:
            w.pack('I',len(rows))
            w.data.extend(np.asarray([orange_color(row[3]) for row in rows],dtype='<f4').tobytes())
        else:w.pack('I',0) # uniform-color legacy/green/history markers
        if modern:
            w.string('');w.header((0,0),'');w.string('');w.pack('I',0);w.pack('I',0)
        w.string('');w.string('')
        if modern:w.string('');w.pack('I',0)
        w.pack('B',0)
    return bytes(w.data)
