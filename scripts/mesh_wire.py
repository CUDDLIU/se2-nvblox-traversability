"""Read the release-3.2 nvblox Mesh CDR without constructing Point objects.

Only vertices and triangle indices are materialized; normals/colors are skipped.
Arrays own their small buffers, so retaining one block never pins a whole message.
Schema and wire bounds are checked explicitly. No deltas or deletions are dropped.
"""
import math
import struct
from types import SimpleNamespace

import numpy as np


def check_schema(mesh, block, index):
    expected = [
        {'header': 'std_msgs/Header', 'block_size_m': 'float',
         'block_indices': 'sequence<nvblox_msgs/Index3D>',
         'blocks': 'sequence<nvblox_msgs/MeshBlock>', 'clear': 'boolean'},
        {'vertices': 'sequence<geometry_msgs/Point32>',
         'normals': 'sequence<geometry_msgs/Point32>',
         'colors': 'sequence<std_msgs/ColorRGBA>', 'triangles': 'sequence<int32>'},
        {'x': 'int32', 'y': 'int32', 'z': 'int32'},
    ]
    for cls, fields in zip((mesh, block, index), expected):
        if list(cls.get_fields_and_field_types().items()) != list(fields.items()):
            raise ValueError('Unsupported nvblox Mesh schema; wire decoder disabled')


class Reader:
    def __init__(self, raw):
        self.raw = memoryview(raw)
        if len(raw) < 4 or bytes(raw[:2]) not in (b'\x00\x00', b'\x00\x01'):
            raise ValueError('Expected CDR1 Mesh encapsulation')
        self.endian = '<' if raw[1] == 1 else '>'
        self.offset = 4

    def take(self, count, alignment=4):
        start = self.offset + (-(self.offset - 4) % alignment)
        end = start + count
        if count < 0 or end > len(self.raw):
            raise ValueError('Truncated Mesh CDR')
        self.offset = end
        return self.raw[start:end]

    def scalar(self, fmt):
        size = struct.calcsize(fmt)
        return struct.unpack(self.endian + fmt, self.take(size, min(size, 4)))[0]

    def array(self, kind, columns=1, skip=False):
        count = self.scalar('I')
        data = self.take(count * columns * 4)
        if skip:
            return count
        # Copy with native scalar type, including a correct big-endian conversion.
        a = np.frombuffer(data, dtype=self.endian+kind).astype(kind, copy=True)
        return a.reshape(count, columns) if columns != 1 else a


def decode_mesh(raw):
    r = Reader(raw)
    sec, nanosec = r.scalar('i'), r.scalar('I')
    count = r.scalar('I')
    frame = bytes(r.take(count, 1))
    if not frame or frame[-1] != 0 or b'\x00' in frame[:-1]:
        raise ValueError('Invalid Mesh frame string')
    header = SimpleNamespace(frame_id=frame[:-1].decode('utf-8'),
                             stamp=SimpleNamespace(sec=sec, nanosec=nanosec))
    size = r.scalar('f')
    if not math.isfinite(size) or size <= 0:
        raise ValueError('Invalid Mesh block size')
    indices = r.array('i4', 3)
    count = r.scalar('I')
    if count != len(indices):
        raise ValueError('Mesh block/index count mismatch')
    blocks = []
    for _ in range(count):
        vertices = r.array('f4', 3)
        normals = r.array('f4', 3, skip=True)
        colors = r.array('f4', 4, skip=True)
        triangles = r.array('i4')
        if normals not in (0, len(vertices)) or colors not in (0, len(vertices)):
            raise ValueError('Mesh attribute count mismatch')
        if len(triangles) % 3:
            raise ValueError('Incomplete Mesh triangle')
        blocks.append(SimpleNamespace(vertices_array=vertices, triangles_array=triangles.reshape(-1, 3)))
    clear = r.scalar('B')
    if clear not in (0, 1):
        raise ValueError('Invalid Mesh clear flag')
    # DDS can append at most three bytes of encapsulation padding.
    if len(raw) - r.offset > 3:
        raise ValueError('Unexpected data after Mesh CDR')
    return SimpleNamespace(header=header, block_size_m=size, clear=bool(clear), blocks=blocks,
                           block_indices=[SimpleNamespace(x=int(x), y=int(y), z=int(z))
                                          for x, y, z in indices])


def block_arrays(block):
    if hasattr(block, 'vertices_array'):
        return (np.asarray(block.vertices_array, dtype=np.float64),
                np.asarray(block.triangles_array, dtype=np.int64))
    return (np.array([(p.x, p.y, p.z) for p in block.vertices], dtype=np.float64).reshape(-1, 3),
            np.asarray(block.triangles, dtype=np.int64).reshape(-1, 3))


def mesh_summary(raw):
    """Count geometry versus deletion messages without constructing arrays."""
    r=Reader(raw)
    sec,nanosec=r.scalar('i'),r.scalar('I')
    r.take(r.scalar('I'),1);r.scalar('f')
    indices=r.array('i4',3,skip=True)
    count=r.scalar('I')
    if count!=indices:raise ValueError('Mesh block/index count mismatch')
    vertices=triangles=0
    for _ in range(count):
        vertices+=r.array('f4',3,skip=True)
        r.array('f4',3,skip=True);r.array('f4',4,skip=True)
        triangles+=r.array('i4',skip=True)//3
    return dict(stamp=sec+nanosec*1e-9,blocks=count,vertices=vertices,
                triangles=triangles,clear=bool(r.scalar('B')),bytes=len(raw))


def recolor_mesh(raw, low, high, palette, *, cyclic=False, contour_spacing=0.):
    """Replace only color sequences in CDR; retain every geometry/deletion byte."""
    if not math.isfinite(low) or not math.isfinite(high) or high<=low:
        raise ValueError('Invalid color height range')
    r=Reader(raw)
    r.scalar('i');r.scalar('I');r.take(r.scalar('I'),1);r.scalar('f')
    indices=r.array('i4',3,skip=True)
    count=r.scalar('I')
    if count!=indices:raise ValueError('Mesh block/index count mismatch')
    pieces=[];last=0
    for _ in range(count):
        nv=r.scalar('I')
        points=np.frombuffer(r.take(nv*12),dtype=r.endian+'f4').reshape(-1,3)
        normals=r.array('f4',3,skip=True)
        color_start=r.offset
        colors=r.array('f4',4,skip=True)
        if normals not in (0,nv) or colors not in (0,nv):
            raise ValueError('Mesh attribute count mismatch')
        # Match the original scalar double arithmetic before integer truncation.
        heights=points[:,2].astype(np.float64)
        phase=(heights-low)/(high-low)
        if cyclic:
            bins=np.floor(np.mod(phase,1.)*len(palette)).astype(np.int64)
        else:
            bins=np.clip((len(palette)-1)*phase,0,len(palette)-1).astype(np.int64)
        rgba=palette[bins].copy()
        if contour_spacing>0:
            # Fixed world-height contours remain consistent across mesh deltas.
            distance=np.minimum(np.mod(heights,contour_spacing),
                                contour_spacing-np.mod(heights,contour_spacing))
            shade=.62+.38*np.clip(distance/(contour_spacing*.18),0,1)
            rgba[:,:3]*=shade[:,None]
        pieces.extend((memoryview(raw)[last:color_start],struct.pack(r.endian+'I',nv),
                       np.asarray(rgba,dtype=r.endian+'f4').tobytes()))
        last=r.offset
        if r.array('i4',skip=True)%3:raise ValueError('Incomplete Mesh triangle')
    if r.scalar('B') not in (0,1) or len(raw)-r.offset>3:
        raise ValueError('Invalid Mesh trailer')
    pieces.append(memoryview(raw)[last:])
    return b''.join(pieces)
