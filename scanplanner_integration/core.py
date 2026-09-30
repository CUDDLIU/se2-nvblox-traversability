"""ROS-free route and M20 wire primitives, shared with integration tests."""
import json
import math
import struct
import time
import numpy as np


def wrap(value):
    return (np.asarray(value) + np.pi) % (2*np.pi) - np.pi


class Route:
    """Ordered SE(2) route in 3-D: no XY-only nearest point or floor jumping."""
    def __init__(self, poses):
        self.poses = np.asarray(poses, dtype=float).reshape(-1, 4)
        if not len(self.poses) or not np.isfinite(self.poses).all():
            raise ValueError('Route is empty or non-finite')
        self.index = 0

    def window(self, body, height, horizon=1.5):
        floor = np.asarray(body, dtype=float).copy()
        floor[2] -= height
        # Advance in order only. Retain pure rotations until the requested yaw
        # is reached; they must not turn into a zero-duration polynomial.
        while self.index+1 < len(self.poses):
            a, b = self.poses[self.index:self.index+2]
            d = np.linalg.norm(b[:3]-floor[:3])
            rotation = np.linalg.norm(b[:3]-a[:3]) < .025
            if d < .12 and (not rotation or abs(float(wrap(b[3]-floor[3]))) < .12):
                self.index += 1
                continue
            if not rotation:
                v = b[:3]-a[:3]
                u = (floor[:3]-a[:3])@v / max(1e-12, v@v)
                distance = np.linalg.norm(floor[:3]-(a[:3]+np.clip(u,0,1)*v))
                if u >= 1 and distance < .18:
                    self.index += 1
                    continue
            break
        first = self.poses[self.index].copy()
        if self.index+1 < len(self.poses):
            b = self.poses[self.index+1]; v=b[:3]-first[:3]
            if v@v > .025**2:
                u=np.clip((floor[:3]-first[:3])@v/(v@v),0,1)
                first[:3]+=u*v;first[3]+=u*float(wrap(b[3]-first[3]))
        points = [first]
        length = 0.
        turn = False
        for b in self.poses[self.index+1:]:
            a = points[-1]
            distance = float(np.linalg.norm(b[:3]-a[:3]))
            if distance < .025 and abs(float(wrap(b[3]-a[3]))) > .05:
                if len(points) == 1 and np.linalg.norm(a[:3]-floor[:3]) < .12:
                    points.append(b.copy()); turn = True
                break
            if length+distance > horizon:
                u = (horizon-length)/max(distance,1e-12)
                q = a+u*(b-a); q[3] = a[3]+u*float(wrap(b[3]-a[3]))
                points.append(q)
                break
            points.append(b.copy()); length += distance
        result = np.asarray(points)
        result[:,2] += height
        return result, turn

    def distance(self, floor):
        p = np.asarray(floor)[:3]
        a = self.poses[self.index,:3]
        if self.index+1 == len(self.poses):
            return float(np.linalg.norm(p-a))
        b = self.poses[self.index+1,:3]; v=b-a
        u=np.clip((p-a)@v/max(1e-12,v@v),0,1)
        return float(np.linalg.norm(p-a-u*v))


def sample_bspline(points, knots, order, spacing=.07):
    """De Boor evaluation without SciPy; require a finite, nondegenerate spline."""
    p = np.asarray(points, dtype=float); k=np.asarray(knots,dtype=float)
    if order != 3 or p.ndim != 2 or p.shape[1] != 3 or len(p)<4 or len(k)!=len(p)+4:
        raise ValueError('invalid spline dimensions')
    if not np.isfinite(p).all() or not np.isfinite(k).all() or np.any(np.diff(k)<0):
        raise ValueError('non-finite or unordered spline')
    lo,hi=k[order],k[-order-1]
    if hi<=lo or hi-lo>120:
        raise ValueError('invalid spline duration')
    count=max(2,math.ceil(np.linalg.norm(np.diff(p,axis=0),axis=1).sum()/spacing)+1)
    if count>4000:raise ValueError('unbounded local spline')
    result=[]
    for t in np.linspace(lo,hi,count):
        i=min(len(p)-1,max(order,int(np.searchsorted(k,t,side='right')-1)))
        d=p[i-order:i+1].copy()
        for r in range(1,order+1):
            for j in range(order,r-1,-1):
                lower,upper=k[i-order+j],k[i+j-r+1]
                a=(t-lower)/(upper-lower) if upper>lower else 0.
                d[j]=(1-a)*d[j-1]+a*d[j]
        result.append(d[order])
    return np.asarray(result)


def route_yaws(positions, reference):
    reference=np.asarray(reference);result=[]
    for p in positions:
        best=math.inf;yaw=reference[0,3]
        for a,b in zip(reference,reference[1:]):
            v=b[:3]-a[:3];u=np.clip((p-a[:3])@v/max(1e-12,v@v),0,1)
            d=np.linalg.norm(p-a[:3]-u*v)
            if d<best:best=d;yaw=a[3]+u*float(wrap(b[3]-a[3]))
        result.append(yaw)
    return np.asarray(result)


HEADER=struct.Struct('<4sHHB7x')
MAGIC=bytes.fromhex('eb91eb90')
SPEED_PROFILES={
    'normal':dict(forward=.50,lateral=.25,yaw=.60,lookahead=.50),
    'slow':dict(forward=.15,lateral=.10,yaw=.30,lookahead=.15),
}
def packet(kind,command,items,sequence=0):
    body=json.dumps({'PatrolDevice':{'Type':kind,'Command':command,
        'Time':time.strftime('%Y-%m-%d %H:%M:%S'),'Items':items}},separators=(',',':')).encode()
    if len(body)>65535:raise ValueError('ASDU too large')
    return HEADER.pack(MAGIC,len(body),sequence%65536,1)+body


def velocity_packet(x,y,yaw,sequence=0):
    if not all(math.isfinite(v) for v in (x,y,yaw)):
        raise ValueError('non-finite velocity')
    limits=SPEED_PROFILES['normal']
    if abs(x)>limits['forward'] or abs(y)>limits['lateral'] or abs(yaw)>limits['yaw']:
        raise ValueError('velocity outside configured navigation limits')
    return packet(2,25,dict(X=x,Y=y,Z=0.,Roll=0.,Pitch=0.,Yaw=yaw),sequence)


def decode_packet(data):
    if len(data)<HEADER.size:raise ValueError('truncated header')
    magic,size,seq,fmt=HEADER.unpack_from(data)
    if magic!=MAGIC or fmt!=1 or len(data)!=HEADER.size+size:
        raise ValueError('invalid APDU')
    doc=json.loads(data[HEADER.size:])
    if not isinstance(doc.get('PatrolDevice'),dict):raise ValueError('missing PatrolDevice')
    return seq,doc['PatrolDevice']
