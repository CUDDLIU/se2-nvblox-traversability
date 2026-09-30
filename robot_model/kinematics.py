"""Read-only M20 SDK motor-coordinate conversion, scoped to nvblox."""
import math

NAMESPACE='nvblox_robot'
JOINT_NAMES=tuple(f'{NAMESPACE}/{leg}_{joint}_joint'
                  for leg in ('fl','fr','hl','hr') for joint in ('hipx','hipy','knee','wheel'))
# Exact constants and formula from preserved SDK m20_interface.hpp / dds_interface.hpp.
OFFSETS_DEG=(-25,-131,160,0, 25,-131,160,0, -25,131,-160,0, 25,131,-160,0)
DIRECTIONS=(1,1,-1,1, 1,-1,1,-1, -1,1,-1,1, -1,-1,1,-1)


def model_angles(values):
    if len(values)!=16 or any(not math.isfinite(v) for v in values):
        raise ValueError('expected 16 finite protocol values')
    angles=[]
    for i,(value,direction,offset) in enumerate(zip(values,DIRECTIONS,OFFSETS_DEG)):
        if i%4==3:
            angles.append(None) # Reported quantity is wheel SPEED, not angle.
            continue
        angle=value*direction+math.radians(offset)
        # Same modulo-turn choice as SDK ResetPositionOffset for HipY/Knee.
        if i%4 in (1,2):angle=(angle+math.pi)%(2*math.pi)-math.pi
        if abs(angle)>math.radians(45 if i%4==0 else 170):
            raise ValueError(f'implausible calibrated joint {i}: {angle}')
        angles.append(angle)
    return angles


class WheelIntegrator:
    """Relative visualization angle only; do not extrapolate across stale telemetry."""
    def __init__(self):
        self.positions=[0.]*4;self.last_ns=None;self.last_speeds=None
    def update(self,values,stamp_ns):
        speeds=[values[i]*DIRECTIONS[i] for i in (3,7,11,15)]
        if self.last_ns is not None:
            dt=(stamp_ns-self.last_ns)/1e9
            if dt<=0:raise ValueError('out-of-order telemetry timestamp')
            if dt<=.3:
                self.positions=[(p+.5*(a+b)*dt+math.pi)%(2*math.pi)-math.pi
                    for p,a,b in zip(self.positions,self.last_speeds,speeds)]
        self.last_ns=stamp_ns;self.last_speeds=speeds
        return list(self.positions),speeds
