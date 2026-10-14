"""Recovered AOS height-map decision kernels; cloud front end in height_frontend.

Evidence: height_map_nav SHA256 1376c2a75fc3a9f30089df19f3d773be3c019fe53354c11aa89697f6f821aae9.
No robot/ROS/DDS imports. The autonomous raw-cloud runtime uses its own input.
Input elevations must already be in the gravity-aligned body frame. This module
does not substitute route Z for measured height or infer a gait from one step.
"""
from dataclasses import dataclass
import math
import numpy as np


def build_height_map(points, *, shape=(81,81), origin=(-4.,-4.), resolution=.1,
                     retained=30, robot_height=.55, stair_range=.25,
                     pitch=0., filtered=True):
    """computeHeightMap 0x8f5d8, changeHeight=false, finite input branch.

    Coordinates are already gravity aligned by the caller. Truncating indices,
    cyclic sample overwrite, cardinal searches and the lower median reproduce
    the binary. Missing cells retain raw=-10, counts=0 and NaN samples; they
    never become synthetic measured returns. Filter 0x8b548 is OpenCV median3.
    The upstream sync/ray-tracing pipeline is implemented in height_frontend.
    """
    p=np.asarray(points,dtype=np.float32).reshape(-1,3)
    if not np.isfinite(p).all():raise ValueError('nonfinite height input')
    nx,ny=shape
    if min(shape)<=0 or retained<=0 or resolution<=0:raise ValueError('invalid height dimensions')
    ij=np.trunc((p[:,:2]-np.asarray(origin,np.float32))/np.float32(resolution)).astype(np.int64)
    valid=(ij[:,0]>=0)&(ij[:,0]<nx)&(ij[:,1]>=0)&(ij[:,1]<ny)
    ij,p=ij[valid],p[valid]
    ids=ij[:,0]*ny+ij[:,1]
    totals=np.bincount(ids,minlength=nx*ny)
    order=np.argsort(ids,kind='stable')
    offsets=np.r_[0,np.cumsum(totals[:-1])]
    ordinal=np.empty(len(ids),np.int64)
    ordinal[order]=np.arange(len(ids))-np.repeat(offsets,totals)
    last=np.full(retained*nx*ny,-1,np.int64)
    np.maximum.at(last,(ordinal%retained)*nx*ny+ids,np.arange(len(ids)))
    samples=np.full(last.shape,np.nan,np.float32)
    populated=last>=0;samples[populated]=p[last[populated],2]
    samples=samples.reshape(retained,nx,ny)
    counts=np.minimum(totals,retained).astype(np.int32).reshape(shape)
    raw=np.max(np.where(np.isfinite(samples),samples,-np.inf),axis=0)
    raw[counts==0]=-10.
    candidates=[]
    yrange=3 if abs(float(np.float32(math.cos(pitch))))<.95 else 20
    for dx,dy,limit in ((-1,0,20),(1,0,20),(0,-1,yrange),(0,1,yrange)):
        neighbor=np.full(shape,np.nan,np.float32)
        for distance in range(1,limit):
            if (dx and distance>=nx) or (dy and distance>=ny):break
            dest=(slice(max(0,-dx*distance),min(nx,nx-dx*distance)),
                  slice(max(0,-dy*distance),min(ny,ny-dy*distance)))
            src=(slice(max(0,dx*distance),min(nx,nx+dx*distance)),
                 slice(max(0,dy*distance),min(ny,ny+dy*distance)))
            take=np.isnan(neighbor[dest])&(counts[src]>0)
            neighbor[dest][take]=raw[src][take]
        candidates.append(neighbor)
    candidates=np.sort(np.stack(candidates),axis=0)
    found=np.isfinite(candidates).sum(axis=0)
    maximum=np.take_along_axis(candidates,np.maximum(0,found-1)[None],axis=0)[0]
    median=np.take_along_axis(candidates,np.maximum(0,(found-1)//2)[None],axis=0)[0]
    fill=np.where(maximum-candidates[0]>np.float32(stair_range),candidates[0],median)
    fill=np.where(found>1,fill,-np.float32(robot_height))
    height=np.where(counts>0,raw,fill).astype(np.float32)
    if filtered:
        import cv2
        height=cv2.medianBlur(height,3)
    return height,samples,counts,raw


@dataclass
class HeightPolicy:
    """0x8c928 and 0x8caa0. Terrain IDs are kept numeric deliberately."""
    single_battery: float = .08
    flat: float = .25
    stair: float = .25
    terrain: int = 1
    stair_oa: bool = False
    height_range: float = .25

    def _update(self):
        # The binary accepts unknown terrain IDs and still returns success.
        # For unknown IDs, disabled OA retains the previous threshold, whereas
        # enabled OA overwrites it with the single-battery threshold.
        # Terrain 2's enabled branch first stores `flat`, then joins the common
        # tail, which overwrites it with `single_battery` (0x8ca6c -> 0x8c9d0).
        if self.stair_oa:
            self.height_range = self.single_battery
        elif self.terrain == 1:
            self.height_range = self.flat
        elif self.terrain == 2:
            self.height_range = self.stair
        self.height_range = float(np.float32(self.height_range))
        return 1

    def set_stair_oa(self, command):
        self.stair_oa = command == 1
        return self._update()

    def set_terrain(self, command):
        self.terrain = int(command)
        return self._update()


def obstacle_mask(height, samples, counts, policy, *, robot_height=.55,
                  point_range_flat=2, point_range_stair=2):
    """HeightMap::computeCostMap 0x8eda8, remove_cloud_nav_enabled=true.

    height[x,y] is the already filtered height image. samples[k,x,y] contains
    retained raw cell heights; counts[x,y] bounds its populated entries.
    The output is the cells emitted to cloud_nav. The native function clears
    cloud_obs but does not populate it along this branch. Two border cells are
    excluded on every side. Strict comparisons and float32 thresholds matter.
    This is not an elevation builder, missing-cell filler or gait classifier.
    """
    height = np.asarray(height, dtype=np.float32)
    samples = np.asarray(samples, dtype=np.float32)
    counts = np.asarray(counts)
    if height.ndim != 2 or samples.ndim != 3 or samples.shape[1:] != height.shape:
        raise ValueError('height/sample dimensions differ')
    if counts.shape != height.shape or not np.issubdtype(counts.dtype, np.integer):
        raise ValueError('invalid cell counts')
    if np.any(counts < 0) or np.any(counts > len(samples)):
        raise ValueError('cell count exceeds retained samples')
    # Production integration must reject invalid/unfilled maps. Native NaN
    # compare behavior is not interpreted here as evidence of free space.
    if not np.isfinite(height).all():
        raise ValueError('height map must be finite and already filled')
    threshold = np.float32(policy.height_range)
    deep = -np.float32(threshold + np.float32(robot_height))
    points_required = point_range_flat if policy.terrain == 1 else point_range_stair
    out = np.zeros(height.shape, dtype=bool)
    if min(height.shape) < 5:
        return out
    c = height[2:-2, 2:-2]
    neighbors = (height[2:-2, 1:-3], height[2:-2, 3:-1],
                 height[1:-3, 2:-2], height[3:-1, 2:-2])
    low = np.minimum.reduce(neighbors)
    # AArch64 promotes each height operand to double before subtracting.
    raised = c.astype(np.float64) - low.astype(np.float64) > float(threshold)
    n = counts[2:-2, 2:-2]
    above = np.zeros(c.shape, dtype=np.int32)
    for k, layer in enumerate(samples):
        above += ((k < n) & (layer[2:-2, 2:-2].astype(np.float64)
                            - low.astype(np.float64) > float(threshold)))
    out[2:-2, 2:-2] = raised & ((above > points_required)
                                               | np.logical_or.reduce([a < deep for a in neighbors]))
    return out
