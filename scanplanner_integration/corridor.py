"""Export SE(2) map geometry as an optimizer cost, never as motion permission."""
import math
import numpy as np
from shapely.geometry import box


def encode_corridor(planner, reference, height, stamp, margin=1.):
    reference=np.asarray(reference)
    low=reference[:,:3].min(axis=0);high=reference[:,:3].max(axis=0)
    low[:2]-=margin;high[:2]+=margin
    low[2]-=height+planner.config.max_step;high[2]-=height-planner.config.max_step
    regions=[];witnesses=set()
    for raw in planner.tree.query(box(*low[:2],*high[:2])):
        i=int(raw);p=planner.polygons[i]
        if not low[2]<=p['floor']<=high[2]:continue
        mask=int(p.get('proven_yaw_mask',0))
        if mask:
            # Soft interior preference prevents an optimizer stopping a fraction
            # of a millimetre outside a strict certificate boundary. This does
            # not alter map permissions or remove narrow feasible witnesses.
            interior=planner.shapes[i].buffer(-.01,join_style=2)
            vertices=(np.asarray(interior.exterior.coords)[:-1] if not interior.is_empty and interior.geom_type=='Polygon' else np.asarray(p['vertices'])[:,:2])
            regions.append((p['floor']+height,mask,vertices))
        for pid in planner.region_positions[i]:
            if pid in planner.witness_cells:witnesses.add(pid)
    # A refined cell does not certify its whole polygon. Export only its
    # feasible pose witnesses as point costs; the exact sweep remains required.
    for pid in witnesses:
        xyz=planner.positions[pid]
        if np.any(xyz[:2]<low[:2]) or np.any(xyz[:2]>high[:2]):continue
        mask=0
        for yaw in planner.angles[pid].values():
            mask |= 1 << (int(math.floor(yaw*planner.bins/(2*math.pi)+.5))%planner.bins)
        regions.append((xyz[2]+height,mask,np.asarray([xyz[:2]])))
    # Reference Z selects the nearby support layer, not a centimetre-accurate
    # floor at every detour XY. Verification resolves containing certificates
    # to their actual floor and checks all continuous transitions separately.
    data=[1.,float(stamp),float(planner.bins),float(planner.config.max_step)+1e-9,float(len(regions))]
    for z,mask,vertices in regions:
        data.extend([float(z),float(mask&0xffffffff),float(mask>>32),float(len(vertices))])
        data.extend(map(float,vertices.ravel()))
    return data
