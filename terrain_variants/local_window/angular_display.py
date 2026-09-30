"""Existing cell color encodes the certified feasible heading measure."""
import math
import numpy as np

DEEP=(.64,.16,.015,.94)
LIGHT=(1.,.72,.12,.94)


def orange_color(fraction):
    # Gamma keeps small but nonzero orientation ranges visibly differentiated.
    t=max(0.,min(1.,float(fraction)))**.5
    return tuple(a+(b-a)*t for a,b in zip(DEEP,LIGHT))


def result_rows(result,yaw_bins):
    states=getattr(result,'pose_states',None)
    comfortable=getattr(result,'comfortable_masks',None)
    if states is not None:
        measure=np.nansum(np.maximum(0.,states[:,:,5]-states[:,:,4]),axis=1)
        measure=np.minimum(measure,2*math.pi)
    else:measure=[int(m).bit_count()*2*math.pi/yaw_bins for m in result.masks]
    if comfortable is None:comfortable=result.masks
    return [[int(s.ix),int(s.iy),int(s.iz),int(m),int(cm),float(a)]
            for s,m,cm,a in zip(result.spans,result.masks,comfortable,measure)]


def row_style(row,yaw_bins):
    full=(1<<yaw_bins)-1
    return ((row[4] if len(row)>4 else row[3])==full,
            min(1.,max(0.,row[5]/(2*math.pi))) if len(row)>5 else int(row[3]).bit_count()/yaw_bins)
