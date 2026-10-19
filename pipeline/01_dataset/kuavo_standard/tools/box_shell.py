"""Five non-overlapping plates within the measured outer box envelope."""
import numpy as np
from scipy.spatial.transform import Rotation

def box_shell(box, thickness_m=.001):
    L,W,H=map(float,box['dimensions']);t=float(thickness_m)
    if not 0<t<min(L,W,H)/2:raise ValueError('Invalid box wall thickness')
    R=Rotation.from_quat(box['quaternion_xyzw']).as_matrix();p=np.asarray(box['position'])
    pieces=[('left',[-L/2+t/2,0,0],[t,W,H]),('right',[L/2-t/2,0,0],[t,W,H]),('front',[0,-W/2+t/2,0],[L-2*t,t,H]),('back',[0,W/2-t/2,0],[L-2*t,t,H]),('bottom',[0,0,-H/2+t/2],[L-2*t,W-2*t,t])]
    return [dict(id=box['id']+'_'+name,position=(p+R@np.asarray(offset)).tolist(),quaternion_xyzw=box['quaternion_xyzw'],dimensions=dims) for name,offset,dims in pieces]
