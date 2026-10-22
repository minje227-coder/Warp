"""Metric planar fit of apt ROS tag positions; no image detector or IPPE fallback."""
import numpy as np
from scipy.spatial.transform import Rotation

def estimate_tag_bundle(ds,centers,size,K,Rwc,twc,**kwargs):
 ids=sorted(set(ds)&set(centers))
 if len(ids)<2:return None
 # Printed tag coordinates: x right, y up, z outward. Physical upright object.
 basis=np.array([[1.,0,0],[0,0,-1],[0,1,0]])
 a=np.array([basis@centers[k] for k in ids]);b=np.array([Rwc@np.array(ds[k]['translation_m'])+twc for k in ids])
 ac=a[:,:2]-a[:,:2].mean(0);bc=b[:,:2]-b[:,:2].mean(0)
 if np.linalg.norm(ac)<.03 or np.linalg.norm(bc)<.03:return None
 yaw=np.arctan2(np.sum(ac[:,0]*bc[:,1]-ac[:,1]*bc[:,0]),np.sum(ac*bc))
 R=Rotation.from_euler('z',yaw).as_matrix();t=np.mean(b-a@R.T,axis=0)
 rms=float(np.sqrt(np.mean(np.sum((a@R.T+t-b)**2,axis=1))))
 if not np.isfinite(rms) or rms>.05:return None
 return dict(rotation=R@basis,position=t,rms_m=rms,rms=None,method='rwh_planar_tag_positions',tag_ids=ids,observed_tag_ids=ids,rejected_tag_ids=[],corner_count=0,per_tag_rms_px={},joint_failure_reason=None,detector='apt_apriltag_ros')
