"""Time-centered position mean and SO(3) chordal mean, without pose re-selection."""
import copy
import numpy as np
from scipy.spatial.transform import Rotation

def stabilize(poses, candidates, K, half_window_s=.1):
    original=copy.deepcopy(poses)
    times=np.array([r['pts_us'] for r in poses],float)/1e6
    valid=np.array(['7' in r['tags'] for r in poses])
    support=[];changes=[];angles=[];before=[];after=[]
    h=.02
    corners=np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]])
    def reprojection(row,tag):
        Rwc=np.array(row['camera_rotation']);twc=np.array(row['camera_position_m'])
        world=corners@np.array(tag['rotation']).T+np.array(tag['position_m'])
        optical=(world-twc)@Rwc
        assert np.all(optical[:,2]>0)
        uv=optical@K.T;uv=uv[:,:2]/uv[:,2:]
        observed=np.array(candidates[(row['quest_decoded_index'],7)]['corners'])[[3,2,1,0]]
        return float(np.sqrt(np.mean(np.sum((uv-observed)**2,axis=1))))
    for i,row in enumerate(poses):
        if not valid[i]:continue
        # Do not bridge a missing observation or a gap > 100 ms.
        lo=hi=i
        while lo>0 and valid[lo-1] and times[lo]-times[lo-1]<=.1 and times[i]-times[lo-1]<=half_window_s+1e-9:lo-=1
        while hi+1<len(poses) and valid[hi+1] and times[hi+1]-times[hi]<=.1 and times[hi+1]-times[i]<=half_window_s+1e-9:hi+=1
        neighbors=[r['tags']['7'] for r in original[lo:hi+1]]
        p=np.mean([r['position_m'] for r in neighbors],axis=0)
        R=Rotation.from_matrix(np.array([r['rotation'] for r in neighbors])).mean().as_matrix()
        raw=original[i]['tags']['7'];tag=row['tags']['7']
        tag['raw_position_m']=raw['position_m'];tag['raw_rotation']=raw['rotation']
        tag['position_m']=p.tolist();tag['rotation']=R.tolist()
        tag['smoothing_support_indices']=[lo,hi]
        tag['smoothing_support_count']=len(neighbors)
        tag['reprojection_rms_px_raw']=reprojection(row,raw)
        tag['reprojection_rms_px']=reprojection(row,tag)
        before.append(tag['reprojection_rms_px_raw']);after.append(tag['reprojection_rms_px'])
        changes.append(float(np.linalg.norm(p-raw['position_m']))*1000)
        angles.append(float(np.degrees(Rotation.from_matrix(R@np.array(raw['rotation']).T).magnitude())))
        support.append(len(neighbors))
        # Smoothed world pose and its camera representation must describe the same pose.
        Rwc=np.array(row['camera_rotation']);twc=np.array(row['camera_position_m'])
        tag['camera_rotation']=(Rwc.T@R).tolist()
        tag['camera_translation_m']=(Rwc.T@(p-twc)).tolist()
    assert all(a['tags'].get('2')==b['tags'].get('2') for a,b in zip(original,poses))
    def summary(v):
        return dict(median=float(np.median(v)),p95=float(np.percentile(v,95)),max=float(max(v)))
    def motion(data):
        speeds=[];angular=[];accelerations=[];steps=[]
        for i in range(1,len(data)):
            if not (valid[i-1] and valid[i]):continue
            dt=times[i]-times[i-1]
            if dt>.1:continue
            a,b=[r['tags']['7'] for r in data[i-1:i+1]]
            dp=(np.array(b['position_m'])-a['position_m'])/dt
            speeds.append(float(np.linalg.norm(dp))*1000)
            steps.append(float(np.linalg.norm(dp))*dt*1000)
            angular.append(float(np.degrees(Rotation.from_matrix(np.array(b['rotation'])@np.array(a['rotation']).T).magnitude()))/dt)
            if i>=2 and valid[i-2] and times[i-1]-times[i-2]<=.1:
                prev=np.array(data[i-2]['tags']['7']['position_m'])
                dt0=times[i-1]-times[i-2]
                v0=(np.array(a['position_m'])-prev)/dt0
                accelerations.append(float(np.linalg.norm((dp-v0)/((dt+dt0)/2)))*1000)
        return dict(position_step_mm=summary(steps),speed_mm_s=summary(speeds),angular_speed_deg_s=summary(angular),acceleration_mm_s2=summary(accelerations))
    return dict(method='centered unweighted arithmetic position mean + scipy Rotation.mean chordal L2',
        half_window_s=half_window_s,full_window_s=2*half_window_s,
        support_count=summary(support),boundary_policy='truncate window at clip boundaries; never bridge missing observations or gaps > 100 ms',
        pose_candidates_reselected=False,stationary_lock=False,
        position_change_mm=summary(changes),rotation_change_deg=summary(angles),
        raw_reprojection_rms_px=summary(before),smoothed_reprojection_rms_px=summary(after),
        raw_motion=motion(original),smoothed_motion=motion(poses),
        note='Motion metrics mix real movement and jitter; lower derivatives alone do not prove greater accuracy.')
