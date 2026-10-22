#!/usr/bin/env python3
"""Fit RWH apt ROS tag centers in the upright rack frame; no redetection."""
import hashlib
import importlib.util
import json
import sys
from pathlib import Path

SOURCE=Path('/home/lab6/Warp-review/wilor-kuavo-standard/tools/ros_tag_pose.py')


def solve(request,module=None):
    import numpy as np
    if request.get('detector') != 'apt_apriltag_ros':
        raise ValueError('apt apriltag_ros 관측만 허용합니다')
    p=np.asarray(request['base_camera_pose'],float)
    from scipy.spatial.transform import Rotation
    Tbc=np.eye(4);Tbc[:3,:3]=Rotation.from_quat(p[3:]).as_matrix();Tbc[:3,3]=p[:3]
    known={int(k):np.asarray(v,float) for k,v in request['rack_tags'].items()}
    ids=[];local=[];measured=[]
    for tag in request['tags']:
        tid=int(tag['id'])
        if tid not in known:continue
        if tid in ids:raise ValueError('중복 태그 ID')
        T=np.asarray(tag['T_camera_tag'],float)
        if T.shape!=(4,4) or not np.isfinite(T).all():raise ValueError('잘못된 태그 자세')
        ids.append(tid);local.append(known[tid][:3,3]);measured.append((Tbc@T)[:3,3])
    if len(ids)==1 and request.get('allow_single_tag_projection'):
        tid=ids[0]
        observed=next(np.asarray(v['T_camera_tag'],float) for v in request['tags'] if int(v['id'])==tid)
        relative=Tbc@observed@np.linalg.inv(known[tid])
        if np.linalg.norm(relative[:2,0])<1e-6:raise ValueError('태그의 수평 방향을 추정할 수 없습니다')
        yaw=np.arctan2(relative[1,0],relative[0,0])
        R=Rotation.from_euler('z',yaw).as_matrix()
        transform=np.eye(4);transform[:3,:3]=R
        transform[:3,3]=measured[0]-R@local[0]
        return dict(T_camera_recording=(np.linalg.inv(Tbc)@transform).tolist(),tag_ids=ids,
            observed_tag_ids=ids,rejected_tag_ids=[],method='rwh_single_tag_projection',
            rms_m=None,rms=None,detector='apt_apriltag_ros',corner_source='not_used',projection_only=True)
    if len(ids)<2:raise ValueError('가로로 떨어진 랙 태그가 두 개 이상 필요합니다')
    a,b=np.asarray(local),np.asarray(measured)
    ac=a[:,:2]-a[:,:2].mean(0);bc=b[:,:2]-b[:,:2].mean(0)
    if np.linalg.norm(ac)<.03 or np.linalg.norm(bc)<.03:raise ValueError('랙 태그 간격이 부족합니다')
    yaw=np.arctan2(np.sum(ac[:,0]*bc[:,1]-ac[:,1]*bc[:,0]),np.sum(ac*bc))
    R=Rotation.from_euler('z',yaw).as_matrix();t=np.mean(b-a@R.T,axis=0)
    rms=float(np.sqrt(np.mean(np.sum((a@R.T+t-b)**2,axis=1))))
    if not np.isfinite(rms) or rms>.05:raise ValueError('랙 태그 피팅 잔차가 5cm를 넘었습니다')
    T=np.eye(4);T[:3,:3]=R;T[:3,3]=t
    return dict(T_camera_recording=(np.linalg.inv(Tbc)@T).tolist(),tag_ids=ids,
        observed_tag_ids=ids,rejected_tag_ids=[],method='rwh_planar_tag_positions',rms_m=rms,
        rms=None,detector='apt_apriltag_ros',corner_source='not_used')


def main():
    sys.dont_write_bytecode=True
    fingerprint=hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    for line in sys.stdin:
        try:
            request=json.loads(line)
            if request['standard_sha256']!=fingerprint:raise ValueError('standard 추정 코드가 바뀌었습니다. 다시 연결하세요.')
            result=solve(request);result['standard_sha256']=fingerprint
            reply={'ok':True,'result':result}
        except Exception as exc:reply={'ok':False,'error':str(exc)}
        print(json.dumps(reply,separators=(',',':')),flush=True)


if __name__=='__main__':main()
