#!/usr/bin/env python3
"""Read-only adapter to the adopted standard's unmodified tag-bundle solver.

Runs in system Python, which already supplies OpenCV. Dashboard Python need
not acquire another numerical stack. ROS is never imported here.
"""
import base64
import hashlib
import importlib.util
import json
import sys
from pathlib import Path

SOURCE=Path('/home/lab6/Warp-review/wilor-kuavo-standard/tools/multi_tag_pose.py')
VISION_DEPS=Path('/home/lab6/humanoid/data/retargeting/vision-deps')
DETECTOR=None


def solve(request,module):
    import cv2
    import numpy as np
    K=np.asarray(request['K'],float).reshape(3,3)
    D=np.asarray(request.get('D',[]),float)
    known={int(k):np.asarray(v,float) for k,v in request['rack_tags'].items()}
    anchor=known[min(known)]
    centers={tid:anchor[:3,:3].T@(M[:3,3]-anchor[:3,3]) for tid,M in known.items()}
    detections={};raw_corners={}
    if request.get('png'):
        global DETECTOR
        if DETECTOR is None:
            sys.path.insert(0,str(VISION_DEPS))
            from pupil_apriltags import Detector
            DETECTOR=Detector(families='tag36h11',nthreads=1,quad_decimate=1.,quad_sigma=0.,
                              refine_edges=1,decode_sharpening=.25)
        image=cv2.imdecode(np.frombuffer(base64.b64decode(request['png']),np.uint8),cv2.IMREAD_GRAYSCALE)
        if image is None:raise ValueError('원본 head PNG 해독 실패')
        observed=[(int(t.tag_id),np.asarray(t.corners,float)) for t in DETECTOR.detect(image)]
        detector='pupil_apriltags'
    else:
        # Saved-corner regression inputs are already in IPPE order.
        observed=[(int(t['id']),np.asarray(t['corners'],float)[[3,2,1,0]]) for t in request['tags']]
        detector='provided_corners'
    for tid,pupil in observed:
        if tid not in known:continue
        if not np.isfinite(pupil).all():raise ValueError('태그 코너가 유효하지 않습니다')
        rect=cv2.undistortPoints(pupil.reshape(-1,1,2),K,D if len(D) else None,P=K).reshape(4,2)
        detections[tid]={'corners':rect.tolist()}
        raw_corners[str(tid)]=pupil[[3,2,1,0]].tolist()
    p=np.asarray(request['base_camera_pose'],float)
    x,y,z,w=p[3:]/np.linalg.norm(p[3:])
    Rbc=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                  [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                  [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    Tbc=np.eye(4);Tbc[:3,:3]=Rbc;Tbc[:3,3]=p[:3]
    config=request['settings'];selection=config['pose_selection'];settings=config['multi_tag_pose']
    q=module.estimate_tag_bundle(detections,centers,config['tag_size_m'],K,Rbc,p[:3],
        max_rms_px=selection['maximum_reprojection_rms_px'],
        min_up=selection['minimum_tag_y_up_component'],
        ransac_error_px=settings['ransac_reprojection_error_px'],
        ransac_iterations=settings['ransac_iterations'],ransac_confidence=settings['ransac_confidence'])
    if q is None:raise ValueError('standard 기준을 통과한 랙 태그 자세가 없습니다')
    Tba=np.eye(4);Tba[:3,:3]=q['rotation'];Tba[:3,3]=q['position']
    T=np.linalg.inv(Tbc)@Tba@np.linalg.inv(anchor)
    meta={k:q[k] for k in ['method','tag_ids','observed_tag_ids','rejected_tag_ids','corner_count',
                          'per_tag_rms_px','rms','joint_failure_reason'] if k in q}
    return dict(T_camera_recording=T.tolist(),raw_tag_corners=raw_corners,detector=detector,**meta)


def main():
    sys.dont_write_bytecode=True
    spec=importlib.util.spec_from_file_location('adopted_standard_multi_tag_pose',SOURCE)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    fingerprint=hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    for line in sys.stdin:
        try:
            request=json.loads(line)
            if request['standard_sha256']!=fingerprint:raise ValueError('standard 추정 코드가 바뀌었습니다. 다시 연결하세요.')
            result=solve(request,module);result['standard_sha256']=fingerprint
            reply={'ok':True,'result':result}
        except Exception as exc:reply={'ok':False,'error':str(exc)}
        print(json.dumps(reply,separators=(',',':')),flush=True)


if __name__=='__main__':main()
