#!/usr/bin/env python3
"""Read-only tag poses and timestamp-matched base/camera TF.

No image subscription or encoding: display uses the dashboard head camera.
No publishers, services or actions. Tag/TF freshness remains independent of display.
"""
import json
import os
import sys
import threading
import time

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")


class TfReadiness:
    """Require consecutive distinct, timestamp-matched camera observations."""
    def __init__(self):
        self.count=0;self.first_at=None;self.last_stamp=None

    def update(self,stamp,matched,now):
        if not matched:
            self.count=0;self.first_at=None;self.last_stamp=None
            return False
        if stamp!=self.last_stamp:
            if self.last_stamp is not None and stamp<self.last_stamp:
                self.count=0;self.first_at=None
            self.last_stamp=stamp;self.count+=1
            if self.first_at is None:self.first_at=now
        return self.count>=3 and now-self.first_at>=.8


def image_transform(buffer,frame,stamp,duration,stopped,clock=time.monotonic,wait=None):
    """Retry a briefly delayed TF at the SAME image stamp, never Time(0)."""
    wait=time.sleep if wait is None else wait
    deadline=clock()+.6;failure=None
    while not stopped():
        try:
            return buffer.lookup_transform('base_link',frame,stamp,duration(.04)).transform
        except Exception as exc:
            failure=exc
            # Buffer cannot gain older history. Skip this image immediately.
            text=str(exc).lower()
            if 'into the past' in text or 'earliest data' in text:raise
            if clock()>=deadline:raise
            wait(.01)
    if failure is not None:raise failure
    raise RuntimeError('TF observation stopped')


def main():
    import numpy as np
    import rospy
    import tf2_ros
    from sensor_msgs.msg import CameraInfo
    from apriltag_ros.msg import AprilTagDetectionArray
    from tf.transformations import quaternion_matrix

    rospy.init_node("dashboard_retargeting_observation", anonymous=True, disable_signals=True)
    lock, done = threading.Lock(), threading.Event()
    state = {}
    def info(msg):
        with lock: state['info'] = msg
    def detections(msg):
        with lock: state['detections'] = msg
    rospy.Subscriber('/head_camera/color/camera_info', CameraInfo, info, queue_size=1)
    rospy.Subscriber('/rwh/tag_detections', AprilTagDetectionArray, detections, queue_size=1)
    buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10))
    listener = tf2_ros.TransformListener(buffer)
    def stdin():
        for _ in sys.stdin: pass
        done.set()
    threading.Thread(target=stdin, daemon=True).start()
    last = None
    readiness = TfReadiness()
    while not done.is_set() and not rospy.is_shutdown():
        with lock:
            ds, ci = state.get('detections'), state.get('info')
        if ds is not None and ci is not None and ds.header.stamp.to_nsec() != last:
            try:
                age = (rospy.Time.now() - ds.header.stamp).to_sec()
                if not 0 <= age < 1.:
                    done.wait(.1)
                    continue
                last = ds.header.stamp.to_nsec()
                payload = dict(frame_id=str(last), stamp=ds.header.stamp.to_sec(),
                    frame=ds.header.frame_id, width=ci.width, height=ci.height,
                    K=list(ci.K), D=list(ci.D), distortion_model=ci.distortion_model,
                    tags=[], tf_ready=False, detector='apt_apriltag_ros',
                    pose_source='/rwh/tag_detections', observation_only=True)
                for d in ds.detections:
                    if len(d.id) != 1 or not 0 <= int(d.id[0]) <= 5: continue
                    q = d.pose.pose.pose
                    T = quaternion_matrix([q.orientation.x,q.orientation.y,q.orientation.z,q.orientation.w])
                    T[:3,3] = [q.position.x,q.position.y,q.position.z]
                    payload['tags'].append(dict(id=int(d.id[0]),T_camera_tag=T.tolist(),size_m=float(d.size[0])))
                try:
                    tr = image_transform(buffer,ds.header.frame_id,ds.header.stamp,rospy.Duration,
                        lambda:done.is_set() or rospy.is_shutdown(),wait=done.wait)
                    payload['tf_ready'] = readiness.update(last,True,time.monotonic())
                    if payload['tf_ready']:
                        payload['base_camera_pose'] = [tr.translation.x,tr.translation.y,tr.translation.z,
                            tr.rotation.x,tr.rotation.y,tr.rotation.z,tr.rotation.w]
                    else: payload['tf_status'] = '태그 좌표 연결 준비 중'
                except Exception as exc:
                    readiness.update(last,False,time.monotonic())
                    payload['tf_error'] = str(exc)[:240]
                    payload['tf_status'] = '태그 시각의 TF를 기다리는 중입니다.'
                payload['image_age_s'] = max(0.,(rospy.Time.now()-ds.header.stamp).to_sec())
                sys.stdout.write(json.dumps(payload,separators=(',',':'))+'\n');sys.stdout.flush()
            except Exception as exc:
                sys.stdout.write(json.dumps({'log':str(exc)[:200]})+'\n');sys.stdout.flush()
        done.wait(.1)


if __name__ == '__main__': main()
