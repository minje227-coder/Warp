#!/usr/bin/env python3
"""Read-only head camera, tag poses and timestamp-matched base/camera TF.

No publishers, services or actions. JPEG, intrinsics and detections always refer
to the same Image; repeated or stale images never receive a new frame identity.
"""
import base64
import json
import os
import sys
import threading
import time

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")


def standard_corners(corners):
    """OpenCV AprilTag dictionary -> adopted pupil/IPPE TL,TR,BR,BL order.

    On identical recorded frames, OpenCV starts at pupil's BR, whereas the
    adopted estimator uses pupil.corners[[3,2,1,0]]. Rotate by two corners;
    leaving this out rotates the tag frame 180 degrees and corrupts placement.
    """
    return [corners[i] for i in (2,3,0,1)]


def main():
    import cv2
    import numpy as np
    import rospy
    import tf2_ros
    from sensor_msgs.msg import CameraInfo, Image

    cv2.setNumThreads(1)
    rospy.init_node("dashboard_retargeting_view", anonymous=True, disable_signals=True)
    lock, done = threading.Lock(), threading.Event()
    state = {}
    def image(msg):
        with lock:
            state.update(image=msg, received=time.monotonic())
    def info(msg):
        with lock:
            state["info"] = msg
    rospy.Subscriber("/head_camera/color/image_raw", Image, image, queue_size=1, buff_size=2**24)
    rospy.Subscriber("/head_camera/color/camera_info", CameraInfo, info, queue_size=1)
    buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10))
    listener = tf2_ros.TransformListener(buffer)
    def stdin():
        for _ in sys.stdin:
            pass
        done.set()
    threading.Thread(target=stdin, daemon=True).start()
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    params = cv2.aruco.DetectorParameters() if hasattr(cv2.aruco, "ArucoDetector") else cv2.aruco.DetectorParameters_create()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, params)
        detect = detector.detectMarkers
    else:
        detect = lambda a: cv2.aruco.detectMarkers(a, dictionary, parameters=params)
    last = None
    while not done.is_set() and not rospy.is_shutdown():
        started = time.monotonic()
        with lock:
            msg, ci, received = state.get("image"), state.get("info"), state.get("received", 0)
        if msg is not None and ci is not None and msg.header.stamp.to_nsec() != last and started-received < 1:
            try:
                if (ci.width, ci.height) != (msg.width, msg.height):
                    raise ValueError("camera_info/image dimensions differ")
                if ci.distortion_model not in ("plumb_bob", "rational_polynomial", ""):
                    raise ValueError("unsupported distortion model: " + ci.distortion_model)
                if ci.header.frame_id and ci.header.frame_id != msg.header.frame_id:
                    raise ValueError("camera_info/image frames differ")
                channels = 1 if msg.encoding in ("mono8", "8UC1") else 3
                if msg.encoding not in ("mono8", "8UC1", "rgb8", "bgr8"):
                    raise ValueError("unsupported image encoding: " + msg.encoding)
                a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width*channels]
                a = a.reshape(msg.height, msg.width, channels)
                bgr = a if channels == 1 or msg.encoding == "bgr8" else cv2.cvtColor(a, cv2.COLOR_RGB2BGR)
                gray = bgr[:, :, 0] if channels == 1 else cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
                K, D = np.array(ci.K, float).reshape(3, 3), np.array(ci.D, float)
                corners, ids, _ = detect(gray)
                tags = []
                for c, tid in zip(corners, [] if ids is None else ids.flatten()):
                    tid = int(tid)
                    if not 0 <= tid <= 5:
                        continue
                    h = .04/2
                    obj = np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]], float)
                    uv = np.asarray(standard_corners(c.reshape(4,2)),dtype=float)
                    solutions = cv2.solvePnPGeneric(obj, uv, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                    choices = []
                    for rv, tv in zip(solutions[1], solutions[2]):
                        R = cv2.Rodrigues(rv)[0]
                        if np.min((obj @ R.T + tv.reshape(3))[:,2]) <= 0:
                            continue
                        px = cv2.projectPoints(obj, rv, tv, K, D)[0].reshape(-1,2)
                        rms = float(np.sqrt(np.mean(np.sum((px-uv)**2,axis=1))))
                        T = np.eye(4); T[:3,:3] = R; T[:3,3] = tv.reshape(3)
                        choices.append((rms,T))
                    if choices:
                        rms,T = min(choices,key=lambda x:x[0])
                        tags.append(dict(id=tid,T_camera_tag=T.tolist(),corners=uv.tolist(),rms_px=rms))
                payload = dict(frame_id=str(msg.header.stamp.to_nsec()), stamp=msg.header.stamp.to_sec(),
                               frame=msg.header.frame_id, width=msg.width,height=msg.height,K=ci.K,D=ci.D,
                               distortion_model=ci.distortion_model,tags=tags,image_age_s=started-received,corner_refinement="subpixel")
                try:
                    tr = buffer.lookup_transform("base_link",msg.header.frame_id,msg.header.stamp,rospy.Duration(.4)).transform
                    payload["base_camera_pose"] = [tr.translation.x,tr.translation.y,tr.translation.z,
                                                  tr.rotation.x,tr.rotation.y,tr.rotation.z,tr.rotation.w]
                except Exception as exc:
                    payload["tf_error"] = str(exc)[:180]
                ok,jpeg = cv2.imencode(".jpg",bgr,[cv2.IMWRITE_JPEG_QUALITY,82])
                if not ok:
                    raise ValueError("JPEG encoding failed")
                payload["jpeg"] = base64.b64encode(jpeg).decode("ascii")
                ok,png = cv2.imencode('.png',bgr,[cv2.IMWRITE_PNG_COMPRESSION,3])
                if not ok:raise ValueError('PNG encoding failed')
                payload['png'] = base64.b64encode(png).decode('ascii')
                payload["image_age_s"] = time.monotonic()-received
                sys.stdout.write(json.dumps(payload,separators=(",",":"))+"\n");sys.stdout.flush()
                last = msg.header.stamp.to_nsec()
            except Exception as exc:
                sys.stdout.write(json.dumps({"log":str(exc)[:200]})+"\n");sys.stdout.flush()
        done.wait(max(.01,.5-(time.monotonic()-started)))


if __name__ == "__main__":
    main()
