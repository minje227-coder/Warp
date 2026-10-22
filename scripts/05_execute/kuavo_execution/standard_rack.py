"""Serialized, bounded read-only bridge to the adopted standard estimator."""
import atexit
import collections
import hashlib
import json
import select
import subprocess
import threading
from pathlib import Path

SOURCE=Path('/home/lab6/Warp-review/wilor-kuavo-standard/tools/ros_tag_pose.py')
WORKER=Path(__file__).with_name('standard_rack_worker.py')
LOCK=threading.RLock()
PROC=None
SIGNATURE=None
CACHE=collections.OrderedDict()


def close():
    global PROC
    with LOCK:
        if PROC:
            try:PROC.stdin.close()
            except (OSError,ValueError):pass
            try:PROC.wait(timeout=.2)
            except subprocess.TimeoutExpired:PROC.terminate()
            try:PROC.stdout.close()
            except (OSError,ValueError):pass
            PROC=None


atexit.register(close)


def estimate(detail,sample):
    global PROC,SIGNATURE
    if not sample.get('base_camera_pose'):raise ValueError('head→base_link TF를 기다리는 중입니다')
    signature=hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    worker_signature=hashlib.sha256((signature+hashlib.sha256(WORKER.read_bytes()).hexdigest()).encode()).hexdigest()
    request={k:sample[k] for k in ['K','tags','base_camera_pose']}
    request.update(D=sample.get('D',[]),detector=sample.get('detector'),
                   corner_order=sample.get('corner_order','ippe_tl_tr_br_bl'),
                   rack_tags=detail['rack_tags'],settings=detail['rack_fit_settings'],standard_sha256=signature,
                   allow_single_tag_projection=bool(detail.get('allow_single_tag_projection')))
    payload=json.dumps(request,separators=(',',':')).encode()+b'\n'
    key=hashlib.sha256(payload).digest()
    with LOCK:
        if SIGNATURE!=worker_signature:
            close();CACHE.clear()
        if key in CACHE:return CACHE[key]
        if SIGNATURE!=worker_signature or PROC is None or PROC.poll() is not None:
            close();PROC=subprocess.Popen(['/usr/bin/python3',str(WORKER)],stdin=subprocess.PIPE,
                                          stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,bufsize=0)
            SIGNATURE=worker_signature
        try:
            PROC.stdin.write(payload);PROC.stdin.flush()
            if not select.select([PROC.stdout],[],[],2)[0]:raise ValueError('standard 태그 추정 응답 시간 초과')
            line=PROC.stdout.readline()
            if not line:raise ValueError('standard 태그 추정 프로세스 연결이 끊겼습니다')
            reply=json.loads(line)
        except (OSError,ValueError):
            close();raise
        if not reply['ok']:raise ValueError(reply['error'])
        result=reply['result'];CACHE[key]=result
        while len(CACHE)>16:CACHE.popitem(last=False)
        return result
