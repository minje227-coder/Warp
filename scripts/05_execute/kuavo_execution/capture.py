"""Warp lifecycle for the read-only replay bag recorder."""
import json
import os
from pathlib import Path
import queue
import threading
import time
SETUP_EXTRA = []

WORKER=Path(os.environ.get('WARP_REPLAY_RECORDER',str(Path(__file__).resolve().parents[1]/'record_replay.py')))


class Capture:
    def __init__(self,link,run_id):
        self.link=link;self.run_id=run_id;self.proc=None
        self.ready=threading.Event();self.finished=threading.Event();self.lock=threading.RLock()
        self.outgoing=queue.Queue(maxsize=256);self.error='';self.result=None;self.info={}
        self.last_output=time.monotonic();self.last_anchor=-1.;self.stopping=False
        self.writer=None;self.reader=None

    def start(self):
        if not WORKER.is_file():raise ValueError('Warp 녹화 실행기가 없습니다: '+str(WORKER))
        self.proc=self.link.popen_script(str(WORKER),['--run-id',self.run_id],setup_extra=SETUP_EXTRA)
        self.writer=threading.Thread(target=self._write,daemon=True);self.writer.start()
        self.reader=threading.Thread(target=self._read,daemon=True);self.reader.start()
        threading.Thread(target=self._stderr,daemon=True).start()
        self.send(dict(op='beat'))

    def send(self,data):
        if self.finished.is_set():raise ValueError(self.error or '녹화 연결이 종료됐습니다')
        try:self.outgoing.put_nowait(data)
        except queue.Full:
            self.error='녹화 이벤트 전송이 지연됐습니다';raise ValueError(self.error)

    def _write(self):
        try:
            while True:
                item=self.outgoing.get()
                if item is None:break
                self.proc.stdin.write((json.dumps(item,separators=(',',':'),allow_nan=False)+'\n').encode())
                self.proc.stdin.flush()
        except Exception as exc:
            with self.lock:self.error='녹화 전송 실패: '+str(exc)

    def _read(self):
        try:
            for raw in iter(self.proc.stdout.readline,b''):
                if not raw.startswith(b'{'):continue
                d=json.loads(raw);self.last_output=time.monotonic()
                with self.lock:
                    if d.get('progress'):self.info['waiting']=d.get('waiting','')
                    if d.get('error'):self.error=d['error']
                    if d.get('ready'):self.info=d;self.ready.set()
                    if d.get('done'):self.result=d['recording']
        except Exception as exc:
            with self.lock:self.error='녹화 상태 수신 실패: '+str(exc)
        finally:self.finished.set()

    def _stderr(self):
        for _ in iter(self.proc.stderr.readline,b''):pass

    def check(self):
        with self.lock:
            if self.error:raise ValueError(self.error)
            if self.finished.is_set() or self.proc.poll() is not None:raise ValueError('전체 데이터 녹화가 중단됐습니다')
            silence_limit=4. if self.ready.is_set() else 30.
            if time.monotonic()-self.last_output>silence_limit:raise ValueError('녹화 상태 응답이 끊겼습니다')

    def wait_ready(self,check_cancel):
        deadline=time.monotonic()+30.
        while not self.ready.wait(.1):
            check_cancel();self.check();self.send(dict(op='beat'))
            if time.monotonic()>deadline:raise ValueError('카메라·센서 녹화 준비 시간 초과')
        check_cancel();self.check()

    def ingest(self,row):
        if row.get('control_modes'):
            self.send(dict(op='event',data=dict(event='controller_configuration',control_modes=row['control_modes'])))
        if row.get('event'):
            self.send(dict(op='event',data=row))
        elif row.get('sample') and row['elapsed_s']-self.last_anchor>=1.:
            self.last_anchor=row['elapsed_s']
            event={k:row[k] for k in ['ros_time_ns','worker_monotonic_s','execution_s','elapsed_s','sensor_stamp_ns','claw_stamp_ns','vision_frame_id'] if k in row}
            event['event']='clock_anchor';self.send(dict(op='event',data=event))

    def finish(self):
        if self.proc is None:return dict(valid=False,errors=['녹화 실행기 시작 실패'])
        self.stopping=True
        if not self.finished.is_set():
            try:self.send(dict(op='stop'))
            except Exception as exc:self.error=self.error or str(exc)
        # Bag indexing and full-data validation may take longer than motion.
        if not self.finished.wait(310):
            try:self.proc.stdin.close()
            except Exception:pass
            self.error=self.error or 'bag 저장·검사 시간 초과 (로봇의 원본 파일 유지)'
        try:self.proc.wait(timeout=3)
        except Exception:
            try:self.proc.terminate();self.proc.wait(timeout=3)
            except Exception:pass
        try:self.outgoing.put_nowait(None)
        except queue.Full:pass
        result=self.result or dict(valid=False,errors=[self.error or '녹화 종료 보고서 없음'])
        result=dict(result,robot_host=getattr(self.link,'host',None))
        if self.error:
            result=dict(result,valid=False,errors=list(result.get('errors',[]))+[self.error])
        return result
