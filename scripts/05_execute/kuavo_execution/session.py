"""Warp's complete preparation/replay/capture lifecycle.

Only configure() binds host resources; imports do not load a dashboard or ROS.
The host provides a control store, episode/vision callbacks and a data directory.
All readiness, ownership, lease, execution, capture and result policy lives here.
"""
import copy
import json
import math
import threading
import time
import uuid

from pathlib import Path
from . import first_state, package as playback, capture as collection

common = None
episodes = None
data_directory = None
setup_extra = []
commanders = lambda snapshot, within: {}
is_busy = lambda: False


def configure(*, control, episode_source, paths, ros_setup=(), command_sources=None, other_busy=None):
    """Bind host ports without initiating control, recording, network or threads.

    control exposes LINK/JOG/REPLAY/ACTIONS. episode_source supplies selected
    detail, entry, HEAD snapshots/overlay and the host's confirmation store.
    paths.path(*parts) resolves the host data root. Optional callbacks inspect
    other controllers; execution never stops or modifies those controllers.
    """
    global common, episodes, data_directory, setup_extra, commanders, is_busy
    with LOCK:
        if CURRENT is not None and CURRENT.running():
            raise ValueError('Cannot rebind an active Warp execution session')
        common = control
        episodes = episode_source
        data_directory = paths
        setup_extra = list(ros_setup)
        collection.SETUP_EXTRA = list(ros_setup)
        if command_sources is not None: commanders = command_sources
        if other_busy is not None: is_busy = other_busy

LOCK=threading.RLock()
CURRENT=None
PAGE_LEASE_S=2.
HEARTBEAT_S=.15


def observation(live,sample):
    if not live.get('ok') or not live.get('robot_error'):
        raise ValueError('새 랙 영상과 head→base_link TF를 먼저 확인하세요.')
    age=live.get('age_s',99)
    e=live['robot_error']
    values=[e['forward_m'],e['left_m'],math.radians(e['yaw_deg'])]
    if not all(math.isfinite(v) for v in values+[age]) or not 0<=age<1.5:
        raise ValueError('정렬 위치를 신뢰할 수 없습니다.')
    return dict(op='vision',frame_id=sample['frame_id'],age_s=age,error=values)


def unavailable():
    link=common.LINK
    if link is None or getattr(link,'status','')!='connected' or getattr(link,'host',None)=='offline':
        return '로봇 연결이 없습니다.'
    for holder in [common.JOG,common.REPLAY]:
        session=holder.get('session')
        if session is not None and session.status().get('alive'):
            return '다른 제어·재생이 진행 중입니다. 먼저 정지하세요.'
    if common.JOG.get('starting') or common.JOG.get('starting_since'):
        return '다른 제어를 시작하는 중입니다.'
    for key in ['mp','tag','align']:
        prog=common.ACTIONS.get(key)
        if prog is not None and prog.running():
            return '다른 이동·정렬이 진행 중입니다.'
    active=commanders(link.snapshot,within=2.)
    nodes=sorted({n for group in ['base','arm','torso','claw'] for n in active.get(group,{})})
    if nodes:return '다른 제어 명령이 들어오고 있습니다: '+', '.join(nodes)
    if is_busy():return 'PoC2 작업이 진행 중입니다. 먼저 정지하세요.'
    return ''


class Session:
    def __init__(self,detail,sid,replay_plan=None,stationary_confirmed=False,confirmed_state=None):
        self.confirmed_state=copy.deepcopy(confirmed_state)
        self.preparation_reused=False
        self.worker_phase='preparation'
        self.stationary_confirmed=stationary_confirmed is True
        self.replay_plan=replay_plan;self.kind='replay' if replay_plan is not None else 'align'
        self.vision_locked=False;self.base_goal_odom=None;self.base_odom_frame=None;self.base_error=None
        self.elapsed=0.;self.run_id=uuid.uuid4().hex if replay_plan is not None else None;self.run_dir=None;self.trace=None
        self.capture=None;self.collection_report=None
        self.tracking_samples=0;self.tracking_squared=0.;self.tracking_max=0.
        self.trace_error=''
        self.key=detail['id'];self.fingerprint=detail['fingerprint'];self.sid=sid
        self.stop_event=threading.Event();self.first_beat=threading.Event();self.write_lock=threading.Lock();self.lock=threading.RLock()
        self.proc=None;self.active=True;self.started=time.time();self.beat_at=time.monotonic()
        self.stage='replay 시작 준비 중' if replay_plan is not None else '정렬 시작 준비 중';self.error='';self.success=False;self.velocity=[0,0,0]
        self.message='';self.thread=threading.Thread(target=self.run,daemon=True,name='retargeting-align')

    def running(self):
        with self.lock:return self.active

    def status(self,sid=None):
        with self.lock:
            stage=self.stage
            if self.capture and not self.capture.ready.is_set() and self.capture.info.get('waiting'):stage+=' · '+self.capture.info['waiting']
            return dict(running=self.active,mine=self.sid==sid,id=self.key,stage=stage,error=self.error,
                        success=self.success,velocity=list(self.velocity),started_at=self.started,head_excluded=True,
                        kind=self.kind,vision_locked=self.vision_locked,base_error=self.base_error,elapsed_s=self.elapsed,duration_s=(self.replay_plan['interval'][1]-self.replay_plan['interval'][0] if self.replay_plan else None),run_id=self.run_id)

    def beat(self,sid,key,fingerprint):
        if sid!=self.sid or key!=self.key or fingerprint!=self.fingerprint:
            raise ValueError('이 페이지의 정렬 작업이 아닙니다.')
        self.beat_at=time.monotonic()
        self.first_beat.set()

    def write(self,data):
        with self.write_lock:
            if data.get('op') == 'beat' and self.stop_event.is_set():return
            if self.proc and self.proc.poll() is None:
                self.proc.stdin.write((json.dumps(data,separators=(',',':'))+'\n').encode());self.proc.stdin.flush()

    def stop(self,key=None,wait=0):
        self.stop_event.set()
        with self.lock:self.stage='정렬·replay 정지 중'
        try:self.write({'op':'stop'})
        except (OSError,ValueError):pass
        if wait and self.thread.ident is not None and self.thread is not threading.current_thread():self.thread.join(timeout=min(wait,3))

    def read(self):
        for line in iter(self.proc.stdout.readline,b''):
            if not line.startswith(b'{'):continue
            try:d=json.loads(line)
            except ValueError:continue
            with self.lock:
                if self.capture:
                    try:self.capture.ingest(d)
                    except (OSError,ValueError) as exc:self.error=str(exc);self.stop_event.set()
                if d.get('vision_locked'):self.vision_locked=True
                if d.get('base_goal_odom') is not None:self.base_goal_odom=d['base_goal_odom'];self.base_odom_frame=d.get('base_odom_frame')
                if d.get('base_error') is not None:self.base_error=d['base_error']
                if d.get('sample') and self.trace:
                    if d.get('actual_q') is not None and d.get('position') is not None:
                        errors=[a-b for a,b in zip(d['actual_q'],d['position'][3:])]
                        if len(errors)!=18 or not all(math.isfinite(v) for v in errors):
                            self.error=self.trace_error='실측 추종 기록 오류';self.stop_event.set()
                        else:
                            self.tracking_samples+=1;self.tracking_squared+=sum(v*v for v in errors)
                            self.tracking_max=max(self.tracking_max,max(map(abs,errors)))
                    try:self.trace.write(json.dumps(d,separators=(',',':'))+'\n')
                    except (OSError,ValueError) as exc:
                        self.error=self.trace_error='replay 실행 기록 저장 실패: '+str(exc);self.stop_event.set()
                if d.get('preparation') and self.trace:
                    try:self.trace.write(json.dumps(d,separators=(',',':'))+'\n')
                    except (OSError,ValueError) as exc:
                        self.error=self.trace_error='준비 단계 기록 저장 실패: '+str(exc);self.stop_event.set()
                if (self.trace and not d.get('sample') and not d.get('preparation')
                        and any(k in d for k in ('stage','error','done','event'))):
                    row=dict(d,phase=self.worker_phase,received_monotonic_s=time.monotonic(),wall_time_s=time.time())
                    try:self.trace.write(json.dumps(row,separators=(',',':'))+'\n');self.trace.flush()
                    except (OSError,ValueError) as exc:
                        self.error=self.trace_error='실행 단계 기록 저장 실패: '+str(exc);self.stop_event.set()
                if d.get('elapsed_s') is not None:self.elapsed=d['elapsed_s']
                if d.get('stage'):self.stage=d['stage']
                if d.get('error'):self.error=d['error']
                if d.get('velocity') is not None:self.velocity=d['velocity']
                if d.get('done'):self.success=bool(d.get('success'))

    def drain(self):
        for _ in iter(self.proc.stderr.readline,b''):pass

    def reuse_preparation(self,goal):
        """Recheck the accepted first alignment without commanding another move."""
        saved=self.confirmed_state
        if saved is None:return False  # Standalone clients may request full preparation.
        if (saved.get('id')!=self.key or saved.get('fingerprint')!=self.fingerprint
                or not all(saved.get('checks',{}).get(k) is True for k in ('robot','boxes'))):
            raise ValueError('첫 정렬 완료·배치 확인이 바뀌었습니다. 다시 확인하세요.')
        snapshot=common.LINK.snapshot;now=time.time()
        if not first_state.base_feedback_matches(saved.get('base_goal_odom'),saved.get('base_odom_frame'),snapshot,now):
            raise ValueError('첫 정렬 후 베이스 위치 또는 피드백이 바뀌었습니다. 다시 정렬하세요.')
        if not first_state.feedback_matches(saved.get('first_state_target',{}),snapshot,now):
            raise ValueError('첫 정렬 후 관절·그리퍼 자세 또는 피드백이 바뀌었습니다. 다시 확인하세요.')
        if not first_state.feedback_matches(goal,snapshot,now):
            raise ValueError('선택한 첫 state와 현재 자세가 다릅니다. 첫 정렬을 다시 실행하세요.')
        self.preparation_reused=True
        self.base_goal_odom=list(saved['base_goal_odom']);self.base_odom_frame=saved['base_odom_frame']
        row=dict(preparation=True,event='first_alignment_reused',phase='preparation',
                 wall_time_s=now,worker_monotonic_s=time.monotonic(),
                 base_goal_odom=self.base_goal_odom,base_odom_frame=self.base_odom_frame)
        if self.trace:self.trace.write(json.dumps(row,separators=(',',':'))+'\n');self.trace.flush()
        return True

    def heartbeat(self,finished,replaying):
        """Keep worker/capture leases independent of catalogue and recorder checks.

        Only the owning page renews beat_at. Vision-based preparation still needs
        real observations; heartbeat must never renew its vision input lease.
        """
        try:
            while not finished.is_set() and not self.stop_event.is_set() and self.proc.poll() is None:
                if time.monotonic()-self.beat_at>PAGE_LEASE_S:
                    raise ValueError('시작한 페이지와 연결이 끊겨 정지합니다.')
                if replaying or self.vision_locked:
                    self.write({'op':'beat'})
                if self.capture:self.capture.send({'op':'beat'})
                finished.wait(HEARTBEAT_S)
        except Exception as exc:
            self.error=self.error or str(exc)
            self.success=False
            self.stop_event.set()
            # Do not wait for catalogue/recorder work (or the trace reader lock)
            # before revoking the remote worker's permission to continue.
            try:self.write({'op':'stop'})
            except (OSError,ValueError):pass

    def execute_worker(self,goal,replay_plan=None):
        """Run one leased phase; preparation must finish before replay is launched."""
        self.success=False;self.vision_locked=False
        self.worker_phase='replay' if replay_plan is not None else 'preparation'
        if self.stop_event.is_set():raise ValueError('첫 자세·replay 시작 취소')
        if time.monotonic()-self.beat_at>PAGE_LEASE_S:raise ValueError('시작한 페이지 연결 소실')
        self.proc=common.LINK.popen_script(str(playback.WORKER),[],setup_extra=setup_extra)
        reader=threading.Thread(target=self.read,daemon=True);reader.start()
        threading.Thread(target=self.drain,daemon=True).start()
        config={'op':'configure','target':goal}
        if replay_plan is not None:config['replay']=replay_plan
        self.write(config)
        finished=threading.Event()
        heartbeat=threading.Thread(target=self.heartbeat,args=(finished,replay_plan is not None),
                                   daemon=True,name='retargeting-heartbeat')
        heartbeat.start()
        try:
            while self.proc.poll() is None and not self.stop_event.is_set():
                if time.monotonic()-self.beat_at>PAGE_LEASE_S:raise ValueError('시작한 페이지와 연결이 끊겨 정지합니다.')
                if self.capture:self.capture.check()
                detail=episodes.selection_detail(self.key,self.sid)
                if detail['fingerprint']!=self.fingerprint:raise ValueError('episode 결과가 바뀌어 정지합니다.')
                if self.stop_event.is_set():break
                if replay_plan is None and not self.vision_locked:
                    episodes.HEAD.ensure();sample,*_=episodes.HEAD.snapshot()
                    if not sample:raise ValueError('랙 영상 없음')
                    live=episodes.overlay(detail,sample)
                    packet=observation(live,sample)
                    self.write(packet)
                self.stop_event.wait(.15)
            if self.stop_event.is_set():self.stop()
            try:self.proc.wait(timeout=3)
            except Exception:
                self.proc.terminate();self.proc.wait(timeout=3)
            reader.join(timeout=3)
            if reader.is_alive():raise ValueError('replay 실행 기록 수신을 완료하지 못했습니다.')
            if self.proc.returncode!=0 or self.stop_event.is_set():self.success=False
        finally:
            finished.set()
            heartbeat.join(timeout=1.)
            if heartbeat.is_alive():
                self.stop_event.set()
                raise ValueError('heartbeat 전송 종료 지연: 제어 연결을 정리합니다.')
            if self.proc.stdin is not None:self.proc.stdin.close()

    def run(self):
        try:
            # Start must reach its owning, visible page before any robot process starts.
            while not self.first_beat.is_set() and not self.stop_event.is_set():
                if time.monotonic()-self.beat_at>PAGE_LEASE_S:raise ValueError('시작 확인 연결이 없어 취소했습니다.')
                self.stop_event.wait(.05)
            if self.stop_event.is_set():return
            detail=episodes.selection_detail(self.key,self.sid)
            if detail['fingerprint']!=self.fingerprint:raise ValueError('선택 구간이 바뀌었습니다.')
            goal=first_state.target(detail)
            goal['stationary_confirmed']=self.stationary_confirmed
            if self.replay_plan is not None:
                current=playback.prepare(episodes.entry(self.key),detail,fresh=True,allow_unverified=self.replay_plan.get('manual_standard_approval') is True)
                if current['payload_sha256']!=self.replay_plan['payload_sha256']:
                    raise ValueError('replay 궤적 파일이 바뀌었습니다. 다시 준비하세요.')
                # Source-phase grippers are interpolated with the same execution
                # clock; replace the nearest camera frame's preparation value.
                goal['claw']=playback.driver().ReplayPath(current).elapsed(0)[3]
                self.run_dir=Path(data_directory.path('retargeting','runs',self.run_id))
                self.run_dir.mkdir(parents=True,exist_ok=False)
                self.trace=(self.run_dir/'trace.jsonl').open('w',buffering=65536)
                self.manifest('starting')
                self.capture=collection.Capture(common.LINK,self.run_id)
                self.stage='카메라·센서 녹화 준비 중'
                self.capture.start()
                def check_capture_wait():
                    if self.stop_event.is_set():raise ValueError('녹화 준비 중 실행 취소')
                    if time.monotonic()-self.beat_at>PAGE_LEASE_S:raise ValueError('녹화 준비 중 페이지 연결 끊김')
                    if episodes.selection_detail(self.key,self.sid)['fingerprint']!=self.fingerprint:raise ValueError('녹화 준비 중 선택 구간 변경')
                self.capture.wait_ready(check_capture_wait)
                self.manifest('running')
            if self.replay_plan is not None:
                if self.reuse_preparation(goal):
                    self.stage='첫 정렬 완료 자세 재사용 · replay 제어 연결 중'
                else:
                    self.stage='replay 준비 · 선택한 첫 자세로 이동 중'
                    self.execute_worker(goal)
                    if not self.success or self.error or self.stop_event.is_set():
                        raise ValueError('replay 첫 자세 이동 실패: '+(self.error or '도착 확인 실패'))
                self.stage='첫 자세 도착 완료 · replay 제어 연결 중'
            self.execute_worker(goal,self.replay_plan)
            if self.success and self.replay_plan is None:
                if self.base_goal_odom is None or not first_state.base_feedback_matches(
                        self.base_goal_odom,self.base_odom_frame,common.LINK.snapshot,time.time()):
                    raise ValueError('랙 정렬 후 정지 위치의 odom 확인에 실패했습니다.')
                if not first_state.feedback_matches(goal,common.LINK.snapshot,time.time()):
                    raise ValueError('첫 자세의 몸통·양팔·그리퍼 실측 도착을 확인하지 못했습니다.')
                with episodes.LOCK:
                    if self.stop_event.is_set():raise ValueError('정렬 완료 확인 취소')
                    saved=episodes.SESSIONS.get(self.sid,{})
                    checks={'robot':True}
                    if (saved.get('id')==self.key and saved.get('fingerprint')==self.fingerprint
                        and saved.get('checks',{}).get('validation_override') is True):
                        checks['validation_override']=True
                    episodes.SESSIONS[self.sid]=dict(id=self.key,fingerprint=self.fingerprint,at=time.time(),checks=checks,first_state_target=goal,base_goal_odom=self.base_goal_odom,base_odom_frame=self.base_odom_frame)
        except Exception as exc:
            with self.lock:self.error=str(exc);self.success=False
            self.stop()
        finally:
            if self.proc:
                try:self.write({'op':'stop'})
                except (OSError,ValueError):pass
                try:self.proc.stdin.close()
                except (OSError,ValueError):pass
                try:self.proc.wait(timeout=3)
                except Exception:self.proc.terminate()
            if self.capture:
                self.stage='전체 데이터 저장·검사 중'
                if self.trace:
                    try:self.trace.close()
                    except (OSError,ValueError) as exc:self.trace_error='trace 저장 실패: '+str(exc)
                    self.trace=None
                try:
                    self.manifest('saving')
                except Exception as exc:self.error=self.error or str(exc)
                try:self.collection_report=self.capture.finish()
                except Exception as exc:self.collection_report=dict(valid=False,errors=[str(exc)])
                self.collection_report['bag_valid']=self.collection_report.get('valid') is True
                if not self.success or self.trace_error:
                    self.collection_report['valid']=False
                    self.collection_report.setdefault('errors',[]).append(self.trace_error or '선택한 replay 구간을 완료하지 못했습니다')
                if not self.collection_report.get('valid'):
                    self.error=self.error or '데이터 검사 실패: '+' / '.join(self.collection_report.get('errors',[]))
            with self.lock:
                self.active=False;self.velocity=[0,0,0]
                self.stage=('replay 종료·정지 확인 완료' if self.kind=='replay' else '첫 자세 도착·정지 확인 완료') if self.success else '실행 정지'
                if self.capture:self.stage='전체 데이터 저장·검사 완료' if self.collection_report.get('valid') else '데이터 저장·검사 실패'
            if self.trace:self.trace.close();self.trace=None
            if self.run_dir:self.manifest('failed' if self.capture and not self.collection_report.get('valid') else 'completed' if self.success else 'stopped')
            with LOCK:
                if common.ACTIONS.get('align') is self:common.ACTIONS['align']=None
            path=Path(data_directory.path('retargeting','events.jsonl'));path.parent.mkdir(parents=True,exist_ok=True)
            with path.open('a') as f:
                f.write(json.dumps(dict(t=time.time(),episode=self.key,op=self.kind+'_result',success=self.success,error=self.error,run_id=self.run_id,stationary_confirmed=self.stationary_confirmed))+'\n')


    def manifest(self,status):
        data=dict(source_episode=self.key,status=status,started_at=self.started,
                  duration_s=self.elapsed,success=None,execution_completed=self.success,
                  fingerprint=self.fingerprint,selection_interval=self.replay_plan['interval'],
                  rate=1.,kalman_applied=False,stationary_confirmed=self.stationary_confirmed,
                  preparation_reused=self.preparation_reused,
                  trace_schema_version=2,base_feedback_enabled=False,base_endpoint_verified=False,
                  executor='Warp/scripts/05_execute/kuavo_replay.py',controller_pva_exact=False,
                  manual_standard_approval=self.replay_plan.get('manual_standard_approval',False),
                  standard_status=self.replay_plan.get('standard_status'),artifact_sha256=self.replay_plan['artifact_sha256'],
                  payload_sha256=self.replay_plan['payload_sha256'],error=self.error,
                  collection=self.collection_report or (dict(valid=None,ready=self.capture.ready.is_set(),**{k:v for k,v in self.capture.info.items() if k in ('path','ros_time_ns','run_id')}) if self.capture else None),
                  collection_valid=self.collection_report.get('valid') if self.collection_report else None,
                  tracking=dict(samples=self.tracking_samples,maximum_joint_error_deg=math.degrees(self.tracking_max),
                    rms_joint_error_deg=math.degrees(math.sqrt(self.tracking_squared/(18*self.tracking_samples))) if self.tracking_samples else None),
                  clock_domains=dict(ros_time_ns='Up ROS clock',worker_monotonic_s='executor monotonic clock',
                    sensor_stamp_ns='original sensor_time or header stamp',claw_stamp_ns='original claw header stamp',vision_frame_id='head image header stamp in ns'),
                  task_label='unlabeled',
                  note='전체 RGB-D·센서 rosbag과 원본 시각을 함께 저장합니다. 작업 성공·실패는 저장 후 직접 표시합니다.')
        temp=self.run_dir/'manifest.tmp';temp.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
        temp.replace(self.run_dir/'manifest.json')


def start(detail,sid,sample,live,replay_plan=None,stationary_confirmed=False):
    global CURRENT
    first_state.target(detail)
    obs=observation(live,sample)
    if math.hypot(*obs['error'][:2])>.75 or abs(obs['error'][2])>math.radians(45):
        raise ValueError('자동 정렬 범위는 현재 위치에서 75cm·45° 이내입니다.')
    with LOCK:
        reason=unavailable()
        if reason:raise ValueError(reason)
        confirmed=None
        if replay_plan is not None:
            with episodes.LOCK:confirmed=copy.deepcopy(episodes.SESSIONS.get(sid,{}))
        session=Session(detail,sid,replay_plan,stationary_confirmed=stationary_confirmed,confirmed_state=confirmed);CURRENT=session;common.ACTIONS['align']=session
        session.thread.start()
        return session.status(sid)


def status(sid):
    with LOCK:return CURRENT.status(sid) if CURRENT else {'running':False,'mine':False,'success':False,'stage':'대기'}


def stop():
    with LOCK:
        if CURRENT and CURRENT.running():CURRENT.stop()
    return {'ok':True,'text':'정렬·replay 정지를 요청했습니다.'}


def beat(key,sid,fingerprint):
    with LOCK:
        if not CURRENT or not CURRENT.running():return {'ok':False}
        CURRENT.beat(sid,key,fingerprint)
    return {'ok':True}
