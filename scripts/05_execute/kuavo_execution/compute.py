"""Persistent, two-slot offline standard queue. Never imports robot control."""
import argparse
from contextlib import contextmanager
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback
import uuid

DATA = Path(os.environ.get('KUAVO_DATA', os.environ.get('WARP_EXECUTION_DATA_ROOT', Path.home()/'Warp_data'))) / 'retargeting'
CATALOG = DATA / 'catalog.json'
JOBS = DATA / 'compute'
STANDARD = Path('/home/lab6/Warp-review/wilor-kuavo-standard')
PYTHON = '/home/lab6/Warp/.venv-retarget/bin/python'
WRAPPER = Path('/home/lab6/Warp/scripts/02_convert/convert_episode_standard.sh')
MAX_PARALLEL = 2
CALIBRATION_CAPTURE = None
BASE_LINK_PREPARER = Path(__file__).resolve().parents[2]/'02_convert/prepare_base_link.py'
ACTIVE = ('queued', 'running')
PROTECTED = ('CURRENT.json','standard.py','standard_config.json','STANDARD.json','motion_policy.json','pipeline/solve.py','pipeline/render.py')


def read(path):
    return json.loads(Path(path).read_text())


def write(path, data):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    tmp=path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
    tmp.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n');tmp.replace(path)


def digest(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()


@contextmanager
def locked(name='queue'):
    JOBS.mkdir(parents=True,exist_ok=True)
    with (JOBS/(name+'.lock')).open('a') as f:
        fcntl.flock(f,fcntl.LOCK_EX)
        yield


def jobs():
    p=JOBS/'jobs.json'
    return read(p) if p.exists() else {}


def update(key, **values):
    with locked():
        rows=jobs();rows[key].update(values,updated_at=time.time());write(JOBS/'jobs.json',rows)


def status():
    with locked():rows=jobs()
    public=[]
    labels={'solve.py.log':'IK','retime.py.log':'시간 조정','check.py.log':'궤적 검증','moveit.log':'MoveIt 검증','render.py.log':'4-window 렌더링'}
    for row in rows.values():
        item=dict(row)
        result=Path(row['result']) if row.get('result') else None
        if row['status']=='running' and result and (result/'logs').exists():
            logs=[p for p in (result/'logs').glob('*.log') if p.name in labels]
            if logs and row.get('stage')=='standard 실행':item['stage']=labels[max(logs,key=lambda p:p.stat().st_mtime).name]
        log=JOBS/row['id']/'worker.log'
        if log.exists():
            with log.open('rb') as f:
                f.seek(max(0,log.stat().st_size-4000));item['log_tail']=f.read().decode(errors='replace')
        public.append(item)
    return dict(max_parallel=MAX_PARALLEL,jobs=public)


def capture_calibration(key):
    if CALIBRATION_CAPTURE is not None:
        return CALIBRATION_CAPTURE(key)
    from urllib.request import urlopen
    from urllib.parse import quote
    from urllib.error import HTTPError, URLError
    base = os.environ.get('KUAVO_CALIBRATION_URL', 'http://127.0.0.1:7860/kuavo/retargeting/compute/calibration/')
    try:
        with urlopen(base+quote(key, safe=''), timeout=15) as response:
            return json.load(response)
    except HTTPError as exc:
        try: message=json.loads(exc.read()).get('detail', str(exc))
        except ValueError: message=str(exc)
        raise ValueError('고정 높이 계산 거부: '+message) from exc
    except (URLError, TimeoutError) as exc:
        raise ValueError('고정 높이 계산 설정을 불러올 수 없습니다.') from exc


def validate_tcp_lowering_cm(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 <= value <= 30:
        raise ValueError('TCP 하향 보정은 0–30 cm 사이 숫자로 입력하세요.')
    return float(value)


def enqueue(keys, tcp_lowering_cm=3.):
    tcp_lowering_cm = validate_tcp_lowering_cm(tcp_lowering_cm)
    if not isinstance(keys,list) or not keys or len(keys)>50 or any(not isinstance(k,str) for k in keys):
        raise ValueError('계산할 episode를 선택하세요.')
    catalog={e['id']:e for e in read(CATALOG)['episodes']}
    if any(k not in catalog for k in keys):raise ValueError('목록에 없는 episode입니다.')
    # Validate original recording height references before mutating the queue.
    calibrations={key:capture_calibration(key) for key in dict.fromkeys(keys)}
    with locked():
        # A deletion may have completed during the calibration preflight.
        catalog={e['id']:e for e in read(CATALOG)['episodes']}
        if any(k not in catalog for k in keys):raise ValueError('삭제된 episode는 계산할 수 없습니다.')
        rows=jobs();accepted=[]
        for key in dict.fromkeys(keys):
            if key in rows and rows[key]['status'] in ACTIVE:
                accepted.append(rows[key]['id']);continue
            e=catalog[key]
            if not Path(e.get('input_prepared') or e['prepared']).is_file():raise ValueError('준비된 입력이 없습니다: '+key)
            jid=uuid.uuid4().hex
            previous=rows.get(key,{})
            rows[key]=dict(id=jid,episode=key,label=e.get('label',key),status='queued',stage='대기',created_at=time.time(),updated_at=time.time(),
                           coordinate_frame='fixed_rack_height',calibration_at=calibrations[key]['captured_at'],
                           tcp_lowering_cm=tcp_lowering_cm,tcp_lowering_frame='box_tag',
                           superseded_result=e.get('result') or previous.get('result'),
                           superseded_media=str(Path(e['video_manifest']).parent) if e.get('video_manifest') else None)
            accepted.append(jid)
        write(JOBS/'jobs.json',rows)
    return dict(ok=True,job_ids=accepted,max_parallel=MAX_PARALLEL)


def cancel(keys):
    if not isinstance(keys,list) or any(not isinstance(k,str) for k in keys):raise ValueError('잘못된 계산 중단 요청')
    with locked():
        rows=jobs()
        for key in keys:
            if key in rows and rows[key]['status'] in ACTIVE:
                rows[key]['cancel_requested']=True
                if rows[key]['status']=='queued':rows[key].update(status='cancelled',stage='취소됨')
        write(JOBS/'jobs.json',rows)
    return {'ok':True}


def prepare(entry, directory):
    """Reuse this take's measured goals/torso/base; repair nominal arm seeds only."""
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=False)
    source=Path(entry.get('input_prepared') or entry['prepared'])
    spec=read(source)
    def resolved(k):
        p=Path(spec[k]);return p if p.is_absolute() else (source.parent/p).resolve()
    rows=[json.loads(s) for s in resolved('reference').read_text().splitlines() if s.strip()]
    full_end=max(r['elapsed_s'] for r in rows)
    end=min(full_end-2.,float(entry.get('source_cutoff_s',float('inf'))))
    config=read(resolved('reference_config'));nominal=read(STANDARD/'standard_config.json')['ik']['initial_q'][4:]
    config['ik']['initial_q'][4:]=nominal
    kept=[]
    for r in rows:
        if r['elapsed_s']>end+1e-9:continue
        if r.get('q') is not None:
            r['q'][4:]=nominal
            r['reason']='prepared_inputs_with_own_torso_base_standard_nominal_arm_seed_unverified'
        kept.append(r)
    if sum(bool(r.get('input_available',r.get('valid'))) for r in kept)<3:raise ValueError('끝 2초 제외 후 유효 입력이 부족합니다.')
    reference=directory/'reference.jsonl';reference.write_text(''.join(json.dumps(r)+'\n' for r in kept))
    write(directory/'reference-config.json',config)
    bundle=dict(episode=spec['episode'],reference=str(reference),world=str(resolved('world')),
                reference_config=str(directory/'reference-config.json'),display_source=str(resolved('display_source')))
    write(directory/'prepared.json',bundle)
    write(directory/'PREPARATION.json',dict(arm_initialization_fixed=True,discard_recording_tail_s=2.,
        full_source_end_s=full_end,prepared_source_end_s=kept[-1]['elapsed_s'],input_descriptor=str(source),robot_commands_sent=0))
    return directory/'prepared.json',bundle


def standard_command(spec, prepared, name):
    return ['bash',str(WRAPPER),spec['episode'],'--prepared',str(prepared),'--run-name',name]


def run_checked(command, env):
    subprocess.run(command,env=env,check=True)


def worker(key, jid):
    row=next(e for e in read(CATALOG)['episodes'] if e['id']==key)
    directory=JOBS/jid
    before={p:digest(STANDARD/p) for p in PROTECTED}
    update(key,stage='실측 태그 고정 높이 적용')
    calibration=capture_calibration(key)
    write(directory/'calibration.json',calibration)
    update(key,calibration_at=calibration['captured_at'])
    with locked(): job = jobs().get(key, {})
    if job.get('id') != jid:
        raise ValueError('계산 작업 ID가 변경됐습니다.')
    tcp_lowering_cm = validate_tcp_lowering_cm(job.get('tcp_lowering_cm', 0.))
    source_prepared,_=prepare(row,directory/'recording-input')
    run_checked([PYTHON,str(BASE_LINK_PREPARER),'--prepared',str(source_prepared),
                 '--calibration',str(directory/'calibration.json'),'--output',str(directory/'input'),
                 '--tcp-lowering-cm',str(tcp_lowering_cm)],
                dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1'))
    prepared=directory/'input/prepared.json';spec=read(prepared)
    name=key+'-dashboard-'+jid[:12]
    result=STANDARD/'results'/name
    update(key,stage='입력 확인',result=str(result))
    env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',MUJOCO_GL='egl',PYTHONDONTWRITEBYTECODE='1')
    command=standard_command(spec,prepared,name)
    run_checked(command+['--dry-run'],env)
    # The original, display-only head interpolation repair, now take-parameterized.
    provider=Path(spec['display_source'])
    if not (provider/'results'/key/'upper-right-flicker-fix.json').is_file():
        update(key,stage='카메라 영상 준비')
        run_checked([PYTHON,str(Path(__file__).with_name('compute_camera.py')),key,str(provider)],
                    dict(env,PYTHONPATH='/home/lab6/humanoid/box_detection/.venv/lib/python3.10/site-packages'))
    update(key,stage='standard 실행')
    code=subprocess.run(command,env=env).returncode
    report=read(result/'REPORT.json') if (result/'REPORT.json').is_file() else {}
    state=report.get('status','failed')
    if state not in ('verified','rejected_by_dense_validation'):
        raise RuntimeError('standard 계산 실패 (exit %d): %s'%(code,state))
    if code not in (0,2):raise RuntimeError('standard 실행 오류: %d'%code)
    update(key,stage='4-window 렌더링')
    video=result/'results/four-views-parameter-winner.mp4'
    if state=='rejected_by_dense_validation' and not video.is_file():
        # Exact copied adopted renderer; this never changes the rejection status.
        run_checked([PYTHON,str(result/'render.py')],env)
    update(key,stage='영상 확인')
    run_checked([PYTHON,str(Path(__file__).with_name('compute_video.py')),str(result),str(directory),
                 str(row.get('preview_cutoff_s') or 0)],env)
    media=read(directory/'MEDIA.json')
    if {p:digest(STANDARD/p) for p in PROTECTED}!=before:raise RuntimeError('계산 중 standard 설정이 변경됐습니다. 결과 연결 중단.')
    with locked():
        if jobs()[key].get('cancel_requested'):return
    with locked('catalog'):
        catalog=read(CATALOG)
        for e in catalog['episodes']:
            if e['id']==key:
                e.update(prepared=str(prepared),result=str(result),video=media['video'],video_sha256=media['video_sha256'],
                         trajectory_sha256=media['trajectory_sha256'],review_duration_s=media['duration_s'],
                         trimmed_source=True,arm_initialization_fixed=True,coordinate_frame='fixed_rack_height',
                         calibration_at=calibration['captured_at'],tcp_lowering_cm=tcp_lowering_cm,tcp_lowering_frame='box_tag',video_manifest=str(directory/'MEDIA.json'))
        write(CATALOG,catalog)
    update(key,status='done' if state=='verified' else 'rejected',stage='검증 통과' if state=='verified' else '검토 영상 준비 · 검증 미통과',
           validation_status=state,finished_at=time.time())


def cleanup(dry_run=False):
    try:
        from . import retention
    except ImportError:
        import retention
    # Same lock order as queue mutation; publication releases catalog before update.
    with locked():
        with locked('catalog'):
            return retention.prune(DATA,STANDARD,read(CATALOG),jobs(),dry_run=dry_run)

def cleanup_safely():
    try:
        cleanup()
    except Exception:
        # A cleanup error must never turn a published result into a compute failure.
        traceback.print_exc()


def schedule(rows,running):
    for key,process in list(running.items()):
        r=rows[key]
        if r.get('cancel_requested') and process.poll() is None:
            import signal
            try:
                if 'termination_requested_at' not in r:r['termination_requested_at']=time.time()
                os.killpg(process.pid,signal.SIGKILL if time.time()-r['termination_requested_at']>5 else signal.SIGTERM)
            except ProcessLookupError:pass
        code=process.poll()
        if code is not None:
            if r.get('cancel_requested'):r.update(status='cancelled',stage='취소됨')
            elif r['status']=='running':r.update(status='failed',stage='계산 오류 · 로그 확인',exit_code=code)
            r['finished_at']=time.time();del running[key]
    waiting=sorted((r for r in rows.values() if r['status']=='queued'),key=lambda r:r['created_at'])
    for r in waiting[:max(0,MAX_PARALLEL-len(running))]:
        # Keep space for the dashboard and existing camera/control processes.
        available=next(int(l.split()[1])*1024 for l in Path('/proc/meminfo').read_text().splitlines() if l.startswith('MemAvailable:'))
        if available<1500*1024**2:r['stage']='메모리 여유 대기';break
        directory=JOBS/r['id'];directory.mkdir(exist_ok=True)
        write(directory/'JOB.json',dict(id=r['id'],episode=r['episode'],tcp_lowering_cm=r.get('tcp_lowering_cm',0.),tcp_lowering_frame='box_tag'))
        with (directory/'worker.log').open('ab') as log:
            process=subprocess.Popen([PYTHON,str(Path(__file__).resolve()),'work',r['episode'],r['id']],
                env=dict(os.environ,KUAVO_DATA=str(DATA.parent),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MUJOCO_GL='egl'),
                stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        r.update(status='running',stage='입력 준비',pid=process.pid,started_at=time.time())
        running[r['episode']]=process


def supervise():
    JOBS.mkdir(parents=True,exist_ok=True)
    singleton=(JOBS/'supervisor.lock').open('a')
    try:fcntl.flock(singleton,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:return
    with locked():
        rows=jobs()
        for r in rows.values():
            if r['status']=='running':r.update(status='failed',stage='계산 서비스가 재시작되어 중단됨')
        write(JOBS/'jobs.json',rows)
    cleanup_safely()
    running={}
    while True:
        with locked():
            rows=jobs()
            completed=any(p.poll() is not None for p in running.values())
            schedule(rows,running)
            write(JOBS/'jobs.json',rows)
        if completed:cleanup_safely()
        time.sleep(1)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('command',choices=['supervise','work','cleanup','cleanup-dry-run']);parser.add_argument('episode',nargs='?');parser.add_argument('job',nargs='?');a=parser.parse_args()
    if a.command=='supervise':supervise()
    elif a.command.startswith('cleanup'):
        print(json.dumps(dict(removed=cleanup(dry_run=a.command=='cleanup-dry-run')),ensure_ascii=False,indent=2))
    else:
        try:worker(a.episode,a.job)
        except Exception:
            traceback.print_exc();update(a.episode,status='failed',stage='계산 오류 · 로그 확인',error=traceback.format_exc()[-3000:]);raise


if __name__=='__main__':
    main()
