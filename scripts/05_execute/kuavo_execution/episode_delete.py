"""Confirmed lab6 source/offline-output deletion; never issues robot commands."""
from contextlib import contextmanager, nullcontext
from pathlib import Path
import fcntl, hashlib, json, os, re, shutil, time, uuid, copy, stat

RAW_ROOT=Path('/home/lab6/Warp_data')
KEY=re.compile(r'\d{2}-\d{2}-\d{2}_[0-9a-f]{32}')
TICKETS={}

def _read(p):return json.loads(Path(p).read_text())
def _paths(x):
 if isinstance(x,str) and x.startswith('/'):yield Path(x).resolve()
 elif isinstance(x,dict):
  for v in x.values():yield from _paths(v)
 elif isinstance(x,list):
  for v in x:yield from _paths(v)
def _inside(p,root):return p==root or root in p.parents

@contextmanager
def _locks(compute,busy_guard):
 with compute.locked():
  with compute.locked('catalog'):
   with busy_guard() if busy_guard else nullcontext():
    p=RAW_ROOT/'.raw_processing/.worker.lock';p.parent.mkdir(parents=True,exist_ok=True)
    with p.open('a') as f:
     try:fcntl.flock(f,fcntl.LOCK_EX|fcntl.LOCK_NB)
     except BlockingIOError:raise ValueError('원본 처리 중입니다. 처리 종료 후 삭제하세요.')
     yield

def _tree(p):
 """Inventory uses lstat; symlink targets are never traversed or removed."""
 records=[]
 def record(q):
  s=q.lstat();records.append((str(q.relative_to(p)),s.st_mode,s.st_size,s.st_mtime_ns))
 record(p)
 if p.is_dir():
  for root,dirs,files in os.walk(p,followlinks=False):
   for name in dirs+files:record(Path(root)/name)
 return records

def _plan(compute,key):
 if not isinstance(key,str) or not KEY.fullmatch(key):raise ValueError('잘못된 episode ID')
 catalog=compute.read(compute.CATALOG);entry=next((e for e in catalog['episodes'] if e['id']==key),None)
 if entry is None:raise ValueError('episode를 찾을 수 없습니다.')
 jobs=compute.jobs()
 if jobs.get(key,{}).get('status') in compute.ACTIVE:raise ValueError('계산 중이거나 대기 중인 episode는 먼저 중단하세요.')
 current=compute.STANDARD/'CURRENT.json'
 adopted=_read(current) if current.exists() else {}
 if adopted.get('take')==key:raise ValueError('현재 채택 standard episode는 삭제할 수 없습니다.')
 specs=[]
 for name in ('input_prepared','prepared'):
  if entry.get(name) and Path(entry[name]).is_file():specs.append((Path(entry[name]),_read(entry[name])))
 raw=[]
 for _,spec in specs:
  if spec.get('episode'):raw.append(Path(spec['episode']))
 if not raw:raise ValueError('원본 녹화 경로를 확인할 수 없습니다.')
 raw={p.resolve() for p in raw}
 if len(raw)!=1:raise ValueError('원본 녹화 경로가 일치하지 않습니다.')
 raw=raw.pop();rawroot=RAW_ROOT.resolve()
 if raw.name!=key or raw.parent.parent!=rawroot or not re.fullmatch(r'\d{4}-\d{2}-\d{2}',raw.parent.name):
  raise ValueError('관리되는 원본 episode 폴더가 아닙니다.')
 if raw.is_dir() and (raw/'manifest.json').is_file():
  state=_read(raw/'manifest.json')
  if state.get('state',state.get('status')) in ('recording','receiving','uploading','processing'):
   raise ValueError('원본 녹화·전송·처리 중에는 삭제할 수 없습니다.')
 resultroot=(compute.STANDARD/'results').resolve();owned=set()
 for p in resultroot.iterdir():
  if p.is_symlink() or not p.is_dir():continue
  policy=p/'POLICY.json'
  if policy.is_file() and _read(policy).get('take')==key:owned.add(p.resolve())
 # Diagnostic results that explicitly reuse this episode's owned IK inputs.
 for p in resultroot.iterdir():
  if p.is_symlink() or not p.is_dir() or p.resolve() in owned:continue
  if (p/'data').is_symlink() and any(_inside((p/'data').resolve(),o) for o in owned):owned.add(p.resolve())
 candidates={raw}|owned
 reviewroot=compute.STANDARD.parent.resolve()
 for descriptor,spec in specs:
  parent=descriptor.parent.resolve()
  if parent.name==key and _inside(parent,reviewroot):candidates.add(parent)
 for p in compute.JOBS.iterdir():
  if p.is_symlink() or not p.is_dir():continue
  job=p/'JOB.json';media=p/'MEDIA.json'
  if job.is_file() and _read(job).get('episode')==key:candidates.add(p.resolve())
  elif media.is_file() and Path(_read(media).get('result','')).resolve() in owned:candidates.add(p.resolve())
 mediaroot=compute.DATA/'media'
 if mediaroot.exists():
  for p in mediaroot.iterdir():
   if not p.is_symlink() and p.is_dir() and (p/'MEDIA.json').is_file() and Path(_read(p/'MEDIA.json').get('result','')).resolve() in owned:candidates.add(p.resolve())
 # Only this source's processing receipt; the worker lock itself is retained.
 receipt=rawroot/'.raw_processing'/(key.rsplit('_',1)[1]+'.json')
 if receipt.is_file():
  state=_read(receipt)
  if Path(state.get('source','')).resolve()==raw:
   if state.get('state') in ('running','processing'):raise ValueError('원본 처리 중에는 삭제할 수 없습니다.')
   candidates.add(receipt)
 protected=set(_paths(adopted))
 for other in catalog['episodes']:
  if other['id']==key:continue
  protected.update(_paths(other))
  for name in ('prepared','input_prepared'):
   if other.get(name) and Path(other[name]).is_file():protected.update(_paths(_read(other[name])))
 for other,j in jobs.items():
  if other!=key and j.get('status') in compute.ACTIVE:protected.update(_paths(j))
 roots=[]
 for p in sorted(candidates,key=lambda p:(len(p.parts),str(p))):
  if p.is_symlink():raise ValueError('삭제 루트가 심볼릭 링크입니다: '+str(p))
  if not p.exists() or any(_inside(p,q) for q in roots):continue
  if any(_inside(q,p) or _inside(p,q) for q in protected):
   raise ValueError('다른 episode 또는 채택 standard가 사용하는 경로입니다: '+str(p))
  roots.append(p)
 inventory=[];size=0;count=0
 for p in roots:
  records=_tree(p);size+=sum(r[2] for r in records if not stat.S_ISDIR(r[1]));count+=len(records)
  inventory.append(dict(path=str(p),records=records))
 fingerprint=hashlib.sha256(json.dumps(dict(key=key,entry=entry,job=jobs.get(key),inventory=inventory),sort_keys=True).encode()).hexdigest()
 return dict(episode=key,label=entry.get('label',key),bytes=size,entries=count,paths=[str(p) for p in roots],
  fingerprint=fingerprint,scope='lab6 원본 녹화·전처리 입력·offline 계산 결과·영상 영구 삭제',
  device_note='Quest 기기 파일과 실제 로봇 수집 기록은 삭제하지 않습니다.')

def preview(compute,key,busy_guard=None):
 with _locks(compute,busy_guard):
  plan=_plan(compute,key);ticket=uuid.uuid4().hex
  now=time.monotonic()
  for k in list(TICKETS):
   if TICKETS[k]['expires']<now:TICKETS.pop(k,None)
  TICKETS[ticket]=dict(key=key,fingerprint=plan['fingerprint'],expires=now+180.)
  return dict(plan,ticket=ticket,expires_in_s=180)

def delete(compute,key,ticket,confirmed=False,busy_guard=None):
 if confirmed is not True:raise ValueError('삭제 확인이 필요합니다.')
 with _locks(compute,busy_guard):
  saved=TICKETS.pop(ticket,None) if isinstance(ticket,str) else None
  if not saved or saved['key']!=key or saved['expires']<time.monotonic():raise ValueError('삭제 확인이 만료됐습니다. 다시 확인하세요.')
  plan=_plan(compute,key)
  if plan['fingerprint']!=saved['fingerprint']:raise ValueError('삭제 대상이 변경됐습니다. 다시 확인하세요.')
  tx=uuid.uuid4().hex;moved=[]
  catalog=compute.read(compute.CATALOG);original_catalog=copy.deepcopy(catalog)
  old=next(e for e in catalog['episodes'] if e['id']==key)
  jobs=compute.jobs();original_jobs=copy.deepcopy(jobs);oldjob=jobs.pop(key,None)
  audit=compute.DATA/'deletions'/(tx+'.json')
  compute.write(audit,dict(plan=plan,episode_entry=old,job=oldjob,status='preparing',robot_commands_sent=0))
  try:
   for value in plan['paths']:
    p=Path(value);quarantine=p.with_name('.delete-'+tx+'-'+p.name)
    p.rename(quarantine);moved.append((p,quarantine))
   catalog['episodes']=[e for e in catalog['episodes'] if e['id']!=key]
   compute.write(compute.CATALOG,catalog);compute.write(compute.JOBS/'jobs.json',jobs)
  except Exception:
   for p,q in reversed(moved):q.rename(p)
   compute.write(compute.CATALOG,original_catalog);compute.write(compute.JOBS/'jobs.json',original_jobs)
   raise
  pending=[]
  for p,q in moved:
   try:
    if q.is_dir():shutil.rmtree(q)
    else:q.unlink()
   except OSError as exc:pending.append(dict(path=str(q),error=str(exc)))
  compute.write(audit,dict(plan=plan,episode_entry=old,job=oldjob,status='cleanup_pending' if pending else 'deleted',
    cleanup_pending=pending,robot_commands_sent=0))
  return dict(ok=True,episode=key,bytes=plan['bytes'],deleted_paths=plan['paths'],cleanup_pending=pending)
