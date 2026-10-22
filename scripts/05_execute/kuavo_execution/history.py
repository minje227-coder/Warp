"""Warp replay history, metadata and manual task labels.

Runs write runs/<id>/manifest.json (or run.json/meta.json) with source_episode,
status, started_at, duration_s, success, recording_path and optional videos.
The existing recorder may instead attach a retargeting.episode field to meta.json.
Ordinary PoC/manual recordings are not relabelled as retargeting executions.
"""
import hashlib
import json
import math
import threading
import time
import uuid
from pathlib import Path

import os
import shutil


class DataDirectory:
    """Explicit storage root for standalone clients; dashboard may bind a provider."""
    def __init__(self, root=None):
        self.root = root

    def path(self, *parts):
        root = self.root or os.environ.get('WARP_EXECUTION_DATA_ROOT')
        if not root:
            raise ValueError('Warp execution data root is not configured')
        return str(Path(root).joinpath(*parts))


datadir = DataDirectory()

LOCK=threading.RLock()
ACTIVE={'starting','recording','running','saving'}
RECOVERY_LIMIT=3


def _source(meta):
    context=meta.get("retargeting")
    context=context if isinstance(context,dict) else {}
    value=meta.get("source_episode") or context.get("source_episode") or context.get("episode")
    if not isinstance(value,str) or not value.strip():
        return None
    return Path(value).name


def _number(value):
    if isinstance(value,bool):
        return None
    try:
        value=float(value)
        return value if math.isfinite(value) and value>=0 else None
    except (ValueError,TypeError):
        return None


def _inside(path,folder):
    path=path.resolve()
    return path.is_relative_to(folder.resolve()) and path.is_file()


def _scan():
    runs=Path(datadir.path("retargeting","runs"))
    recorded=Path(datadir.path("recordings"))
    rows=[];errors=0
    for root,is_run in ((runs,True),(recorded,False)):
        if not root.is_dir():
            continue
        for folder in root.iterdir():
            if not folder.is_dir() or folder.is_symlink():
                continue
            candidates=("manifest.json","run.json","meta.json") if is_run else ("meta.json",)
            meta_path=next((folder/n for n in candidates if (folder/n).is_file()),None)
            if not meta_path:
                continue
            try:
                if not _inside(meta_path,root):
                    continue
                meta=json.loads(meta_path.read_text(encoding="utf-8"))
                if not isinstance(meta,dict):
                    raise ValueError("manifest must be an object")
                source=_source(meta)
                if source is None:
                    continue
                key=hashlib.sha256(str(meta_path.resolve()).encode()).hexdigest()[:24]
                success=meta.get("success") if isinstance(meta.get("success"),bool) else None
                status=str(meta.get("status") or meta.get("state") or ("saved" if not is_run else "unknown"))[:80]
                # A completed run alone does not establish task success.
                row=dict(id=key,name=str(meta.get("name") or folder.name)[:180],source_episode=source,
                         status=status,started_at=_number(meta.get("started_at")),
                         duration_s=_number(meta.get("duration_s",meta.get("duration"))),success=success,
                         note=str(meta.get("task_note") or meta.get("note") or meta.get("message") or "")[:1000],
                         collection_valid=meta.get('collection_valid'),execution_completed=meta.get('execution_completed'))
                rows.append((row,folder,meta))
            except (OSError,ValueError,TypeError):
                errors+=1
    rows.sort(key=lambda r:(r[0]["started_at"] or 0,r[0]["name"]),reverse=True)
    return rows,errors


def listing():
    rows,errors=_scan()
    return {"recordings":[r[0] for r in rows],"unreadable":errors}


def dashboard_runs():
    """Share run identities with the dashboard without changing recorder formats."""
    root=Path(datadir.path('retargeting','runs')).resolve()
    rows=[]
    for row,folder,meta in _scan()[0]:
        if folder.resolve().parent!=root:
            continue
        collection=meta.get('collection') or {}
        size=_number(collection.get('bytes')) if isinstance(collection,dict) else None
        rows.append(dict(row,kind='retargeting_run',run_id=row['id'],
            duration=row['duration_s'],size_mb=size/1e6 if size else None,
            note='Replay · 원본 '+row['source_episode'].split('_')[0]))
    return rows


def _find(key):
    for row,folder,meta in _scan()[0]:
        if row["id"]==key:
            return row,folder,meta
    raise ValueError("Replay 녹화 기록을 찾을 수 없습니다.")


def _videos(folder,meta):
    videos=meta.get("videos") or {}
    videos=videos.items() if isinstance(videos,dict) else []
    out=[];seen=set()
    for label,value in videos:
        if not isinstance(value,str):
            continue
        path=Path(value)
        path=path if path.is_absolute() else folder/path
        if path.suffix.lower()==".mp4" and _inside(path,folder):
            out.append((str(label)[:80],path.resolve()));seen.add(path.resolve())
    for path in sorted(folder.glob("*.mp4")):
        if _inside(path,folder) and path.resolve() not in seen:
            out.append((path.stem,path.resolve()))
    return out


def detail(key):
    row,folder,meta=_find(key)
    files=[]
    for path in sorted(folder.iterdir()):
        if _inside(path,folder):
            files.append({"name":path.name,"bytes":path.stat().st_size})
    return dict(row,recording_path=str(folder),files=files,collection=meta.get('collection'),tracking=meta.get('tracking'),
                can_label=row['status'] not in ACTIVE,task_note=meta.get('task_note',''),
                videos=[{"index":i,"label":label} for i,(label,_) in enumerate(_videos(folder,meta))])



def rename(key,name):
    if not isinstance(name,str):
        raise ValueError("녹화 이름을 문자열로 입력하세요.")
    name=name.strip()
    if not name or len(name)>180 or any(ord(c)<32 or ord(c)==127 for c in name):
        raise ValueError("녹화 이름은 줄바꿈 없이 1~180자로 입력하세요.")
    with LOCK:
        row,folder,meta=_find(key)
        if row["status"] in ACTIVE:
            raise ValueError("녹화와 파일 저장이 끝난 뒤 이름을 변경하세요.")
        meta.update(name=name,name_updated_at=time.time())
        path=next(folder/n for n in ("manifest.json","run.json","meta.json") if (folder/n).is_file())
        temp=path.with_name(path.name+"."+uuid.uuid4().hex+".tmp")
        temp.write_text(json.dumps(meta,ensure_ascii=False,indent=2)+"\n");temp.replace(path)
    return detail(key)


def label(key,success,note=''):
    if success is not None and type(success) is not bool:raise ValueError('작업 결과는 성공·실패·미분류 중 하나입니다.')
    if not isinstance(note,str) or len(note)>1000:raise ValueError('메모는 1000자 이내로 입력하세요.')
    with LOCK:
        row,folder,meta=_find(key)
        if row['status'] in ACTIVE:raise ValueError('녹화와 파일 검사가 끝난 뒤 작업 결과를 표시하세요.')
        meta.update(success=success,task_label='success' if success is True else 'failure' if success is False else 'unlabeled',
                    task_note=note,task_label_updated_at=time.time())
        path=next(folder/n for n in ('manifest.json','run.json','meta.json') if (folder/n).is_file())
        temp=path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
        temp.write_text(json.dumps(meta,ensure_ascii=False,indent=2)+'\n');temp.replace(path)
    return detail(key)


def video_path(key,index):
    _,folder,meta=_find(key)
    videos=_videos(folder,meta)
    if index<0 or index>=len(videos):
        raise ValueError("이 기록의 영상이 없습니다.")
    return videos[index][1]


def _deletion_state(key):
    row,folder,meta=_find(key)
    if row['status'] in ACTIVE:
        raise ValueError('녹화와 파일 저장이 끝난 뒤 삭제하세요.')
    roots=[Path(datadir.path('retargeting','runs')).resolve(),Path(datadir.path('recordings')).resolve()]
    if folder.is_symlink() or folder.resolve().parent not in roots:
        raise ValueError('녹화 기록 폴더의 경로를 확인할 수 없습니다.')
    inventory=[]
    for parent,dirs,files in os.walk(folder,followlinks=False):
        dirs[:]=[name for name in dirs if not (Path(parent)/name).is_symlink()]
        for name in sorted(files):
            path=Path(parent)/name
            stat=path.lstat()
            inventory.append((str(path.relative_to(folder)),stat.st_size,stat.st_mtime_ns,path.is_symlink()))
    inventory.sort()
    revision=hashlib.sha256(json.dumps([meta,inventory],sort_keys=True).encode()).hexdigest()
    return row,folder,revision,sum(item[1] for item in inventory if not item[3])


def deletion_preview(key):
    with LOCK:
        row,folder,revision,size=_deletion_state(key)
        return dict(id=key,name=row['name'],bytes=size,revision=revision,recovery_limit=RECOVERY_LIMIT)


def _trim_recovery(trash):
    entries=[]
    for folder in trash.iterdir():
        if folder.is_symlink() or not folder.is_dir() or len(folder.name)!=32:
            continue
        try:
            int(folder.name,16)
            receipt=folder/'RESTORE.json'
            recording=folder/'recording'
            if receipt.is_symlink() or recording.is_symlink() or not recording.is_dir():
                continue
            meta=json.loads(receipt.read_text())
            order=int(meta.get('deleted_at_ns',float(meta['deleted_at'])*1e9))
            entries.append((order,folder.name,folder))
        except (OSError,ValueError,TypeError,KeyError,OverflowError):
            continue
    entries.sort()
    expired=[]
    for _,name,folder in entries[:-RECOVERY_LIMIT]:
        shutil.rmtree(folder)
        expired.append(name)
    return expired


def delete(key,revision):
    """Move only this completed history folder to recoverable local storage."""
    with LOCK:
        row,folder,current,size=_deletion_state(key)
        if not isinstance(revision,str) or revision!=current:
            raise ValueError('녹화 기록이 변경됐습니다. 목록에서 다시 삭제하세요.')
        root=Path(datadir.path()).resolve()
        trash=root/'.trash'/'replay-recordings'
        if not trash.resolve().is_relative_to(root):
            raise ValueError('복구 보관 폴더의 경로가 올바르지 않습니다.')
        container=trash/uuid.uuid4().hex
        container.mkdir(parents=True)
        receipt=dict(id=key,name=row['name'],original_path=str(folder),deleted_at=time.time(),
                     deleted_at_ns=time.time_ns(),bytes=size)
        (container/'RESTORE.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n')
        folder.rename(container/'recording')
        expired=_trim_recovery(trash)
        return dict(ok=True,id=key,name=row['name'],bytes=size,recoverable=True,
                    recovery_limit=RECOVERY_LIMIT,expired_count=len(expired))
