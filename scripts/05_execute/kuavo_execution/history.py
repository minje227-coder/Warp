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
