"""Warp execution-package validation and caching.

No ROS, no motion on imports or GETs, and no rewriting adopted artifacts.
"""
import hashlib
import importlib.util
import os
from functools import lru_cache
from pathlib import Path



WORKER = Path(os.environ.get('WARP_ROBOT_EXECUTOR', str(Path(__file__).resolve().parents[1]/'kuavo_replay.py')))


@lru_cache(maxsize=1)
def driver():
    spec=importlib.util.spec_from_file_location('retargeting_replay_contract',WORKER)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def identity(entry,detail,allow_unverified=False):
    result=Path(entry['result'])
    paths=[result/'results/execution-trajectory-100hz.npz',result/'data/source-trajectory.jsonl',result/'REPORT.json']
    return (detail['fingerprint'],tuple((str(p),p.stat().st_mtime_ns,p.stat().st_size) for p in paths),
            tuple(detail['selection'][x]['execution_s'] for x in ('start','end')),allow_unverified)


def prepare(entry,detail,fresh=False,allow_unverified=False):
    if not detail.get('selection'):raise ValueError('영상에서 실행 구간을 선택하세요.')
    if not entry.get('result'):raise ValueError('계산된 실행 궤적이 없습니다.')
    key=identity(entry,detail,allow_unverified)
    if fresh:_prepare.cache_clear()
    return _prepare(key)


@lru_cache(maxsize=8)
def _prepare(key):
    return driver().build_package(key)

def reason(entry,detail,allow_unverified=False):
    try:prepare(entry,detail,allow_unverified=allow_unverified)
    except (ValueError,OSError,KeyError,TypeError,IndexError) as exc:return str(exc)
    return ''
