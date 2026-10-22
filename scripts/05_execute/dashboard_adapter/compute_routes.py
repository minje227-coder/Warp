"""Offline computation API: distinct from all robot commands."""
from fastapi import APIRouter, HTTPException, Request
from starlette.concurrency import run_in_threadpool
from core.common import PAGE_TOKEN
from tabs.retargeting import compute
from contextlib import contextmanager

router=APIRouter()


@contextmanager
def deletion_guard():
    from tabs.retargeting.warp_backend import module
    session=module('session')
    with session.LOCK:
        if session.CURRENT is not None and session.CURRENT.running():
            raise ValueError('로봇 정렬·재생·수집이 끝난 뒤 삭제하세요.')
        yield


async def deletion_request(key,request,confirm):
    p=await request.json()
    if not isinstance(p,dict) or p.get('token')!=PAGE_TOKEN:
        raise HTTPException(status_code=403,detail='페이지를 새로고침하세요.')
    from tabs.retargeting.warp_backend import module
    deletion=module('episode_delete')
    try:
        if not confirm:
            return await run_in_threadpool(deletion.preview,compute,key,deletion_guard)
        result=await run_in_threadpool(deletion.delete,compute,key,p.get('ticket'),p.get('confirmed'),deletion_guard)
        from tabs.retargeting import retargeting as rt
        with rt.LOCK:
            rt.DETAILS.pop(key,None)
            for k in list(rt.SELECTIONS):
                if k[1]==key:rt.SELECTIONS.pop(k,None)
            for k,v in list(rt.RANGE_DETAILS.items()):
                if v.get('id')==key:rt.RANGE_DETAILS.pop(k,None)
        return result
    except (ValueError,KeyError,OSError) as exc:
        raise HTTPException(status_code=409,detail=str(exc)) from exc


@router.post('/episode/{key}/delete-preview')
async def delete_preview(key:str,request:Request):
    return await deletion_request(key,request,False)


@router.post('/episode/{key}/delete')
async def delete_episode(key:str,request:Request):
    return await deletion_request(key,request,True)


@router.get('/compute')
async def status():
    return await run_in_threadpool(compute.status)


@router.get('/compute/calibration/{key}')
async def calibration(key:str):
    try:
        return await run_in_threadpool(compute.capture_calibration,key)
    except (ValueError,KeyError,OSError) as exc:
        raise HTTPException(status_code=409,detail=str(exc)) from exc


@router.post('/compute')
async def submit(request:Request):
    p=await request.json()
    if not isinstance(p,dict) or p.get('token')!=PAGE_TOKEN:
        raise HTTPException(status_code=403,detail='페이지를 새로고침하세요.')
    try:
        operation=p.get('op','enqueue')
        if operation not in ('enqueue','cancel'):raise ValueError('모르는 계산 명령입니다.')
        if operation == 'cancel':
            return await run_in_threadpool(compute.cancel,p.get('episodes'))
        return await run_in_threadpool(compute.enqueue,p.get('episodes'),p.get('tcp_lowering_cm',3.))
    except (ValueError,KeyError,OSError) as exc:
        raise HTTPException(status_code=409,detail=str(exc)) from exc


@router.get('/episode/{key}/validation')
async def validation(key:str):
    from tabs.retargeting import validation as audit
    try:
        return await run_in_threadpool(audit.detail,key)
    except (ValueError,KeyError,OSError) as exc:
        raise HTTPException(status_code=409,detail=str(exc)) from exc
