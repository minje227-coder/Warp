"""Retargeting tab HTTP interface. GETs never acquire robot control."""
from fastapi import APIRouter, HTTPException, Request
from fastapi.responses import FileResponse, JSONResponse
from starlette.concurrency import run_in_threadpool

from core.common import PAGE_TOKEN
from tabs.retargeting import retargeting as rt
from tabs.retargeting import alignment

router=APIRouter(prefix="/kuavo/retargeting")


async def invoke(fn,*args):
    try:
        return await run_in_threadpool(fn,*args)
    except (ValueError,OSError,KeyError) as exc:
        raise HTTPException(status_code=409,detail=str(exc)) from exc


@router.get("")
async def catalogue():
    return await invoke(rt.catalogue)


@router.get("/episode/{key}")
async def detail(key:str,sid:str=''):
    return await invoke(rt.selection_detail,key,sid[:100])


@router.get("/episode/{key}/state")
async def state(key:str,sid:str):
    return await invoke(rt.state,key,sid[:100])


@router.get("/episode/{key}/video")
async def video(key:str):
    path=await invoke(rt.video_path,key)
    return FileResponse(path,media_type="video/mp4",headers={"Cache-Control":"no-cache"})


@router.post("/episode/{key}/command")
async def command(key:str,request:Request):
    p=await request.json()
    # A stale page can still cancel alignment. Stop never starts a worker.
    if isinstance(p,dict) and p.get('op')=='align_stop':
        return await invoke(alignment.stop)
    if not isinstance(p,dict) or p.get("token")!=PAGE_TOKEN:
        raise HTTPException(status_code=403,detail="페이지를 새로고침하세요.")
    sid=p.get("sid")
    if not isinstance(sid,str) or not sid or len(sid)>100:
        raise HTTPException(status_code=400,detail="페이지 세션이 없습니다.")
    if p.get("op")=="confirm":
        return await invoke(rt.confirm,key,sid,p.get("fingerprint"),p.get("step"),p.get("checked"))
    if p.get('op')=='select_range':
        return await invoke(rt.select_range,key,sid,p.get('fingerprint'),p.get('start_video_s'),p.get('end_video_s'))
    if p.get('op')=='align_start':
        return await invoke(rt.align_start,key,sid,p.get('fingerprint'),p.get('stationary_confirmed') is True)
    if p.get('op')=='align_beat':
        return await invoke(alignment.beat,key,sid,p.get('fingerprint'))
    if p.get("op")=="replay":
        return await invoke(rt.replay,key,sid,p.get("fingerprint"),p.get("stationary_confirmed") is True)
    raise HTTPException(status_code=400,detail="모르는 명령입니다.")


# Output history is independent of source episode selection and robot control.
from tabs.retargeting import recordings as history


@router.get("/recordings")
async def recordings_list():
    return await invoke(history.listing)


@router.get("/recordings/{key}")
async def recording_detail(key:str):
    return await invoke(history.detail,key)


@router.post("/recordings/{key}/rename")
async def recording_rename(key:str,request:Request):
    p=await request.json()
    if not isinstance(p,dict) or p.get("token")!=PAGE_TOKEN:
        raise HTTPException(status_code=403,detail="페이지를 새로고침하세요.")
    return await invoke(history.rename,key,p.get("name"))


@router.post("/recordings/{key}/label")
async def recording_label(key:str,request:Request):
    p=await request.json()
    if not isinstance(p,dict) or p.get('token')!=PAGE_TOKEN:
        raise HTTPException(status_code=403,detail='페이지를 새로고침하세요.')
    return await invoke(history.label,key,p.get('success'),p.get('note',''))


@router.get("/recordings/{key}/video/{index}")
async def recording_video(key:str,index:int):
    path=await invoke(history.video_path,key,index)
    return FileResponse(path,media_type="video/mp4",headers={"Cache-Control":"no-cache"})

# Offline standard calculation queue.
@router.post('/recordings/{key}/delete-preview')
async def recording_delete_preview(key:str,request:Request):
    p=await request.json()
    if not isinstance(p,dict) or p.get('token')!=PAGE_TOKEN:
        raise HTTPException(status_code=403,detail='페이지를 새로고침하세요.')
    return await invoke(history.deletion_preview,key)


@router.post('/recordings/{key}/delete')
async def recording_delete(key:str,request:Request):
    p=await request.json()
    if not isinstance(p,dict) or p.get('token')!=PAGE_TOKEN:
        raise HTTPException(status_code=403,detail='페이지를 새로고침하세요.')
    def remove():
        with alignment.LOCK:
            if alignment.CURRENT is not None and alignment.CURRENT.running():
                raise ValueError('로봇 정렬·replay·녹화가 끝난 뒤 삭제하세요.')
            return history.delete(key,p.get('revision'))
    return await invoke(remove)


from tabs.retargeting.compute_routes import router as compute_router
router.include_router(compute_router)
