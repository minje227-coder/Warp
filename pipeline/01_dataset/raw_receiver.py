#!/usr/bin/env python3
"""Receive sealed raw episodes; never resample or run perception on upload."""
import argparse
import asyncio
import hashlib
import os
from pathlib import Path
import re
import shutil
import tempfile
from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse
from raw_episode import import_raw_episode

MAX_UPLOAD_BYTES=20*1024**3


def create_app(out_root):
    out_root=Path(out_root).expanduser().resolve()
    scratch=out_root/'.raw_incoming'
    scratch.mkdir(parents=True,exist_ok=True)
    app=FastAPI()
    @app.get('/health')
    def health(): return {'state':'ready','capture_protocol':'warp_raw_v3'}
    @app.post('/upload_raw')
    async def upload(request:Request):
        episode_id=request.query_params.get('upload_id','')
        checksum=request.headers.get('x-warp-sha256','')
        if not re.fullmatch('[a-f0-9]{32}',episode_id) or not re.fullmatch('[a-f0-9]{64}',checksum):
            return JSONResponse({'error':'Invalid identity/checksum'},status_code=400)
        temporary=None
        try:
            length=int(request.headers.get('content-length','0'))
            if length<=0 or length>MAX_UPLOAD_BYTES: raise ValueError('Invalid upload length')
            if shutil.disk_usage(scratch).free<length*2+512*1024**2: raise OSError('PC storage low')
            h=hashlib.sha256();size=0
            with tempfile.NamedTemporaryFile(dir=scratch,suffix='.partial',delete=False) as f:
                temporary=Path(f.name)
                async for chunk in request.stream():
                    size+=len(chunk)
                    if size>length: raise ValueError('Upload exceeds declared length')
                    h.update(chunk)
                    await asyncio.to_thread(f.write,chunk)
                if size!=length or h.hexdigest()!=checksum: raise ValueError('Upload length/checksum mismatch')
                await asyncio.to_thread(f.flush)
                await asyncio.to_thread(os.fsync,f.fileno())
            receipt=await asyncio.to_thread(import_raw_episode,temporary,out_root,episode_id,checksum)
            return receipt
        except Exception as e:
            return JSONResponse({'error':str(e)},status_code=422)
        finally:
            if temporary is not None: temporary.unlink(missing_ok=True)
    return app


def main():
    import uvicorn
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out-root',type=Path,default=Path.home()/'Warp_data')
    p.add_argument('--host',default='0.0.0.0');p.add_argument('--port',type=int,default=8080)
    a=p.parse_args();uvicorn.run(create_app(a.out_root),host=a.host,port=a.port)

if __name__=='__main__': main()
