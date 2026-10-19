#!/usr/bin/env python3
"""Receive sealed raw episodes; never resample or run perception on upload."""
import argparse
import asyncio
import hashlib
import logging
import os
from pathlib import Path
import re
import shutil
import tempfile
import time
from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse
from raw_episode import import_raw_episode, saved_receipt
from upload_compression import ZstdBlocks

MAX_UPLOAD_BYTES=20*1024**3


def create_app(out_root):
    out_root=Path(out_root).expanduser().resolve()
    scratch=out_root/'.raw_incoming'
    scratch.mkdir(parents=True,exist_ok=True)
    app=FastAPI()
    @app.get('/health')
    def health(): return {'state':'ready','capture_protocol':'warp_raw_v3','upload_stream':'zip-zstd-block-v1'}
    @app.get('/upload_raw/receipt')
    async def receipt(request:Request):
        try:
            result=await asyncio.to_thread(saved_receipt,out_root,request.query_params.get('upload_id',''),request.query_params.get('manifest_sha256',''))
            return result if result is not None else JSONResponse({'error':'Not received'},status_code=404)
        except Exception as error:
            return JSONResponse({'error':str(error)},status_code=409)
    @app.post('/upload_raw')
    async def upload(request:Request):
        episode_id=request.query_params.get('upload_id','')
        checksum=request.headers.get('x-warp-sha256','')
        streaming=request.headers.get('x-warp-stream','')=='zip-zstd-block-v1'
        manifest_checksum=request.headers.get('x-warp-manifest-sha256','')
        if not re.fullmatch('[a-f0-9]{32}',episode_id) or not re.fullmatch('[a-f0-9]{64}',manifest_checksum if streaming else checksum):
            return JSONResponse({'error':'Invalid identity/checksum'},status_code=400)
        temporary=None
        try:
            started=time.monotonic()
            length=int(request.headers.get('content-length','0'))
            source_bytes=int(request.headers.get('x-warp-source-bytes','0')) if streaming else None
            if streaming:
                if not 0 < source_bytes <= MAX_UPLOAD_BYTES or not 0 <= length <= MAX_UPLOAD_BYTES: raise ValueError('Invalid source/upload length')
                required=source_bytes*2
            else:
                if length<=0 or length>MAX_UPLOAD_BYTES: raise ValueError('Invalid upload length')
                required=length*2
            if shutil.disk_usage(scratch).free<required+512*1024**2: raise OSError('PC storage low')
            h=hashlib.sha256();wire_hash=hashlib.sha256();size=0;archive_size=0
            decoder=ZstdBlocks() if streaming else None
            with tempfile.NamedTemporaryFile(dir=scratch,suffix='.partial',delete=False) as f:
                temporary=Path(f.name)
                async for chunk in request.stream():
                    size+=len(chunk)
                    if size>MAX_UPLOAD_BYTES or (length and size>length): raise ValueError('Upload exceeds declared length')
                    if shutil.disk_usage(scratch).free<512*1024**2: raise OSError('PC storage low during upload')
                    wire_hash.update(chunk)
                    def write_chunk(data):
                        nonlocal archive_size
                        for raw in decoder.feed(data) if decoder is not None else (data,):
                            archive_size+=len(raw)
                            if archive_size>MAX_UPLOAD_BYTES+MAX_UPLOAD_BYTES//100+1024**2: raise ValueError('Expanded ZIP exceeds limit')
                            h.update(raw);f.write(raw)
                    await asyncio.to_thread(write_chunk,chunk)
                if decoder is not None:decoder.finish()
                if (length and size!=length) or (not streaming and h.hexdigest()!=checksum): raise ValueError('Upload length/checksum mismatch')
                await asyncio.to_thread(f.flush)
                await asyncio.to_thread(os.fsync,f.fileno())
            received=time.monotonic()
            result=await asyncio.to_thread(import_raw_episode,temporary,out_root,episode_id,h.hexdigest(),
                manifest_checksum=manifest_checksum if streaming else None,source_bytes=source_bytes,
                transport={'encoding':'zip-zstd-block-v1','wire_sha256':wire_hash.hexdigest(),'wire_bytes':size} if streaming else None)
            logging.warning('Raw upload saved id=%s wire_bytes=%d source_bytes=%s receive_s=%.3f verify_s=%.3f stream=%s',episode_id,size,source_bytes,received-started,time.monotonic()-received,streaming)
            return result
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
