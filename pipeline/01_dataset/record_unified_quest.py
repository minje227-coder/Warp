#!/usr/bin/env python3
"""Record same-timestamp RGB/skeleton/AprilTag bundles from the existing XRT app."""
import argparse
import asyncio
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import time
from unified_capture import BundleWriter, Reassembler
from validate_csv import inspect_csv

def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session')
    p.add_argument('--out-root', type=Path, default=Path(__file__).resolve().parents[2] / 'local_data/raw_human')
    p.add_argument('--host', default='0.0.0.0'); p.add_argument('--port', type=int, default=8080)
    p.add_argument('--duration', type=float, default=0); p.add_argument('--wait-seconds', type=float, default=90)
    p.add_argument('--min-frames', type=int, default=30); p.add_argument('--check', action='store_true')
    p.add_argument('--rgb-fps', type=float, default=10)
    p.add_argument('--tag-size-m', type=float, default=0, help='Actual printed tag width in meters; 0 keeps existing app setting')
    p.add_argument('--keep-overlays', action='store_true')
    args = p.parse_args(argv)
    if args.session and not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}',args.session):
        p.error('Invalid session ID')
    if not 0 < args.port < 65536 or args.duration < 0 or args.wait_seconds <= 0 or args.min_frames < 1:
        p.error('Invalid port/duration/wait/min-frames')
    if not 1 <= args.rgb_fps <= 30 or not 0 <= args.tag_size_m <= 10:
        p.error('rgb-fps must be 1-30; tag-size-m must be 0-10')
    return args

async def capture(args):
    from aiortc import RTCPeerConnection, RTCSessionDescription
    from fastapi import FastAPI, Request
    from fastapi.responses import JSONResponse
    import uvicorn
    import av
    if args.check:
        print('[OK] Unified receiver dependencies. Updated XRT APK/hardware NOT tested.')
        return 0
    session = args.session or datetime.now(timezone.utc).strftime('take_%Y%m%dT%H%M%SZ')
    folder = args.out_root.expanduser().resolve() / session
    folder.mkdir(parents=True, exist_ok=False)
    writer = BundleWriter(folder)
    assembler = Reassembler()
    app = FastAPI()
    peers = set()
    channels = []
    error = None
    start = time.monotonic()
    first = last = None
    requested_at = None
    stopping = False
    stop_at = None
    final_metadata = None
    server = uvicorn.Server(uvicorn.Config(app, host=args.host, port=args.port, log_level='warning'))

    def state(value):
        payload = json.dumps({'type':'state','state':value})
        for channel in channels:
            if channel.readyState == 'open': channel.send(payload)

    async def finalize():
        nonlocal final_metadata, error
        writer.close()
        if assembler.pending: error = error or 'Incomplete RGB bundle at stop'
        summary = inspect_csv(folder / 'body.csv', min_frames=args.min_frames, strict_sew=True)
        video = None
        if writer.frames >= args.min_frames and not error and summary['ok']:
            try: video = await asyncio.to_thread(writer.make_video)
            except Exception as exc: error = 'RGB video verification failed: ' + str(exc)
        success = not error and summary['ok'] and writer.frames >= args.min_frames
        final_metadata = {'status':'complete' if success else 'failed','error':error,
            'capture_protocol':'warp_capture_v1','rgb_required':True,
            'streams_captured':['body_pose_bones','rgb','apriltag'],
            'clock':'quest_utc_ticks_100ns','timestamp_owner':'camera SDK capture timestamp',
            'skeleton_time_source':'provider_query_utc','alignment':'interpolated_between_provider_samples',
            'max_bracket_distance_ms':50,'shared_frames':writer.frames,
            'tag_detections':writer.tag_detections,'hardware_synchronized':False,
            'rgb':{'frames':writer.frames,'video':video}, 'control':'Quest volume-up only',
            'tag_size_note':'Actual printed tag size must match server/app setting for metric pose.'}
        (folder/'metadata.json').write_text(json.dumps(final_metadata,indent=2)+'\n')
        state('saved' if success else 'error')
        await asyncio.sleep(0.4) # Deliver final headset status before closing SCTP.
        server.should_exit = True

    @app.post('/offer')
    async def offer(request: Request):
        nonlocal error, first, last
        if peers or writer.frames:
            return JSONResponse({'error':'Only one headset connection per episode'}, status_code=409)
        params = await request.json()
        peer = RTCPeerConnection(); peers.add(peer)
        @peer.on('datachannel')
        def on_channel(channel):
            if channel.label != 'warp_capture': return
            channels.append(channel)
            def arm():
                if channel.readyState == 'open': channel.send(json.dumps({'type':'config',
                    'rgb_fps':args.rgb_fps,'tag_size_m':args.tag_size_m,'clean_view':not args.keep_overlays}))
            channel.on('open', arm)
            arm()
            @channel.on('message')
            def receive(packet):
                nonlocal error, first, last, requested_at, stopping, stop_at
                if error: return
                try:
                    # Unity RTCDataChannel.Send(byte[]) produces binary SCTP messages.
                    if packet in (b'WARP_REQUEST_START', b'WARP_REQUEST_STOP'):
                        packet = packet.decode('ascii')
                    if packet == 'WARP_REQUEST_START':
                        if requested_at is None and not stopping:
                            requested_at = time.monotonic(); state('recording')
                        return
                    if packet == 'WARP_REQUEST_STOP':
                        if requested_at is not None and not stopping:
                            stopping = True; stop_at = time.monotonic(); state('saving')
                        return
                    if requested_at is None:
                        raise ValueError('Capture data arrived before volume-up start')
                    bundle = assembler.feed(packet)
                    if bundle is not None:
                        writer.write(*bundle)
                        last = time.monotonic()
                        if first is None:
                            first = last
                            print('RGB + aligned Skeleton + AprilTag: first shared timestamp received.', flush=True)
                except Exception as exc:
                    error = f'{type(exc).__name__}: {exc}'; state('error'); server.should_exit = True
        try:
            await peer.setRemoteDescription(RTCSessionDescription(**params))
            await peer.setLocalDescription(await peer.createAnswer())
            return {'type':peer.localDescription.type,'sdp':peer.localDescription.sdp}
        except Exception as exc:
            await peer.close(); peers.discard(peer)
            return JSONResponse({'error':str(exc)}, status_code=400)

    async def monitor():
        nonlocal error, stopping, stop_at
        report = time.monotonic()
        while not server.should_exit:
            now = time.monotonic()
            if not channels and now - start > args.wait_seconds:
                error = 'No unified capture bundle. The existing APK must be rebuilt with RGB/shared timestamps.'
                server.should_exit = True
            elif requested_at is not None and first is None and now - requested_at > 5:
                error = 'No valid same-timestamp bundle after volume-up start'; state('error'); server.should_exit = True
            elif last is not None and now - last > 2 and not stopping:
                error = 'Unified RGB/skeleton/AprilTag stream stalled'; server.should_exit = True
            elif first is not None and args.duration and now - first >= args.duration:
                if not stopping:
                    stopping = True; stop_at = now; state('saving')
            if stopping and now - stop_at > 0.5:
                await finalize(); return
            if now - report >= 5:
                print(f'Shared frames={writer.frames}; detected tags={writer.tag_detections}', flush=True)
                report = now
            await asyncio.sleep(0.05)

    print('Episode:',folder, flush=True)
    print(f'Existing XRT app → http://PC-IP:{args.port}/offer; volume-up starts/stops all three.', flush=True)
    task = asyncio.create_task(monitor())
    try:
        await server.serve()
    finally:
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)
        if final_metadata is None:
            if writer.frames and not error: error = 'Server stopped without Quest stop/finalization'
            await finalize()
        await asyncio.gather(*(peer.close() for peer in peers))
        writer.close()
    success = final_metadata['status'] == 'complete'
    print('[OK]' if success else '[FAIL]',final_metadata, flush=True)
    return 0 if success else 2

def main(argv=None):
    try: return asyncio.run(capture(parse_args(argv)))
    except (ImportError, FileExistsError, OSError) as exc:
        print('[FAIL]',exc); return 3

if __name__ == '__main__': raise SystemExit(main())
