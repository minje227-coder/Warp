#!/usr/bin/env python3
"""Record same-timestamp RGB/skeleton/AprilTag bundles from the existing XRT app."""
import argparse
import asyncio
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import time
from unified_capture import BundleWriter, Reassembler, TagJoiner
from validate_csv import inspect_csv

def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session')
    p.add_argument('--out-root', type=Path, default=Path.home() / 'Warp_data')
    p.add_argument('--host', default='0.0.0.0'); p.add_argument('--port', type=int, default=8080)
    p.add_argument('--duration', type=float, default=0); p.add_argument('--wait-seconds', type=float, default=90)
    p.add_argument('--min-frames', type=int, default=30); p.add_argument('--check', action='store_true')
    p.add_argument('--max-episodes', type=int, default=0, help='0 records repeatedly; 1 exits after one take')
    p.add_argument('--rgb-fps', type=float, default=30)
    p.add_argument('--tag-size-m', type=float, default=0.04, help='Printed tag side length in meters (default 40 mm); 0 keeps app setting')
    p.add_argument('--keep-overlays', action='store_true')
    args = p.parse_args(argv)
    if args.session and not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}',args.session):
        p.error('Invalid session ID')
    if not 0 < args.port < 65536 or args.duration < 0 or args.wait_seconds <= 0 or args.min_frames < 1 or args.max_episodes < 0:
        p.error('Invalid port/duration/wait/min-frames')
    if not 1 <= args.rgb_fps <= 30 or not 0 <= args.tag_size_m <= 10:
        p.error('rgb-fps must be 1-30; tag-size-m must be 0-10')
    return args

def reserve_episode(out_root, day=None):
    """Atomically reserve DATE/N; concurrent recorders never reuse a take folder."""
    day = day or datetime.now().strftime('%Y-%m-%d')
    date_folder = out_root.expanduser().resolve() / day
    date_folder.mkdir(parents=True, exist_ok=True)
    number = max((int(p.name) for p in date_folder.iterdir() if p.name.isdigit()), default=0) + 1
    while True:
        folder = date_folder / str(number)
        try:
            folder.mkdir(exist_ok=False)
            return folder
        except FileExistsError:
            number += 1

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
    folder = None
    writer = None
    assembler = Reassembler()
    tag_assembler = Reassembler(magic=b'WRT1', metadata_only=True)
    tag_joiner = TagJoiner()
    write_queue = asyncio.Queue(maxsize=8)
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
    completed_episodes = 0
    server = uvicorn.Server(uvicorn.Config(app, host=args.host, port=args.port, log_level='warning'))

    def state(value):
        payload = json.dumps({'type':'state','state':value})
        for channel in channels:
            if channel.readyState == 'open': channel.send(payload)

    async def write_bundles():
        nonlocal error
        while True:
            bundle = await write_queue.get()
            try:
                if not error:
                    await asyncio.to_thread(writer.write, *bundle)
            except Exception as exc:
                error = f'{type(exc).__name__}: {exc}'
                state('error'); server.should_exit = True
            finally:
                write_queue.task_done()

    async def finalize():
        nonlocal final_metadata, error, completed_episodes, stopping, requested_at, first, last
        await write_queue.join()
        if writer is None: return
        await asyncio.to_thread(writer.close)
        if not folder.is_dir():
            error = f'Recording directory disappeared: {folder}'
            final_metadata = {'status':'failed','error':error}
            print('[FAIL]',final_metadata,flush=True)
            state('error'); await asyncio.sleep(0.4); server.should_exit = True
            return
        if assembler.pending or tag_assembler.pending or tag_joiner.pending:
            error = error or 'Incomplete RGB/AprilTag bundle at stop'
        summary = inspect_csv(folder / 'body.csv', min_frames=args.min_frames, strict_sew=True)
        video = None
        if writer.source_frames >= args.min_frames and not error and summary['ok']:
            try: video = await asyncio.to_thread(writer.make_video)
            except Exception as exc: error = 'RGB video verification failed: ' + str(exc)
        success = not error and summary['ok'] and writer.source_frames >= args.min_frames
        final_metadata = {'session':session, 'episode_date':folder.parent.name,
            'episode_number':int(folder.name), 'status':'complete' if success else 'failed','error':error,
            'capture_protocol':'warp_capture_v2' if writer.protocol_version == 2 else 'warp_capture_v1','rgb_required':True,
            'streams_captured':['body_pose_bones','rgb','apriltag'],
            'clock':'quest_utc_ticks_100ns','timestamp_owner':'camera SDK capture timestamp',
            'skeleton_time_source':writer.skeleton_time_source,'alignment':'interpolated_between_provider_samples',
            'max_bracket_distance_ms':50,'shared_frames':writer.frames,
            'tag_detections':writer.tag_detections,'hardware_synchronized':False,
            'recording_fps':args.rgb_fps,'timestep_s':1 / args.rgb_fps,
            'rgb_storage':'direct_mp4','resampling':'nearest_camera_slot_with_explicit_previous_bundle_hold',
            'source_frames':writer.source_frames,'repeated_frames':writer.repeated_frames,
            'source_fps': ((writer.source_frames - 1)*10_000_000/(writer.last_timestamp-writer.origin)
                           if writer.source_frames > 1 else 0),
            'coalesced_frames':writer.coalesced_frames,
            'rgb':{'frames':writer.frames,'video':video}, 'control':'Quest volume-up only',
            'tag_size_note':'Actual printed tag size must match server/app setting for metric pose.'}
        (folder/'metadata.json').write_text(json.dumps(final_metadata,indent=2)+'\n')
        completed_episodes += int(success)
        print('[OK]' if success else '[FAIL]', final_metadata, flush=True)
        stopping = False; requested_at = None; first = last = None
        state('saved' if success else 'error')
        if not success or (args.max_episodes and completed_episodes >= args.max_episodes):
            await asyncio.sleep(0.4) # Deliver final headset status before closing SCTP.
            server.should_exit = True

    @app.post('/upload')
    async def upload(request: Request):
        import hashlib
        import tempfile
        from local_episode import import_local_episode
        upload_id = request.query_params.get('upload_id', '')
        checksum = request.headers.get('x-warp-sha256', '')
        if not re.fullmatch(r'[a-f0-9]{32}', upload_id) or not re.fullmatch(r'[a-f0-9]{64}', checksum):
            return JSONResponse({'error':'Invalid upload identity/checksum'}, status_code=400)
        upload_root = args.out_root.expanduser().resolve() / '.uploads'
        upload_root.mkdir(parents=True, exist_ok=True)
        digest = hashlib.sha256()
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(dir=upload_root, suffix='.partial', delete=False) as file:
                temporary = Path(file.name)
                async for chunk in request.stream():
                    file.write(chunk); digest.update(chunk)
            if digest.hexdigest() != checksum:
                raise ValueError('Upload checksum mismatch')
            receipt = await asyncio.to_thread(import_local_episode, temporary, args.out_root,
                                              upload_id, checksum, args.min_frames)
            return receipt
        except Exception as exc:
            print('[UPLOAD FAIL]', str(exc), flush=True)
            return JSONResponse({'error':str(exc)}, status_code=422)
        finally:
            if temporary is not None: temporary.unlink(missing_ok=True)

    @app.post('/offer')
    async def offer(request: Request):
        nonlocal error, first, last
        if peers or (writer is not None and writer.frames):
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
                nonlocal error, first, last, requested_at, stopping, stop_at, writer, folder, assembler, final_metadata, session, tag_assembler, tag_joiner
                if error: return
                try:
                    # Unity RTCDataChannel.Send(byte[]) produces binary SCTP messages.
                    if packet in (b'WARP_REQUEST_START', b'WARP_REQUEST_STOP'):
                        packet = packet.decode('ascii')
                    if packet == b'WARP_CAPTURE_ERROR': raise ValueError('Quest capture processing backlog/error')
                    if packet == 'WARP_REQUEST_START':
                        if requested_at is None and not stopping:
                            if writer is None or final_metadata is not None:
                                folder = reserve_episode(args.out_root)
                                writer = BundleWriter(folder, fps=args.rgb_fps)
                                assembler = Reassembler()
                                tag_assembler = Reassembler(magic=b'WRT1', metadata_only=True)
                                tag_joiner = TagJoiner()
                                final_metadata = None
                                session = args.session or datetime.now(timezone.utc).strftime('take_%Y%m%dT%H%M%SZ')
                                print('Episode:', folder, flush=True)
                            requested_at = time.monotonic(); state('recording')
                        return
                    if packet == 'WARP_REQUEST_STOP':
                        if requested_at is not None and not stopping:
                            stopping = True; stop_at = time.monotonic(); state('saving')
                        return
                    if requested_at is None:
                        raise ValueError('Capture data arrived before volume-up start')
                    is_tag = isinstance(packet,bytes) and packet[:4] == b'WRT1'
                    bundle = (tag_assembler if is_tag else assembler).feed(packet)
                    if bundle is not None:
                        meta, jpeg = bundle
                        if is_tag: bundles = tag_joiner.feed(meta)
                        elif meta.get('version') == 2: bundles = tag_joiner.feed(meta,jpeg)
                        else: bundles = [bundle] # Preserve legacy capture protocol compatibility.
                        for ready in bundles:
                            write_queue.put_nowait(ready)
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
            try: tag_joiner.expire()
            except ValueError as exc:
                error = str(exc); state('error'); server.should_exit = True; return
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
                await finalize()
                if server.should_exit: return
            if now - report >= 5:
                print(f'Shared frames={writer.frames if writer else 0}; source frames={writer.source_frames if writer else 0}; repeated={writer.repeated_frames if writer else 0}; detected tags={writer.tag_detections if writer else 0}', flush=True)
                report = now
            await asyncio.sleep(0.05)

    print('Ready; episode folder is reserved on volume-up start.', flush=True)
    print(f'Existing XRT app → http://PC-IP:{args.port}/offer; volume-up starts/stops all three.', flush=True)
    write_task = asyncio.create_task(write_bundles())
    task = asyncio.create_task(monitor())
    try:
        await server.serve()
    finally:
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)
        if final_metadata is None and writer is not None:
            if writer.frames and not error: error = 'Server stopped without Quest stop/finalization'
            await finalize()
        write_task.cancel()
        await asyncio.gather(write_task, return_exceptions=True)
        await asyncio.gather(*(peer.close() for peer in peers))
        if writer is not None: writer.close()
    if final_metadata is None: return 0 if not error else 2
    success = final_metadata['status'] == 'complete'
    print('[OK]' if success else '[FAIL]',final_metadata, flush=True)
    return 0 if success else 2

def main(argv=None):
    try: return asyncio.run(capture(parse_args(argv)))
    except (ImportError, FileExistsError, OSError) as exc:
        print('[FAIL]',exc); return 3

if __name__ == '__main__': raise SystemExit(main())
