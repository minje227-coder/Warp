"""Import a sealed Quest-local WQL1 take, preserving source timing provenance."""
import json
import math
from pathlib import Path
import struct
import threading
from unified_capture import BundleWriter, validate_bundle, MAX_BYTES
from validate_episode import inspect_episode

_IMPORT_LOCK = threading.Lock()

def import_local_episode(source, out_root, upload_id, checksum, min_frames=30):
    from record_unified_quest import reserve_episode
    out_root = Path(out_root).expanduser().resolve()
    receipts = out_root / '.uploads'
    receipts.mkdir(parents=True, exist_ok=True)
    receipt_path = receipts / (upload_id + '.json')
    with _IMPORT_LOCK:
        if receipt_path.exists():
            receipt = json.loads(receipt_path.read_text())
            if receipt['sha256'] != checksum: raise ValueError('Upload ID reused with different content')
            if not Path(receipt['episode']).is_dir(): raise ValueError('Previously saved episode is missing')
            return receipt
        folder = reserve_episode(out_root)
        writer = None
        try:
            with Path(source).open('rb') as file:
                if file.read(4) != b'WQL1': raise ValueError('Unknown local capture format')
                header = file.read(8)
                if len(header) != 8: raise ValueError('Truncated local header')
                fps, = struct.unpack('<d', header)
                if not math.isfinite(fps) or not 1 <= fps <= 30: raise ValueError('Invalid local FPS')
                writer = BundleWriter(folder, fps=fps)
                previous_id = 0
                while True:
                    record = file.read(4)
                    if not record: break
                    if len(record) != 4: raise ValueError('Truncated local record')
                    length, = struct.unpack('<i', record)
                    if not 4 < length <= MAX_BYTES: raise ValueError('Invalid local record size')
                    payload = file.read(length)
                    if len(payload) != length: raise ValueError('Truncated local payload')
                    metadata_length, = struct.unpack_from('<i', payload)
                    if not 0 < metadata_length < length-4: raise ValueError('Invalid local metadata size')
                    meta = json.loads(payload[4:4+metadata_length])
                    jpeg = payload[4+metadata_length:]
                    validate_bundle(meta, jpeg)
                    if meta['frame_id'] <= previous_id: raise ValueError('Local source frame IDs regressed')
                    previous_id = meta['frame_id']
                    writer.write(meta,jpeg)
            writer.close()
            if writer.source_frames < min_frames: raise ValueError('Too few unique source frames')
            video = writer.make_video()
            metadata = {'status':'complete','error':None,'capture_protocol':'warp_capture_v2',
                'session':upload_id,'episode_date':folder.parent.name,'episode_number':int(folder.name),
                'clock':'quest_utc_ticks_100ns','hardware_synchronized':False,
                'rgb_required':True,'recording_fps':fps,'timestep_s':1/fps,
                'rgb_storage':'direct_mp4','shared_frames':writer.frames,'source_frames':writer.source_frames,
                'repeated_frames':writer.repeated_frames,'coalesced_frames':writer.coalesced_frames,
                'tag_detections':writer.tag_detections,'skeleton_time_source':writer.skeleton_time_source,
                'transport':'quest_local_then_http','upload_sha256':checksum,
                'rgb':{'frames':writer.frames,'video':video}}
            (folder/'metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
            result = inspect_episode(folder,min_frames=min_frames,strict_sew=True)
            if not result['ok']: raise ValueError('Episode verification failed: '+str(result['messages']))
            receipt = {'state':'saved','upload_id':upload_id,'sha256':checksum,
                       'episode':str(folder),'shared_frames':writer.frames,'source_frames':writer.source_frames}
            temporary_receipt = receipt_path.with_suffix('.tmp')
            temporary_receipt.write_text(json.dumps(receipt,indent=2)+'\n')
            temporary_receipt.replace(receipt_path)
            print('[LOCAL SAVED]',receipt,flush=True)
            return receipt
        except Exception as exc:
            if writer is not None: writer.close()
            (folder/'metadata.json').write_text(json.dumps({'status':'failed','error':str(exc)},indent=2)+'\n')
            raise
