"""Immutable raw-v3 ingestion. A transfer receipt is not a dataset-quality verdict."""
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import stat
import threading
import zipfile
from datetime import datetime
from zoneinfo import ZoneInfo

FILES = {'camera.mp4', 'camera_frames.jsonl', 'encoded_frames.jsonl',
         'tracking.jsonl', 'events.jsonl', 'calibration.json'}
USB_FILES = {'external_camera.mjpeg', 'external_camera_frames.jsonl', 'external_camera.json'}
USB_YUYV_FILES = {'external_camera.yuyv', 'external_camera_frames.jsonl', 'external_camera.json'}
LOCK = threading.Lock()
MAX_EXPANDED_BYTES = 20 * 1024**3


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + '.partial')
    with temporary.open('w') as f:
        json.dump(value, f, indent=2, allow_nan=False)
        f.write('\n')
        f.flush()
        os.fsync(f.fileno())
    temporary.replace(path)
    fd = os.open(path.parent, os.O_RDONLY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def import_raw_episode(source, out_root, upload_id, checksum):
    if not re.fullmatch('[a-f0-9]{32}', upload_id) or not re.fullmatch('[a-f0-9]{64}', checksum):
        raise ValueError('Invalid transfer identity')
    source, out_root = Path(source), Path(out_root).resolve()
    if digest(source) != checksum:
        raise ValueError('Upload checksum mismatch')
    with LOCK:
        receipts = out_root / '.raw_uploads'
        receipts.mkdir(parents=True, exist_ok=True)
        receipt_path = receipts / (upload_id + '.json')
        if receipt_path.exists():
            r = json.loads(receipt_path.read_text())
            if r['sha256'] != checksum:
                raise ValueError('Episode identity reused with different bytes')
            if not (Path(r['episode']) / 'transfer.json').is_file():
                raise ValueError('Confirmed PC episode missing; do not acknowledge')
            return r
        # Recover a crash after durable episode commit but before receipt commit.
        matches = []
        for archive_root in (out_root, out_root / 'raw'):
            for leaf in (upload_id, '*_' + upload_id):
                matches.extend(archive_root.glob('*/' + leaf + '/transfer.json'))
        # Restrict recovery to the supported old and new storage layouts.
        leaf_pattern = r'(?:[01][0-9]|2[0-3])-[0-5][0-9]-[0-5][0-9]_' + upload_id
        matches = [p for p in matches if p.parent.name == upload_id or re.fullmatch(leaf_pattern, p.parent.name)]
        if matches:
            if len(matches) != 1:
                raise ValueError('Duplicate raw episode identity')
            r = json.loads(matches[0].read_text())
            if r['sha256'] != checksum:
                raise ValueError('Episode identity collision')
            atomic_json(receipt_path, r)
            return r
        with zipfile.ZipFile(source) as archive:
            entries = archive.infolist()
            names = [e.filename for e in entries]
            if len(names) != len(set(names)) or not FILES | {'manifest.json'} <= set(names) or not set(names) <= FILES | USB_FILES | USB_YUYV_FILES | {'manifest.json'}:
                raise ValueError('Unexpected/duplicate archive entries')
            if sum(e.file_size for e in entries) > MAX_EXPANDED_BYTES:
                raise ValueError('Archive exceeds raw episode size limit')
            if any(e.is_dir() or stat.S_ISLNK(e.external_attr >> 16) for e in entries):
                raise ValueError('Directories/symlinks are not valid episode files')
            if archive.getinfo('manifest.json').file_size > 1024 * 1024:
                raise ValueError('Manifest too large')
            manifest = json.loads(archive.read('manifest.json'))
            if manifest.get('schema_version') != 3 or manifest.get('episode_id') != upload_id:
                raise ValueError('Raw manifest identity/schema mismatch')
            external = manifest.get('external_camera', {})
            if not isinstance(external, dict) or type(external.get('enabled', False)) is not bool:
                raise ValueError('Invalid external camera declaration')
            usb_files = USB_FILES
            if external.get('enabled') and 'external_camera.yuyv' in names:
                usb_files = USB_YUYV_FILES
            expected_files = FILES | usb_files if external.get('enabled') else FILES
            if set(manifest.get('files', {})) != expected_files or set(names) != expected_files | {'manifest.json'}:
                raise ValueError('Missing source provenance files')
            if (manifest.get('width'), manifest.get('height')) != (1280, 1280):
                raise ValueError('Unexpected camera resolution')
            if any(manifest.get(k, 0) <= 0 for k in ('sensor_frames','encoded_frames','tracking_samples')):
                raise ValueError('Missing camera/tracking observations')
            created_ms = manifest.get('created_unix_ms')
            if type(created_ms) is not int or created_ms <= 0:
                raise ValueError('Invalid recording start timestamp')
            try:
                started = datetime.fromtimestamp(created_ms / 1000, ZoneInfo('Asia/Seoul'))
            except (ValueError, OverflowError, OSError) as exc:
                raise ValueError('Invalid recording start timestamp') from exc
            # Display names carry capture time; UUIDs remain transfer/job identities.
            leaf = started.strftime('%H-%M-%S') + '_' + upload_id
            folder = out_root / started.strftime('%Y-%m-%d') / leaf
            if folder.exists():
                raise ValueError('Episode destination already exists; preserved')
            folder.parent.mkdir(parents=True, exist_ok=True)
            staging = folder.with_name(folder.name + '.partial')
            if staging.exists():
                shutil.rmtree(staging)  # Receiver-owned incomplete import only.
            staging.mkdir()
            try:
                for name in names:
                    target = staging / name
                    with archive.open(name) as src, target.open('wb') as dst:
                        shutil.copyfileobj(src, dst, length=1024*1024)
                        dst.flush()
                        os.fsync(dst.fileno())
                    if name != 'manifest.json':
                        expected = manifest['files'][name]
                        if target.stat().st_size != expected['bytes'] or digest(target) != expected['sha256']:
                            raise ValueError('Source checksum mismatch: ' + name)
                # Header verification only. Full decoding/QC is a separate task.
                import av
                with av.open(str(staging/'camera.mp4')) as video:
                    if len(video.streams.video) != 1:
                        raise ValueError('Expected one video stream')
                    stream = video.streams.video[0]
                    if (stream.width, stream.height) != (1280,1280):
                        raise ValueError('Actual video dimensions differ from manifest')
                if external.get('enabled'):
                    from external_camera import validate_external_camera
                    validate_external_camera(staging)
                r = {'state':'saved', 'upload_id':upload_id, 'sha256':checksum,
                     'episode':str(folder), 'processing_state':'pending',
                     'source_preserved':True, 'dataset_ready':False}
                atomic_json(staging/'transfer.json', r)
                staging.rename(folder)
                parent_fd = os.open(folder.parent, os.O_RDONLY)
                try: os.fsync(parent_fd)
                finally: os.close(parent_fd)
                atomic_json(receipt_path, r)
                return r
            except Exception:
                if staging.exists(): shutil.rmtree(staging)
                raise
