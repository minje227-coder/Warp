"""Retain current dashboard artifacts; never sweep raw data or experiments."""
import json
from pathlib import Path
import re
import shutil

JOB_ID = re.compile(r'[0-9a-f]{32}')


def _read(path):
    return json.loads(path.read_text())


def _paths(value):
    if isinstance(value, str) and value.startswith('/'):
        yield Path(value).resolve()
    elif isinstance(value, dict):
        for key, item in value.items():
            yield from _paths(key)
            yield from _paths(item)
    elif isinstance(value, list):
        for item in value:
            yield from _paths(item)


def _contains(folder, path):
    return path == folder or folder in path.parents


def plan(data, standard, catalog, jobs):
    """Plan only owned outputs, preserving catalog/input/CURRENT/active references."""
    data, standard = Path(data).resolve(), Path(standard).resolve()
    job_root, result_root = (data/'compute').resolve(), (standard/'results').resolve()
    entries = catalog['episodes']
    protected = set(_paths(catalog))
    candidates = set()
    media_candidates = set()
    partial_files = set()
    for key, row in jobs.items():
        # Latest attempt retains diagnostics even if a previous result stays selected.
        protected.add((job_root/row['id']).resolve())
        if row.get('result'):
            protected.add(Path(row['result']).resolve())
        if row.get('superseded_media'):
            folder = Path(row['superseded_media'])
            if folder.resolve().is_relative_to((data/'media').resolve()) and (folder/'MEDIA.json').is_file():
                meta = _read(folder/'MEDIA.json')
                if meta.get('kind') == 'replay_trajectory_preview' and meta.get('physical_replay_recording') is False:
                    media_candidates.add(folder)
        for name in ('superseded_result', 'source_result'):
            if row.get(name):
                candidates.add(Path(row[name]))
    current = standard/'CURRENT.json'
    if current.exists():
        protected.update(_paths(_read(current)))
    for manifest in (data/'runs').glob('*/manifest.json'):
        meta = _read(manifest)
        if meta.get('status') in ('starting', 'recording', 'running', 'saving'):
            protected.update(_paths(meta))
    # Prepared bundles can refer to files from older input preparation jobs.
    for entry in entries:
        for name in ('prepared', 'input_prepared'):
            if entry.get(name):
                descriptor = Path(entry[name])
                if descriptor.is_file():
                    for name, value in _read(descriptor).items():
                        if name in ('reference', 'reference_config', 'world', 'display_source') and isinstance(value, str):
                            path = Path(value)
                            protected.add((path if path.is_absolute() else descriptor.parent/path).resolve())
    for entry in entries:
        key = entry['id']
        if Path(key).name != key:
            raise ValueError('Invalid episode id')
        if result_root.is_dir():
            candidates.update(p for p in result_root.iterdir() if p.name.startswith(key+'-dashboard-'))
    if job_root.is_dir():
        for folder in job_root.iterdir():
            if not JOB_ID.fullmatch(folder.name) or folder.is_symlink() or not folder.is_dir():
                continue
            descriptor = folder/'JOB.json' if (folder/'JOB.json').is_file() else folder/'input/prepared.json'
            if descriptor.is_file():
                source = Path(_read(descriptor).get('episode', '')).name
                if source in {e['id'] for e in entries}:
                    candidates.add(folder)
                    partial_files.update(folder/name for name in ('MEDIA.json', 'replay-preview.mp4', 'worker.log'))
    # Resolve dependencies of every retained output, including non-dashboard names.
    outputs = candidates | media_candidates
    if result_root.is_dir():
        outputs.update(result_root.iterdir())
    while True:
        before = len(protected)
        for folder in outputs:
            if folder.is_dir() and not folder.is_symlink() and any(_contains(folder.resolve(), p) for p in protected):
                protected.update(p.resolve() for p in folder.rglob('*') if p.is_symlink())
        if len(protected) == before:
            break
    removable = []
    for folder in sorted(candidates | media_candidates):
        if (folder.parent not in (job_root, result_root) and folder not in media_candidates) or folder.is_symlink() or not folder.is_dir():
            continue
        resolved = folder.resolve()
        if any(_contains(resolved, p) or _contains(p, resolved) for p in protected):
            continue
        removable.append(folder)
    for path in sorted(partial_files):
        if path.is_symlink() or not path.is_file() or any(_contains(folder, path) for folder in removable):
            continue
        if not any(_contains(path.resolve(), p) or _contains(p, path.resolve()) for p in protected):
            removable.append(path)
    return removable


def prune(data, standard, catalog, jobs, dry_run=False):
    removed = []
    for folder in plan(data, standard, catalog, jobs):
        if not dry_run:
            if folder.is_dir():
                shutil.rmtree(folder)
            else:
                folder.unlink()
        removed.append(str(folder))
    return removed
