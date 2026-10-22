#!/usr/bin/env python3
"""Install only the Warp adapters into an existing tab-based dashboard.

No service restart, robot connection, recording or control is performed.
Dry-run is the default. Existing files/links are backed up before --apply.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import time
import uuid

ROOT=Path(__file__).resolve().parent


def plan(dashboard):
    dashboard=Path(dashboard).resolve()
    if not (dashboard/'webpage/core').is_dir() or not (dashboard/'webpage/tabs/retargeting/routes.py').is_file():
        raise ValueError('Requires an existing tab-based retargeting dashboard; older Gradio layouts are not migrated')
    files=[]
    for source in sorted((ROOT/'dashboard_adapter').iterdir()):
        if not source.is_file():continue
        target=dashboard/'webpage/tabs/retargeting'/source.name
        if not target.parent.resolve().is_relative_to(dashboard):raise ValueError('Adapter target leaves dashboard root')
        files.append((source,target))
    tests = ROOT/'dashboard_adapter/tests'
    for source in sorted(tests.glob('test_*.py')):
        target = dashboard/'webpage/tests'/source.name
        if not target.parent.is_dir() or not target.parent.resolve().is_relative_to(dashboard):
            raise ValueError('Missing dashboard tests directory')
        files.append((source,target))
    links=[]
    for name,source in [('retargeting_align.py',ROOT/'kuavo_replay.py'),('retargeting_agent.py',ROOT/'observe_replay.py')]:
        target=dashboard/'robot/agent'/name
        if not target.parent.is_dir() or not target.parent.resolve().is_relative_to(dashboard):
            raise ValueError('Missing dashboard robot/agent directory')
        links.append((source,target))
    return dashboard,files,links


def install(dashboard,apply=False,backup_root=None):
    dashboard,files,links=plan(dashboard)
    report=dict(dashboard=str(dashboard),warp_root=str(ROOT),applied=False,
                files=[str(t.relative_to(dashboard)) for _,t in files],
                links=[str(t.relative_to(dashboard)) for _,t in links])
    if not apply:return report
    backup=Path(backup_root or dashboard/'data/retargeting/backups')/('warp-adapter-'+time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:8])
    backup.mkdir(parents=True,exist_ok=False)
    previous=[]
    for source,target in files+links:
        exists=target.exists() or target.is_symlink()
        previous.append(dict(path=str(target.relative_to(dashboard)),existed=exists))
        if exists:
            saved=backup/target.relative_to(dashboard);saved.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(target,saved,follow_symlinks=False)
    (backup/'before.json').write_text(json.dumps(previous,indent=2)+'\n')
    for source,target in files:
        mode=(target.stat().st_mode&0o777) if target.exists() else 0o644
        temp=target.with_name(target.name+'.warp-'+uuid.uuid4().hex+'.tmp')
        shutil.copyfile(source,temp);temp.chmod(mode);os.replace(temp,target)
    for source,target in links:
        temp=target.with_name(target.name+'.warp-'+uuid.uuid4().hex+'.tmp')
        temp.symlink_to(os.path.relpath(source,target.parent));os.replace(temp,target)
    report.update(applied=True,backup=str(backup),sha256={str(t.relative_to(dashboard)):hashlib.sha256(t.read_bytes()).hexdigest() for _,t in files})
    (backup/'installed.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--dashboard-root',required=True)
    ap.add_argument('--apply',action='store_true')
    ap.add_argument('--backup-root')
    args=ap.parse_args()
    print(json.dumps(install(args.dashboard_root,args.apply,args.backup_root),indent=2))


if __name__=='__main__':main()
