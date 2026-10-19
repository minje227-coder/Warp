"""Separate serial offline processing worker; transfer receipts never await this job."""
import argparse
import fcntl
import os
import json
import logging
from pathlib import Path
import shutil
import time

from process_raw_episode import process
from raw_episode import atomic_json


def _run_once(root, processor=process, tag_size_m=None, offset_ns=None, retry_errors=False):
    root=Path(root).resolve();jobs=root/'.raw_processing';jobs.mkdir(parents=True,exist_ok=True)
    completed=0
    receipts = list(root.glob('*/*/transfer.json'))
    receipts += list((root/'raw').glob('*/*/transfer.json'))
    for receipt in sorted(receipts):
        source=receipt.parent;manifest=source/'manifest.json'
        # Receiver staging becomes visible before its final atomic rename.
        # Processing it races that rename and poisons the UUID job with an error.
        if source.name.endswith('.partial'):continue
        if not manifest.is_file(): continue
        try:
            metadata=json.loads(manifest.read_text());transfer=json.loads(receipt.read_text())
        except (ValueError,OSError): continue
        ident=metadata.get('episode_id')
        if metadata.get('schema_version')!=3 or transfer.get('state')!='saved' or not isinstance(ident,str) or len(ident)!=32 or any(c not in '0123456789abcdef' for c in ident): continue
        # Keep legacy archives/jobs readable; new episodes hold their own results.
        legacy = source.parent.parent == root/'raw'
        output = root/'derived'/source.parent.name/ident if legacy else source/'processed'
        partial = output.with_name(ident+'.partial' if legacy else 'processed.partial')
        job_path=jobs/(ident+'.json')
        previous=json.loads(job_path.read_text()) if job_path.exists() else {}
        if (output/'quality.json').is_file() and (output/'processing_manifest.json').is_file():
            if json.loads((output/'processing_manifest.json').read_text()).get('state')=='complete':
                if previous.get('state')!='complete':atomic_json(job_path,dict(previous,state='complete',output=str(output),dataset_ready=False))
                continue
        if output.exists():
            atomic_json(job_path,dict(previous,state='error',error='Existing final output is incomplete; preserved for inspection',dataset_ready=False))
            continue
        if previous.get('state')=='error' and not retry_errors: continue
        # Only this worker's exact derived staging directory is recoverable/deletable.
        if partial.exists():
            if partial.is_symlink():
                atomic_json(job_path,dict(previous,state='error',error='Refusing staging symlink',dataset_ready=False));continue
            shutil.rmtree(partial)
        output.parent.mkdir(parents=True,exist_ok=True)
        state={'episode_id':ident,'source':str(source),'output':str(output),'state':'running','attempts':previous.get('attempts',0)+1,'dataset_ready':False,'settings':{'tag_size_m':tag_size_m,'camera_to_utc_offset_ns':offset_ns}}
        atomic_json(job_path,state)
        try:
            qc=processor(source,partial,offset_ns=offset_ns,tag_size_m=tag_size_m)
            partial.rename(output)
            fd=os.open(output.parent,os.O_RDONLY)
            try: os.fsync(fd)
            finally: os.close(fd)
            atomic_json(job_path,dict(state,state='complete',quality_state=qc.get('state'),finished_unix_s=time.time()))
            completed+=1
        except Exception as error:
            # Retain failed derived artifacts for inspection until an explicit retry.
            atomic_json(job_path,dict(state,state='error',error=str(error),finished_unix_s=time.time()))
            logging.exception('Processing failed for %s',ident)
    return completed


def run_once(root, processor=process, tag_size_m=None, offset_ns=None, retry_errors=False):
    jobs=Path(root).resolve()/'.raw_processing';jobs.mkdir(parents=True,exist_ok=True)
    with (jobs/'.worker.lock').open('a') as lock:
        try: fcntl.flock(lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError: return 0
        try: return _run_once(root,processor,tag_size_m,offset_ns,retry_errors)
        finally: fcntl.flock(lock.fileno(),fcntl.LOCK_UN)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('out_root');p.add_argument('--tag-size-m',type=float)
    p.add_argument('--camera-to-utc-offset-ns',type=int);p.add_argument('--poll-seconds',type=float,default=1);p.add_argument('--once',action='store_true');p.add_argument('--retry-errors',action='store_true',help='Retry failed jobs once during startup')
    a=p.parse_args()
    if a.poll_seconds<=0:p.error('poll-seconds must be positive')
    if a.tag_size_m is not None and a.tag_size_m<=0:p.error('tag-size-m must be positive')
    logging.basicConfig(level=logging.INFO)
    retry=a.retry_errors
    while True:
        run_once(a.out_root,tag_size_m=a.tag_size_m,offset_ns=a.camera_to_utc_offset_ns,retry_errors=retry);retry=False
        if a.once:break
        time.sleep(a.poll_seconds)

if __name__=='__main__':main()
