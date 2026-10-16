#!/usr/bin/env python3
"""Convert a recorded episode using its prepared input bundle and adopted standard."""
from pathlib import Path
import argparse,json,os,subprocess,sys,datetime

def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('episode',type=Path,help='Existing Warp_data/YYYY-MM-DD/take directory')
 p.add_argument('--runtime',type=Path,default=Path(os.environ.get('KUAVO_STANDARD_RUNTIME','/home/lab6/Warp-review/wilor-kuavo-standard')))
 p.add_argument('--python',default=os.environ.get('KUAVO_STANDARD_PYTHON','/home/lab6/Warp/.venv-retarget/bin/python'))
 p.add_argument('--prepared',type=Path,help='This recording\'s prepared.json; default: RUNTIME/prepared/TAKE/prepared.json')
 p.add_argument('--run-name',help='Unique output name; existing outputs are never overwritten')
 p.add_argument('--no-render',action='store_true',help='Explicitly skip four-window rendering')
 p.add_argument('--dry-run',action='store_true',help='Check paths/provenance and show the exact command')
 a=p.parse_args();episode=a.episode.expanduser().resolve()
 if not episode.is_dir():p.error('Episode directory does not exist: '+str(episode))
 runtime=a.runtime.expanduser().resolve();descriptor=(a.prepared or runtime/'prepared'/episode.name/'prepared.json').expanduser().resolve()
 if not descriptor.is_file():p.error('Prepared inputs are not available for this episode. Run WiLoR/Meta fusion, original timestamp gap handling, AprilTag/world estimation and lazy-base reference preparation first; then provide --prepared <prepared.json>. See docs/RETARGETING_STANDARD.md. The 13.35s benchmark starts after these stages.')
 spec=json.loads(descriptor.read_text());declared=Path(spec['episode']).expanduser().resolve()
 if declared!=episode:p.error('Prepared bundle belongs to a different episode: '+str(declared))
 def path(name):
  value=Path(spec[name]).expanduser();return (descriptor.parent/value).resolve() if not value.is_absolute() else value.resolve()
 files={k:path(k) for k in ['reference','world','reference_config']}
 for k,f in files.items():
  if not f.is_file():p.error(k+' file missing: '+str(f))
 if not (runtime/'standard.py').is_file():p.error('Standard runtime missing: '+str(runtime))
 name=a.run_name or episode.name+'-'+datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
 if Path(name).name!=name or name in ['.','..']:p.error('run-name must be a plain directory name')
 if (runtime/'results'/name).exists():p.error('Output already exists; choose a new run-name')
 command=[a.python,str(runtime/'standard.py'),'run',name,'--recording-take',episode.name]
 for k,f in files.items():command+=['--'+k.replace('_','-'),str(f)]
 if not a.no_render:
  if not spec.get('display_source'):p.error('Four-window rendering needs this recording\'s display_source in prepared.json. Use --no-render only when intentionally skipping video.')
  display=path('display_source')
  if not display.is_dir():p.error('display_source directory missing: '+str(display))
  if not (display/'results'/episode.name).is_dir():p.error('display_source does not contain this recording')
  command+=['--display-source',str(display)]
 print(json.dumps(dict(episode=str(episode),prepared=str(descriptor),output=str(runtime/'results'/name),four_window_rendering=not a.no_render,command=command),ensure_ascii=False,indent=2),flush=True)
 if a.dry_run:return 0
 return subprocess.run(command,env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MUJOCO_GL='egl',PYTHONDONTWRITEBYTECODE='1')).returncode
if __name__=='__main__':raise SystemExit(main())
