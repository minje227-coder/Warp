"""CLI compatibility launcher; implementation lives in Warp."""
import runpy
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from tabs.retargeting.warp_backend import ROOT
if __name__ == '__main__':
    runpy.run_path(str(ROOT/'kuavo_execution/compute_camera.py'),run_name='__main__')
