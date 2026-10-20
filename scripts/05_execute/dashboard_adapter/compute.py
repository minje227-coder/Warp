"""Connect the dashboard queue API/CLI to Warp's offline standard runner."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import datadir
from tabs.retargeting.warp_backend import module
backend = module('compute')
backend.DATA = Path(datadir.path('retargeting'))
backend.CATALOG = backend.DATA/'catalog.json'
backend.JOBS = backend.DATA/'compute'
if __name__ == '__main__':
    backend.main()
else:
    sys.modules[__name__] = backend
