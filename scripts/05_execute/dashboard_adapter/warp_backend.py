"""Load Warp's backend; the dashboard owns no execution implementation."""
import importlib
import importlib.util
import os
from pathlib import Path
import sys

ROOT = Path(os.environ.get('WARP_EXECUTION_ROOT') or
            (str(Path(os.environ['WARP_ROBOT_EXECUTOR']).parent) if os.environ.get('WARP_ROBOT_EXECUTOR') else
             str(Path(__file__).resolve().parents[4]/'Warp/scripts/05_execute'))).resolve()
NAME = 'warp_kuavo_execution'


def module(name):
    if NAME not in sys.modules:
        source = ROOT/'kuavo_execution/__init__.py'
        if not source.is_file():
            raise ValueError('Warp 실행 backend가 없습니다: '+str(source))
        spec = importlib.util.spec_from_file_location(NAME,source,submodule_search_locations=[str(source.parent)])
        backend = importlib.util.module_from_spec(spec)
        sys.modules[NAME] = backend
        try: spec.loader.exec_module(backend)
        except Exception:
            sys.modules.pop(NAME,None)
            raise
    elif Path(sys.modules[NAME].__file__).resolve().parent != ROOT/'kuavo_execution':
        raise ValueError('이미 연결한 Warp backend 경로가 다릅니다. 대시보드를 재시작하세요.')
    return importlib.import_module(NAME+'.'+name)
