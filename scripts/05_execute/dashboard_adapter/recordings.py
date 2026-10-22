"""Bind dashboard storage to Warp's history and task-label API."""
import sys
import datadir
from tabs.retargeting.warp_backend import module
backend = module('history')
backend.datadir = datadir
sys.modules[__name__] = backend
