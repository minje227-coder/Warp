"""Connect the dashboard's transport and storage to Warp's application API."""
import sys
import datadir
from core import common
from tabs.tag.tag import RWH_SETUP
from tabs.retargeting.warp_backend import module
backend = module('catalog')
backend.configure(control=common,paths=datadir,setup=[RWH_SETUP])
sys.modules[__name__] = backend
# Bind execution ports as well when this API is used without the HTTP router.
from tabs.retargeting import alignment
