"""Compatibility import; implementation is owned by Warp."""
import sys
from tabs.retargeting.warp_backend import module
sys.modules[__name__] = module('first_state')
