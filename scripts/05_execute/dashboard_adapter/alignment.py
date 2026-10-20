"""Bind dashboard resources to Warp; lifecycle and recording live in Warp."""
import sys
import datadir
from core import common
from tabs.tag.tag import RWH_SETUP
from tabs.retargeting.warp_backend import module


def command_sources(snapshot,within):
    from tabs.control.control import commanders
    return commanders(snapshot,within=within)


def other_busy():
    from tabs.poc2 import poc2
    return poc2.busy()


backend = module('session')
backend.configure(control=common,episode_source=module('catalog'),paths=datadir,
                  ros_setup=[RWH_SETUP],command_sources=command_sources,other_busy=other_busy)
sys.modules[__name__] = backend
