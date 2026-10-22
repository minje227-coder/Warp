"""The dashboard imports Warp's implementation rather than retaining a copy."""
from pathlib import Path
import unittest
from tabs.retargeting import warp_backend


class BackendOwnershipTests(unittest.TestCase):
    def test_all_domain_modules_are_owned_by_warp_and_shared_with_adapters(self):
        import importlib
        mapping={'alignment':'session','collection':'capture','recordings':'history',
                 'first_state':'first_state','playback':'package','retargeting':'catalog',
                 'timeline':'timeline','validation':'validation','standard_rack':'standard_rack','compute':'compute'}
        for name,implementation in mapping.items():
            adapter=importlib.import_module('tabs.retargeting.'+name)
            self.assertIs(adapter,warp_backend.module(implementation))
            self.assertEqual(Path(adapter.__file__).resolve().parent,warp_backend.ROOT/'kuavo_execution')
        session=warp_backend.module('session')
        self.assertIs(session.episodes,warp_backend.module('catalog'))
        self.assertIs(session.collection,warp_backend.module('capture'))
        self.assertIs(session.playback,warp_backend.module('package'))
        self.assertEqual(session.episodes.OBSERVER,warp_backend.ROOT/'observe_replay.py')
