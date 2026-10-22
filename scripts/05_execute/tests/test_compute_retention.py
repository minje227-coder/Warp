"""Exercise deletion boundaries and publication/failure behavior on fake storage."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

source = Path(__file__).with_name('retention.py')
if not source.is_file():
    source = Path(__file__).resolve().parents[1]/'kuavo_execution/retention.py'
spec = importlib.util.spec_from_file_location('retention', source)
retention = importlib.util.module_from_spec(spec)
spec.loader.exec_module(retention)


class RetentionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.data, self.standard = self.root/'data', self.root/'standard'
        self.key = 'take_012345'
        self.old_id, self.new_id = 'a'*32, 'b'*32
        self.old = self.folder(self.standard/'results'/(self.key+'-dashboard-'+self.old_id[:12]))
        self.new = self.folder(self.standard/'results'/(self.key+'-dashboard-'+self.new_id[:12]))
        self.old_job = self.folder(self.data/'compute'/self.old_id)
        self.new_job = self.folder(self.data/'compute'/self.new_id)
        for folder in (self.old_job, self.new_job):
            self.write(folder/'input/prepared.json', {'episode':'/raw/'+self.key})
            for name in ('MEDIA.json', 'replay-preview.mp4', 'worker.log'):
                (folder/name).write_text('old')
        self.catalog = {'episodes':[{'id':self.key, 'result':str(self.new), 'prepared':str(self.new_job/'input/prepared.json')}]}
        self.jobs = {self.key:{'id':self.new_id,'result':str(self.new),'status':'done','superseded_result':str(self.old)}}

    def folder(self, path):
        path.mkdir(parents=True, exist_ok=True)
        (path/'artifact').write_text('artifact')
        return path

    def write(self, path, value):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(value))

    def prune(self, **kwargs):
        return retention.prune(self.data, self.standard, self.catalog, self.jobs, **kwargs)

    def test_success_replaces_previous_results_and_all_job_artifacts(self):
        self.prune()
        self.assertFalse(self.old.exists())
        self.assertFalse(self.old_job.exists())
        self.assertTrue(self.new.exists())
        self.assertTrue((self.new_job/'worker.log').exists())
        self.assertEqual(self.prune(), [])

    def test_dry_run_does_not_mutate(self):
        self.assertIn(str(self.old), self.prune(dry_run=True))
        self.assertTrue(self.old.exists())

    def test_failed_or_active_attempt_keeps_selected_result_and_latest_diagnostics(self):
        self.catalog['episodes'][0]['result'] = str(self.old)
        for state in ('failed', 'running', 'queued', 'cancelled'):
            self.jobs[self.key]['status'] = state
            self.prune()
            self.assertTrue(self.old.exists())
            self.assertTrue(self.new.exists())
            self.assertTrue(self.new_job.exists())

    def test_keeps_old_prepared_inputs_but_removes_obsolete_preview_and_log(self):
        prepared = self.old_job/'input/prepared.json'
        self.catalog['episodes'][0]['prepared'] = str(prepared)
        self.prune()
        self.assertTrue(prepared.exists())
        self.assertFalse((self.old_job/'replay-preview.mp4').exists())
        self.assertFalse((self.old_job/'MEDIA.json').exists())
        self.assertFalse((self.old_job/'worker.log').exists())

    def test_current_other_episode_experiments_raw_and_symlink_target_survive(self):
        self.write(self.standard/'CURRENT.json', {'result':str(self.old)})
        experiment = self.folder(self.standard/'results'/(self.key+'-experiment'))
        other = self.folder(self.standard/'results'/'different-dashboard-123')
        raw = self.folder(self.root/'raw')
        link = self.standard/'results'/(self.key+'-dashboard-link')
        link.symlink_to(raw, target_is_directory=True)
        self.prune()
        for folder in (self.old, experiment, other, raw, link):
            self.assertTrue(folder.exists())

    def test_current_input_can_reference_an_old_job(self):
        path = self.old_job/'input/reference.jsonl'
        path.write_text('reference')
        self.write(self.new_job/'input/prepared.json', {'episode':'/raw/'+self.key,'reference':str(path)})
        self.prune()
        self.assertTrue(path.exists())

    def test_retained_result_symlink_dependency_survives(self):
        (self.new/'input-link').symlink_to(self.old/'artifact')
        self.prune()
        self.assertTrue((self.new/'input-link').is_file())

    def test_active_replay_reference_survives(self):
        self.write(self.data/'runs/replay/manifest.json', {'status':'running','artifact_sha256':{str(self.old/'artifact'):'hash'}})
        self.prune()
        self.assertTrue(self.old.exists())

    def test_old_media_only_removed_after_catalog_publication(self):
        media = self.folder(self.data/'media'/'previous')
        self.write(media/'MEDIA.json', {'kind':'replay_trajectory_preview','physical_replay_recording':False})
        self.jobs[self.key]['superseded_media'] = str(media)
        self.catalog['episodes'][0]['video_manifest'] = str(media/'MEDIA.json')
        self.prune()
        self.assertTrue(media.exists())
        del self.catalog['episodes'][0]['video_manifest']
        self.prune()
        self.assertFalse(media.exists())

    def test_outside_superseded_path_cannot_be_deleted(self):
        raw = self.folder(self.root/'raw')
        self.jobs[self.key]['superseded_result'] = str(raw)
        self.prune()
        self.assertTrue(raw.exists())

    def test_unreadable_current_stops_cleanup(self):
        (self.standard/'CURRENT.json').write_text('broken')
        with self.assertRaises(ValueError):
            self.prune()
        self.assertTrue(self.old.exists())


if __name__ == '__main__':
    unittest.main()
