"""The episode command must never silently reuse another recording's inputs."""
import json,subprocess,sys,tempfile,unittest
from pathlib import Path
CLI=Path(__file__).resolve().parents[3]/'scripts/02_convert/convert_episode_standard.py'
class EpisodeCLI(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.root=Path(self.tmp.name)
  self.episode=self.root/'episode with spaces';self.episode.mkdir();self.runtime=self.root/'runtime';self.runtime.mkdir();(self.runtime/'standard.py').write_text('raise RuntimeError("dry run must not execute")')
  self.provider=self.root/'provider';(self.provider/'results'/self.episode.name).mkdir(parents=True)
  self.spec=dict(episode=str(self.episode),display_source=str(self.provider))
  for key in ['reference','world','reference_config']:
   f=self.root/(key+'.json');f.write_text('{}');self.spec[key]=str(f)
  self.descriptor=self.root/'prepared.json'
 def call(self,*extra):
  self.descriptor.write_text(json.dumps(self.spec))
  return subprocess.run([sys.executable,str(CLI),str(self.episode),'--runtime',str(self.runtime),'--prepared',str(self.descriptor),'--run-name','review','--dry-run',*extra],text=True,capture_output=True)
 def test_correct_episode_uses_standard_and_four_views(self):
  r=self.call();self.assertEqual(r.returncode,0,r.stderr);v=json.loads(r.stdout);self.assertTrue(v['four_window_rendering']);self.assertIn('--recording-take',v['command']);self.assertEqual(v['command'][1],str(self.runtime/'standard.py'));self.assertIn(str(self.episode.name),v['command'])
 def test_wrong_recording_is_rejected(self):
  self.spec['episode']=str(self.root/'another episode');r=self.call();self.assertNotEqual(r.returncode,0);self.assertIn('different episode',r.stderr)
 def test_missing_prepared_input_is_rejected(self):
  self.spec['world']=str(self.root/'missing');r=self.call();self.assertNotEqual(r.returncode,0);self.assertIn('world file missing',r.stderr)
 def test_video_is_required_by_default(self):
  self.spec.pop('display_source');r=self.call();self.assertNotEqual(r.returncode,0);self.assertIn('Four-window rendering needs',r.stderr);self.assertEqual(self.call('--no-render').returncode,0)
 def test_existing_output_is_preserved(self):
  (self.runtime/'results/review').mkdir(parents=True);r=self.call();self.assertNotEqual(r.returncode,0);self.assertIn('Output already exists',r.stderr)

 def test_base_link_dry_run_prepares_without_writing(self):
  calibration=self.root/'calibration.json';calibration.write_text('{}')
  destination=self.root/'new-inputs'
  r=self.call('--no-render','--base-link-calibration',str(calibration),'--base-link-inputs',str(destination))
  self.assertEqual(r.returncode,0,r.stderr);v=json.loads(r.stdout)
  self.assertIn(str(calibration),v['preparation_command'])
  self.assertIn(str(destination/'reference.jsonl'),v['command'])
  self.assertFalse(destination.exists())
 def test_base_link_supports_matching_adopted_renderer(self):
  r=self.call('--base-link-calibration',str(self.root/'calibration.json'))
  self.assertEqual(r.returncode,0,r.stderr);v=json.loads(r.stdout)
  self.assertTrue(v['four_window_rendering']);self.assertIn('--display-source',v['command'])

if __name__=='__main__':unittest.main()
