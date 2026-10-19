"""Pure execution contracts; no ROS import, node, publisher or service."""
import importlib.util,math,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('warp_execution_contract',ROOT/'kuavo_replay.py')
w=importlib.util.module_from_spec(spec);spec.loader.exec_module(w)

class Contracts(unittest.TestCase):
    def test_torso_fk_matches_independent_complex_linkage(self):
        import cmath,random
        model=w.TorsoModel((Path(__file__).parent/'torso_fixture.urdf').read_text())
        rng=random.Random(19)
        for _ in range(1000):
            q=[rng.uniform(0.,1.44),rng.uniform(-2.8,0.),rng.uniform(-.15,2.9),rng.uniform(-3.,3.)]
            got=model.position(q);a=q[0];b=a+q[1];c=b+q[2]
            p=complex(.40635,.093)+complex(.18325,-.46521)*cmath.exp(1j*a)+complex(.12941,.48296)*cmath.exp(1j*b)+.25*cmath.exp(1j*c)
            for x,y in zip(got,[p.imag,p.real,c,q[3]]):self.assertAlmostEqual(x,y,places=12)

    def test_model_rejects_unknown_axes_before_creating_any_ros_interface(self):
        import xml.etree.ElementTree as ET
        root=ET.fromstring((Path(__file__).parent/'torso_fixture.urdf').read_text())
        next(j for j in root.findall('joint') if j.get('type')!='fixed').find('axis').set('xyz','1 0 0')
        xml=ET.tostring(root,encoding='unicode')
        with self.assertRaises(ValueError):w.TorsoModel(xml)

    def test_placeholder_feedback_cannot_satisfy_done_even_with_reached_states(self):
        self.assertIsNone(w.claw_feedback([-.6981317]*2,[0.,0.],[0.,0.],[2,2],['left_claw','right_claw'],10.,10.))
        self.assertIsNone(w.claw_feedback([0.,0.],[0.,0.],[0.,0.],[2,2],['left_claw','right_claw'],10.,10.))

    def test_claw_requires_matching_names_fresh_finite_feedback(self):
        def accept(**kw):
            args=dict(position=[20.,30.],velocity=[.01,.01],effort=[.1,.1],states=[2,3],
                      names=['left_claw','right_claw'],stamp=10.,ros_now=10.1)
            args.update(kw);return w.claw_feedback(**args)
        self.assertEqual(accept(),([20.,30.],[2,3]))
        self.assertIsNone(accept(stamp=8.))
        self.assertIsNone(accept(names=['right_claw','left_claw']))
        self.assertIsNone(accept(position=[float('nan'),20.]))
        with self.assertRaises(ValueError):accept(position=[200.,20.])
        with self.assertRaises(ValueError):accept(states=[-1,2])

    def test_clock_wait_targets_future_boundary_and_does_not_flush_old_ticks(self):
        for elapsed in [0.,.009,.015,.031,.049,.10,1.139]:
            delay=w.next_replay_wait(100.,100.+elapsed)
            self.assertGreater(delay,0)
            self.assertLessEqual(delay,.010001)
            self.assertGreaterEqual(elapsed+delay,(math.floor(elapsed/.01)+1)*.01-1e-8)
