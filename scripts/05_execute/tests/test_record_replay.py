import importlib.util
from pathlib import Path
import time
import unittest
spec=importlib.util.spec_from_file_location('record_replay',Path(__file__).resolve().parents[1]/'record_replay.py')
worker=importlib.util.module_from_spec(spec);spec.loader.exec_module(worker)
def good_stats():
    row=dict(count=100,hz=50.,first_s=10.,last_s=12.,max_gap_s=.05,bad_source_stamps=0,
        source_first_ns=10_000_000_000,source_last_ns=12_000_000_000)
    return {t:dict(row) for t in worker.REQUIRED_LIVE+worker.COMMAND_TOPICS}

class QualityTests(unittest.TestCase):
    def test_topic_union_keeps_team_topics_and_new_torso_commands(self):
        topics=worker.topics_for_run('a'*32,['/team_extra','/cmd_vel'])
        self.assertIn('/team_extra',topics);self.assertIn('/lb_leg_traj',topics)
        self.assertIn('/cmd_lb_torso_pose',topics);self.assertEqual(topics.count('/cmd_vel'),1)
        self.assertIn(worker.EVENT_PREFIX+'a'*32,topics)
        for bad in ['../run','a'*31,'A'*32]:
            with self.assertRaises(ValueError):worker.topics_for_run(bad)

    def check(self,rows,**kw):
        return worker.quality(rows,{t:1 for t in worker.LATCHED},10_000_000_000,12_000_000_000,**kw)

    def test_all_streams_and_commands_pass(self):self.assertTrue(self.check(good_stats())['valid'])

    def test_missing_depth_or_torso_command_is_rejected(self):
        for topic in ['/left_wrist_camera/depth/image_rect_raw/compressedDepth','/lb_leg_traj']:
            rows=good_stats();rows.pop(topic)
            self.assertFalse(self.check(rows)['valid'])

    def test_clock_reset_duplicate_stamp_and_source_clock_offset_fail(self):
        for changes in [dict(bad_source_stamps=1),dict(source_first_ns=1),dict(source_last_ns=None)]:
            rows=good_stats();rows[worker.IMAGE_TOPICS[0]].update(changes)
            self.assertFalse(self.check(rows)['valid'])

    def test_low_rate_gap_and_short_coverage_fail(self):
        for changes in [dict(hz=.1),dict(max_gap_s=3.),dict(first_s=11.8),dict(last_s=10.2)]:
            rows=good_stats();rows[worker.IMAGE_TOPICS[0]].update(changes)
            self.assertFalse(self.check(rows)['valid'])

    def test_unclosed_bag_and_missing_replay_clock_fail(self):
        self.assertFalse(self.check(good_stats(),finalized=False)['valid'])
        self.assertFalse(worker.quality({}, {}, None,None)['valid'])

    def test_final_claw_setpoint_need_not_be_republished_while_robot_settles(self):
        rows=good_stats()
        for topic in worker.REQUIRED_LIVE:rows[topic].update(last_s=15.,source_last_ns=15_000_000_000)
        counts={t:1 for t in worker.LATCHED}
        self.assertTrue(worker.quality(rows,counts,10_000_000_000,15_000_000_000,command_end_ns=12_000_000_000)['valid'])
        rows['/leju_claw_command'].update(last_s=10.1)
        self.assertFalse(worker.quality(rows,counts,10_000_000_000,15_000_000_000,command_end_ns=12_000_000_000)['valid'])

    def test_live_readiness_requires_each_camera_and_distinct_source_frames(self):
        stats={}
        for topic in worker.REQUIRED_LIVE:
            s=worker.Stats();s.add(10.,10_000_000_000);s.add(10.1,10_100_000_000);stats[topic]=s
        now=time.monotonic();self.assertEqual(worker.live_problem(stats,now),'')
        stats[worker.IMAGE_TOPICS[0]].distinct_source=1
        self.assertIn('원본 시각',worker.live_problem(stats,now))
        stats.pop(worker.IMAGE_TOPICS[0])
        self.assertIn('수신 대기',worker.live_problem(stats,now))

    def test_sensor_loss_after_ready_is_detected(self):
        stats={}
        for topic in worker.REQUIRED_LIVE:
            s=worker.Stats();s.add(10.,10_000_000_000);s.add(10.1,10_100_000_000);stats[topic]=s
        stats['/sensors_data_raw'].last_received-=2.
        self.assertIn('/sensors_data_raw',worker.live_problem(stats,time.monotonic()))

if __name__=='__main__':unittest.main()
