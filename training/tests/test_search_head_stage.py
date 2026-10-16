import sys
import unittest
import json
import tempfile
from types import SimpleNamespace
from unittest.mock import Mock,patch
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from search_head_stage import proof_valid,StopConfirmation,validate_environment,HeadStage
from head_view_trial import Settling


class HeadTraceTests(unittest.TestCase):
    def setUp(self):
        self.directory=tempfile.TemporaryDirectory();self.addCleanup(self.directory.cleanup)
        h=self.h=HeadStage.__new__(HeadStage)
        h.path=Path(self.directory.name)/'head-ready.json';h.task='task';h.point='point'
        h.goal=(0.,-.25);h.gate=Settling(h.goal);h.latest=None
        h.sent=None;h.armed=None;h.ready=False;h.failed=None;h.cancelled=False
        h.stop=None;h.stop_recorded=False;h.trace_count=0;h.previous_receive=None;h.last_wait=None
        h.trace_error=None
        h.sub=Mock();h.pub=Mock();h.pub.get_subscription_count.return_value=1
        h.node=Mock();h.node.count_publishers.return_value=1
        h.node.get_clock.return_value.now.return_value.nanoseconds=2_000_000_000
    def rows(self):
        return [json.loads(s) for s in self.h.path.with_name('head-trace.jsonl').read_text().splitlines()]
    def message(self,positions=(0.,0.),stamp=2_000_000_000):
        return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp//10**9,nanosec=stamp%10**9)),
                               name=['head_pan_joint','head_tilt_joint'],position=positions)
    def test_fault_retains_specific_reason_once(self):
        self.h.fail('head_stage_timeout');self.h.fail('other')
        self.assertEqual(len(self.rows()),1)
        self.assertEqual(self.rows()[0]['reason'],'head_stage_timeout')
        self.assertFalse(self.rows()[0]['command_sent'])
    def test_feedback_records_angle_and_age(self):
        self.h.receive(self.message())
        row=self.rows()[0]
        self.assertEqual(row['positions'],[0.,0.]);self.assertEqual(row['age_s'],0.)
        self.assertEqual(row['feedback_reason'],'outside_target_tolerance')
        self.h.pub.publish.assert_not_called()
    def test_invalid_number_is_recorded_without_nan(self):
        self.h.receive(self.message((0.,float('nan'))))
        self.assertEqual(self.rows()[0]['raw_positions'],[0.,None])
        self.assertIsNone(self.h.latest)
    def test_future_feedback_still_rejected(self):
        self.h.receive(self.message(stamp=2_021_000_000))
        self.assertIsNone(self.h.latest)
        self.assertEqual(self.rows()[0]['feedback_reason'],'stale_or_invalid_feedback')
    def test_cap_does_not_drop_fault_record(self):
        self.h.trace_count=2000
        self.h.trace('feedback');self.h.fail('head_stage_timeout')
        self.assertEqual([r['event'] for r in self.rows()],['head_fault'])
    def test_wait_records_only_changes(self):
        self.h.wait('fresh_feedback_required');self.h.wait('fresh_feedback_required')
        self.assertEqual(len(self.rows()),1)
    def test_timeout_does_not_send_command(self):
        self.h.armed=1.
        with patch('search_head_stage.time.monotonic',return_value=10.):self.h.tick(True)
        self.assertEqual(self.h.failed,'head_stage_timeout');self.h.pub.publish.assert_not_called()
    def test_close_never_sends_hold(self):
        self.h.close();self.h.pub.publish.assert_not_called()
        self.assertFalse(self.rows()[0]['stop_recorded'])
    def test_trace_write_failure_does_not_block_close(self):
        with patch.object(Path,'open',side_effect=OSError('disk unavailable')):self.h.close()
        self.h.node.destroy_subscription.assert_called_once()
        self.h.node.destroy_publisher.assert_called_once()
        self.assertEqual(self.h.trace_error,'disk unavailable')
    def stable(self,i,positions=(0.,-.25)):
        ns=2_000_000_000+i*100_000_000
        self.h.node.get_clock.return_value.now.return_value.nanoseconds=ns
        with patch('search_head_stage.time.monotonic',return_value=2.+i*.1):
            self.h.receive(self.message(positions,ns))
    def prepare_ready(self):
        self.h.sent=0.
        for i in range(10):self.stable(i)
        self.assertTrue(self.h.ready)
    def test_ready_proof_renews_without_new_motion(self):
        self.prepare_ready();before=json.loads(self.h.path.read_text())
        for i in range(10,40):self.stable(i)
        after=json.loads(self.h.path.read_text())
        self.assertGreater(after['stamp_ns'],before['stamp_ns'])
        self.assertTrue(proof_valid(after,'task','point',-.25,6_000_000_000))
        self.assertFalse(proof_valid(before,'task','point',-.25,6_000_000_000))
        self.h.pub.publish.assert_not_called()
        self.assertEqual(sum(r['event']=='head_ready' for r in self.rows()),1)
    def test_bad_angle_does_not_renew(self):
        self.prepare_ready();before=self.h.path.read_bytes();self.stable(10,(0.,0.))
        self.assertEqual(self.h.path.read_bytes(),before)
        self.assertEqual(self.h.failed,'head_changed_after_ready')
        self.stable(11);self.assertEqual(self.h.path.read_bytes(),before)
    def test_cancelled_does_not_renew(self):
        self.prepare_ready();before=self.h.path.read_bytes()
        self.h.cancelled=True;self.h.try_hold=Mock();self.stable(10)
        self.assertEqual(self.h.path.read_bytes(),before)
    def test_feedback_gap_does_not_renew(self):
        self.prepare_ready();before=self.h.path.read_bytes();self.stable(15)
        self.assertEqual(self.h.path.read_bytes(),before)
        self.assertEqual(self.h.failed,'head_feedback_invalid_after_ready')
    def test_no_feedback_expires_proof(self):
        self.prepare_ready();before=self.h.path.read_bytes()
        with patch('search_head_stage.time.monotonic',return_value=5.):self.h.tick(True)
        self.assertEqual(self.h.failed,'head_feedback_timeout')
        self.assertEqual(self.h.path.read_bytes(),before)
        self.assertFalse(proof_valid(json.loads(before),'task','point',-.25,5_000_000_000))


class HeadProofTests(unittest.TestCase):
    def setUp(self):
        self.row=dict(task_id='task',point_id='point',positions=[0.,-.25],state='ready',stamp_ns=2_000_000_000)
    def valid(self):return proof_valid(self.row,'task','point',-.25,2_100_000_000)
    def test_valid(self):self.assertTrue(self.valid())
    def test_wrong_task(self):self.row['task_id']='old';self.assertFalse(self.valid())
    def test_wrong_point(self):self.row['point_id']='other';self.assertFalse(self.valid())
    def test_wrong_angle(self):self.row['positions']=[0.,-.5];self.assertFalse(self.valid())
    def test_stale(self):self.row['stamp_ns']=1;self.assertFalse(self.valid())
    def test_future(self):self.row['stamp_ns']=3_000_000_000;self.assertFalse(self.valid())
    def test_bounded_future_proof(self):
        self.row['stamp_ns']=2_120_000_000;self.assertTrue(self.valid())
    def test_beyond_future_bound(self):
        self.row['stamp_ns']=2_120_000_001;self.assertFalse(self.valid())
    def test_not_ready(self):self.row['state']='moving';self.assertFalse(self.valid())
    def test_bad_type(self):self.row['stamp_ns']=True;self.assertFalse(self.valid())


class StopTests(unittest.TestCase):
    def setUp(self):self.g=StopConfirmation((0.,-.25),1_000_000_000,0.)
    def feed(self,i,pose=(0.,-.25)):
        ns=1_000_000_000+i*100_000_000
        self.g.sample(['head_pan_joint','head_tilt_joint'],pose,ns,ns,i*.1)
    def test_time_alone_never_sufficient(self):self.assertFalse(self.g.verified(10.))
    def test_fresh_stable_confirms(self):
        for i in range(1,10):self.feed(i)
        self.assertTrue(self.g.verified(.95))
    def test_old_feedback_rejected(self):self.feed(0);self.assertFalse(self.g.verified(1.))
    def test_pre_hold_sample_with_positive_clock_offset_rejected(self):
        self.g.sample(['head_pan_joint','head_tilt_joint'],(0.,-.25),1_014_000_000,1_001_000_000,.001)
        self.assertIsNone(self.g.latest)
    def test_cutoff_boundary_rejected(self):
        self.g.sample(['head_pan_joint','head_tilt_joint'],(0.,-.25),1_020_000_000,1_010_000_000,.01)
        self.assertIsNone(self.g.latest)
    def test_skewed_post_hold_feedback_confirms(self):
        for i in range(1,10):
            ns=1_000_000_000+i*100_000_000
            self.g.sample(['head_pan_joint','head_tilt_joint'],(0.,-.25),ns+14_000_000,ns,i*.1)
        self.assertTrue(self.g.verified(.95))
    def test_expired_feedback_rejected(self):
        for i in range(1,10):self.feed(i)
        self.assertFalse(self.g.verified(1.41))
    def test_moving_rejected(self):
        for i in range(1,10):self.feed(i,(0.,-.25+i*.01))
        self.assertFalse(self.g.verified(.95))
    def test_wrong_pose_rejected(self):
        for i in range(1,10):self.feed(i,(0.,0.))
        self.assertFalse(self.g.verified(.95))
    def test_duplicate_resets(self):
        for i in range(1,10):self.feed(i)
        self.feed(9);self.assertFalse(self.g.verified(.95))
    def test_future_rejected(self):
        self.g.sample(['head_pan_joint','head_tilt_joint'],(0.,-.25),2_000_000_000,1_000_000_000,.1)
        self.assertFalse(self.g.verified(.1))


class EnvironmentTests(unittest.TestCase):
    def test_live_requires_explicit_flag(self):
        with self.assertRaises(ValueError):validate_environment('71','1',False)
    def test_live_explicit(self):validate_environment('71','1',True)
    def test_isolated(self):validate_environment('73','1',False)
    def test_wrong_domain(self):
        with self.assertRaises(ValueError):validate_environment('0','1',True)
    def test_nonlocal(self):
        with self.assertRaises(ValueError):validate_environment('71','0',True)


if __name__=='__main__':unittest.main()
