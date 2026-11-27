import unittest
from round_sequence import RoundSequence

class RoundTests(unittest.TestCase):
    def test_resume_b_requires_actual_b_handshake(self):
        s=RoundSequence();s.index=1
        self.handshake(s,'LayoutA');self.assertIsNone(s.pending)
        self.handshake(s,'LayoutB',2);self.assertEqual(s.pending,1)
    def handshake(self,s,layout,t=1):
        s.message('Are_you_ready?','',t);s.message('Environment',layout,t+.1)
    def test_old_session2_not_started_as_session1(self):
        s=RoundSequence();self.handshake(s,'LayoutB');self.assertIsNone(s.pending)
    def test_start_a_then_wait_for_actual_b(self):
        s=RoundSequence();self.handshake(s,'LayoutA');self.assertEqual(s.pending,0)
        s.launch(0);s.exited();self.assertFalse(s.finished);self.assertIsNone(s.pending)
        self.handshake(s,'LayoutA',2);self.assertIsNone(s.pending)
        self.handshake(s,'LayoutB',3);self.assertEqual(s.pending,1)
        s.launch(1);s.exited();self.assertFalse(s.finished)
        self.handshake(s,'LayoutA',4);self.assertEqual(s.pending,2)
        s.launch(2);s.exited();self.assertFalse(s.finished)
        self.handshake(s,'LayoutC',5);self.assertEqual(s.pending,3)
        s.launch(3);s.exited();self.assertFalse(s.finished)
    def test_change_while_old_task_active_requests_transition(self):
        s=RoundSequence();s.launch(0);self.handshake(s,'LayoutB')
        self.assertEqual(s.pending,1);self.assertEqual(s.index,0)
    def test_failure_does_not_guess_next_layout(self):
        s=RoundSequence();s.launch(0);s.message('Task_failed','Time_is_up',2)
        self.assertTrue(s.done);self.assertIsNone(s.pending)
        self.handshake(s,'LayoutB',3);self.assertEqual(s.pending,1)
    def test_stale_ready_not_reused(self):
        s=RoundSequence();s.message('Are_you_ready?','',1);s.message('Environment','LayoutA',5)
        self.assertIsNone(s.pending)
    def test_mission_complete_stops_sequence(self):
        s=RoundSequence();s.message('Mission_complete','',1);self.handshake(s,'LayoutA',2)
        self.assertTrue(s.finished);self.assertIsNone(s.pending)
    def test_session2_timeout_waits_for_session3_handshake(self):
        s=RoundSequence();s.index=1;s.launch(1)
        s.message('Task_failed','Time_is_up',10)
        self.assertFalse(s.finished);self.assertTrue(s.done);self.assertIsNone(s.pending)
        self.handshake(s,'LayoutB',11);self.assertIsNone(s.pending)
        self.handshake(s,'LayoutA',12);self.assertEqual(s.pending,2)
    def test_session3_handshake_retires_running_session2(self):
        s=RoundSequence();s.index=1;s.launch(1)
        self.handshake(s,'LayoutA',10);self.assertEqual(s.pending,2)
        s.exited();self.assertFalse(s.finished);self.assertEqual(s.pending,2)
    def test_old_layout_a_cannot_trigger_session3(self):
        s=RoundSequence();self.handshake(s,'LayoutA');s.launch(0)
        self.handshake(s,'LayoutB',3);s.launch(1)
        s.message('Are_you_ready?','',4);self.assertIsNone(s.pending)
        s.message('Environment','LayoutA',8);self.assertIsNone(s.pending)
    def test_no_replay_completed_session3(self):
        s=RoundSequence();s.index=2;s.launch(2);s.exited()
        self.handshake(s,'LayoutA',20);self.assertFalse(s.finished);self.assertIsNone(s.pending)
    def test_correct_session3_runner_and_map(self):
        from run_sessions12 import round_spec,task_command
        from pathlib import Path
        spec=round_spec(2)
        self.assertEqual(spec['layout'],'LayoutA')
        self.assertEqual(spec['config'].name,'session3-search-config.json')
        command=task_command(2,spec['config'],Path('/tmp/test'))
        self.assertTrue(command[1].endswith('/run_session3_search.py'))
        self.assertNotIn('--resume-held-from',command)
        self.assertEqual(command[-1],'/tmp/test/task')

    def test_session3_timeout_waits_for_session4(self):
        s=RoundSequence();s.index=2;s.launch(2)
        s.message('Task_failed','Time_is_up',10)
        self.assertFalse(s.finished)
        self.handshake(s,'LayoutC',12);self.assertEqual(s.pending,3)
    def test_session4_handshake_retires_running_session3(self):
        s=RoundSequence();s.index=2;s.launch(2)
        self.handshake(s,'LayoutC',10);self.assertEqual(s.pending,3)
        s.exited();self.assertFalse(s.finished)
        s.launch(3);s.exited();self.assertFalse(s.finished)
    def test_session4_requires_fresh_handshake_and_correct_config(self):
        from run_sessions12 import round_spec
        s=RoundSequence();s.index=2;s.launch(2);s.exited()
        s.message('Are_you_ready?','',1);s.message('Environment','LayoutC',5)
        self.assertIsNone(s.pending)
        spec=round_spec(3)
        self.assertEqual(spec['layout'],'LayoutC')
        self.assertEqual(spec['config'].name,'session4-live-config.json')
        self.assertEqual(spec['runner'].name,'run_live_session1.py')

    def test_session4_completion_waits_for_fifth_round(self):
        s=RoundSequence();s.index=3;s.launch(3);s.exited()
        self.assertFalse(s.finished)
        self.handshake(s,'LayoutC',10);self.assertIsNone(s.pending)
        self.handshake(s,'LayoutD',12);self.assertEqual(s.pending,4)
        s.launch(4);s.exited();self.assertFalse(s.finished)
    def test_session4_timeout_and_running_task_transition(self):
        for timeout in (False,True):
            s=RoundSequence();s.index=3;s.launch(3)
            if timeout:s.message('Task_failed','Time_is_up',10)
            self.assertFalse(s.finished)
            self.handshake(s,'LayoutD',12);self.assertEqual(s.pending,4)
            s.exited();self.assertFalse(s.finished);s.launch(4)
            s.message('Task_failed','Time_is_up',20);self.assertFalse(s.finished)
    def test_fifth_round_no_skips_or_stale_handshake(self):
        s=RoundSequence();s.launch(0);self.handshake(s,'LayoutD',2);self.assertIsNone(s.pending)
        s=RoundSequence();s.index=3;s.launch(3)
        s.message('Are_you_ready?','',1);s.message('Environment','LayoutD',5);self.assertIsNone(s.pending)
        from run_sessions12 import round_spec
        spec=round_spec(4)
        self.assertEqual(spec['config'].name,'session5-live-config.json')
        self.assertEqual(spec['runner'].name,'run_live_session1.py')

    def test_same_layout_initial_duplicates_do_not_skip_fifth(self):
        s=RoundSequence();s.index=4;s.launch(4)
        self.handshake(s,'LayoutD',10);self.assertIsNone(s.pending)
        s.exited();self.handshake(s,'LayoutD',12);self.assertIsNone(s.pending)
    def test_sixth_requires_new_handshake_after_instruction(self):
        s=RoundSequence();s.index=4;s.launch(4)
        self.handshake(s,'LayoutD',10)
        s.message('Instruction','session5 task',11)
        s.message('Environment','LayoutD',11.1);self.assertIsNone(s.pending)
        self.handshake(s,'LayoutD',12);self.assertEqual(s.pending,5)
        s.exited();self.assertFalse(s.finished);s.launch(5);s.exited();self.assertTrue(s.finished)
    def test_same_layout_success_or_timeout_then_new_handshake(self):
        for terminal in ('Task_succeeded','Task_failed'):
            s=RoundSequence();s.index=4;s.launch(4)
            self.handshake(s,'LayoutD',10)
            s.message(terminal,'',11);self.assertIsNone(s.pending)
            s.message('Environment','LayoutD',11.1);self.assertIsNone(s.pending)
            self.handshake(s,'LayoutD',12);self.assertEqual(s.pending,5)
    def test_session6_config_and_final_failure(self):
        from run_sessions12 import round_spec
        self.assertEqual(round_spec(5)['config'].name,'session6-live-config.json')
        s=RoundSequence();s.index=5;s.launch(5)
        s.message('Task_failed','Time_is_up',10);self.assertTrue(s.finished)

class Session3InitialStartTests(unittest.TestCase):
    def test_fresh_session3_handshake_starts_third_round_only(self):
        from round_sequence import RoundSequence
        seq=RoundSequence();seq.index=2
        seq.message('Are_you_ready?','',0.);seq.message('Environment','LayoutB',.1)
        self.assertIsNone(seq.pending)
        seq.message('Environment','LayoutA',.2);self.assertEqual(seq.pending,2)
        seq.launch(seq.pending);self.assertEqual(seq.index,2)
        seq.message('Instruction','Go to the lobby',1.)
        seq.message('Are_you_ready?','',2.);seq.message('Environment','LayoutC',2.1)
        self.assertEqual(seq.pending,3)
