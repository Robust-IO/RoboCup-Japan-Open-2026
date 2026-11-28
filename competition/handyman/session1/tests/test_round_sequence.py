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


class LimitedRoundTests(unittest.TestCase):
    def test_boundary_and_default(self):
        from round_sequence import LimitedRound
        for limit in (0,2,6,4294967295):
            s=LimitedRound(limit)
            for index in range(6):
                self.assertEqual(s.begin(index),'I_am_ready' if index>=limit else None)
                s.receive('Instruction',10)
                self.assertIsNone(s.tick(14.999))
                self.assertEqual(s.tick(15),'Give_up' if index>=limit else None)
                self.assertIsNone(s.tick(100))

    def test_only_first_instruction_starts_timer(self):
        from round_sequence import LimitedRound
        s=LimitedRound(0)
        self.assertIsNone(s.receive('Instruction',0))  # not yet begun
        s.begin(0)
        self.assertIsNone(s.tick(100))
        s.receive('Corrected_instruction',100)
        self.assertIsNone(s.tick(200))
        s.receive('Instruction',201)
        s.receive('Instruction',203)
        self.assertIsNone(s.receive('Are_you_ready?',204))
        self.assertIsNone(s.tick(205.999))
        self.assertEqual(s.tick(206),'Give_up')
        s.receive('Instruction',207)
        self.assertIsNone(s.tick(300))

    def test_terminal_events_cancel_delayed_give_up(self):
        from round_sequence import LimitedRound
        for message in ('Task_failed','Task_succeeded','Mission_complete'):
            s=LimitedRound(0);s.begin(0);s.receive('Instruction',0)
            s.receive(message,2)
            self.assertIsNone(s.tick(10))
            self.assertIsNone(s.receive('Are_you_ready?',11))
        s=LimitedRound(0);s.begin(0);s.receive('Instruction',0);s.cancel()
        self.assertIsNone(s.tick(10))

    def test_next_round_does_not_inherit_old_timer(self):
        from round_sequence import LimitedRound
        s=LimitedRound(0);s.begin(4);s.receive('Instruction',0)
        s.begin(5);self.assertIsNone(s.tick(10))
        s.receive('Instruction',20)
        self.assertEqual(s.tick(25),'Give_up')

    def test_six_abandoned_rounds_follow_real_handshakes(self):
        from round_sequence import LimitedRound
        seq=RoundSequence();skip=LimitedRound(0)
        for index,layout in enumerate(seq.layouts):
            now=index*20
            seq.message('Environment',layout,now)
            seq.message('Are_you_ready?','',now+.1)
            self.assertEqual(seq.pending,index)
            seq.launch(index);self.assertEqual(skip.begin(index),'I_am_ready')
            seq.message('Instruction','task',now+1)
            skip.receive('Instruction',now+1)
            self.assertIsNone(skip.tick(now+5.99))
            self.assertEqual(skip.tick(now+6),'Give_up')
            seq.message('Task_failed','Give_up',now+7)
            skip.receive('Task_failed',now+7)
            self.assertIsNone(skip.tick(now+8))
        self.assertTrue(seq.finished)


class SupervisorLimitIntegrationTests(unittest.TestCase):
    def test_fixed_and_dynamic_supervisor_routes_normal_and_abandoned_rounds(self):
        """Run main's real event loop with fake ROS/time/process boundaries."""
        import contextlib,io,tempfile,types
        from pathlib import Path
        from unittest.mock import Mock,patch
        import run_sessions12 as supervisor

        for dynamic in (False,True):
            for limit in (0,2,6):
                with self.subTest(dynamic=dynamic,limit=limit),tempfile.TemporaryDirectory() as tmp:
                    clock=[0.]
                    wire=[]
                    events=[]
                    setups=[]
                    commands=[]
                    for index,layout in enumerate(RoundSequence.layouts):
                        base=index*20.
                        events.extend([(base,'Environment',layout),(base+.1,'Are_you_ready?',''),
                                       (base+.2,None,''),(base+1,'Instruction','task'),
                                       (base+3,'Instruction','duplicate'),
                                       (base+5.999,None,''),(base+6,None,''),(base+7,None,''),
                                       (base+8,'Task_failed','Give_up' if index>=limit else 'test end')])
                    class Message:
                        def __init__(self,message='',detail=''):
                            self.message=message;self.detail=detail
                    class Node:
                        def create_publisher(self,*args):
                            return types.SimpleNamespace(publish=lambda msg:wire.append((clock[0],msg.message,msg.detail)))
                        def create_subscription(self,kind,topic,callback,qos):
                            self.receive=callback
                        def destroy_node(self):pass
                    node=Node()
                    def spin_once(node,timeout_sec):
                        if not events:raise AssertionError('supervisor failed to finish six rounds')
                        now,msg,detail=events.pop(0);clock[0]=now
                        if msg:node.receive(Message(msg,detail))
                    class Future:
                        def __init__(self,value):self.value=value
                        def done(self):return True
                        def result(self):return self.value
                    class Pool:
                        def __init__(self,**kwargs):pass
                        def submit(self,fn,*args):return Future(fn(*args))
                        def shutdown(self,**kwargs):pass
                    def setup(index,out):
                        setups.append(index);return Path(tmp)/'config.json'
                    class Task:
                        pid=123456
                        def __init__(self,index):self.index=index
                        def poll(self):return 0 if clock[0]>=self.index*20+8 else None
                        def wait(self):return 0
                    def spawn(command,log):
                        index=int(log.parent.name.removeprefix('session'))-1
                        commands.append((index,command));return Task(index)
                    ros=types.SimpleNamespace(init=lambda:None,create_node=lambda name:node,
                                              spin_once=spin_once,shutdown=lambda:None)
                    prewarm=Mock(return_value=types.SimpleNamespace(pid=999))
                    modules={'rclpy':ros,'handyman_msgs':types.ModuleType('handyman_msgs'),
                             'handyman_msgs.msg':types.SimpleNamespace(HandymanMsg=Message),
                             'ensure_clock_bridge':types.SimpleNamespace(ensure=lambda:None),
                             'model_prewarm':types.SimpleNamespace(start=prewarm,stop_waiter=lambda p:None)}
                    argv=['run_sessions12.py','--execute','--from-session','1','--N',str(limit)]
                    if dynamic:argv.append('--dynamic-instructions')
                    with patch.dict('sys.modules',modules),patch.object(supervisor.sys,'argv',argv),                          patch.object(supervisor,'RUNS',Path(tmp)),patch.object(supervisor,'processes',return_value=[]),                          patch.object(supervisor,'setup',side_effect=setup),patch.object(supervisor,'spawn',side_effect=spawn),                          patch.object(supervisor,'retire_services') as retire,patch.object(supervisor,'ThreadPoolExecutor',Pool),                          patch.object(supervisor,'ConsoleExit',return_value=types.SimpleNamespace(poll=lambda:False)),                          patch.object(supervisor.time,'monotonic',side_effect=lambda:clock[0]),                          patch.object(supervisor.signal,'signal'),patch.object(supervisor.os,'killpg'),                          contextlib.redirect_stdout(io.StringIO()):
                        supervisor.main()
                    self.assertEqual(setups,list(range(limit)))
                    self.assertEqual([i for i,_ in commands],list(range(limit)))
                    for i,command in commands:
                        expected='run_dynamic_session.py' if dynamic else ('run_session3_search.py' if i==2 else 'run_live_session1.py')
                        self.assertEqual(Path(command[1]).name,expected)
                    self.assertEqual([(t,m) for t,m,d in wire if m=='Give_up'],
                                     [(i*20.+6,'Give_up') for i in range(limit,6)])
                    self.assertEqual([m for _,m,_ in wire].count('I_am_ready'),6-limit)
                    self.assertTrue(all(d=='' for _,_,d in wire))
                    self.assertEqual(retire.call_count,6-limit)
                    self.assertEqual(prewarm.call_count,int(limit>0))
