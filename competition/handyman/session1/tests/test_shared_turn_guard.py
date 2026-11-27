import json,math,tempfile,unittest
from pathlib import Path
from threading import Event
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from candidate_selection import CandidateSelector
from carry_clearance import CarryClearance
from navigation_turn_policy import TurnPolicy,heading_goal,ArrivalHeading,wrap
from shared_turn_guard import SharedTurnGuard

class PolicyTests(unittest.TestCase):
    def guard(self,obstacles=()):
        grid=np.ones((160,160),bool)
        for y,x in obstacles:grid[y:y+4,x:x+4]=False
        return CarryClearance(CandidateSelector(grid,.025,[0,0,0],.2,{}),np.array([[.65,-.04,.3],[.65,.04,.3]]))
    def test_midroute_wrong_side_replaced_before_forwarding(self):
        p=TurnPolicy();v=p.step(self.guard([(65,92)]),[2,2,0],(.2,0,-.5),-math.pi/2)
        self.assertEqual(v[:2],(0.,0.));self.assertGreater(v[2],0)
        v=p.step(self.guard([(65,92)]),[2,2,.1],(.2,0,-.6),-math.pi/2)
        self.assertGreater(v[2],0) # Nav2 still requesting right cannot flip latch.
    def test_same_side_pure_turn_does_not_inherit_slow_nav2_command(self):
        p=TurnPolicy();g=self.guard();pose=[2.,2.,.4469];target=-.43625
        requested=(0.,0.,-.03);duration=0.
        while abs(math.atan2(math.sin(target-pose[2]),math.cos(target-pose[2])))>.04 and duration<10:
            v=p.step(g,pose,requested,target)
            self.assertEqual(v[:2],(0.,0.));self.assertLessEqual(abs(v[2]),.8)
            pose[2]+=v[2]*.05;duration+=.05
        self.assertLess(duration,4.)
        self.assertLess(abs(pose[2]-target),.04)
    def test_small_curved_motion_checked_not_only_static_turn(self):
        p=TurnPolicy()
        class Geometry:
            def choose_turn(self,*a):return dict(delta_rad=.2)
            def occupied(self,pose):return {(1,2)} if pose[0]>.06 else set()
        with self.assertRaisesRegex(ValueError,'no_clear_carry_turn_arc'):
            p.step(Geometry(),[0,0,0],(.3,0,.2),.2)
    def test_small_clear_steering_does_not_choose_alternate_arc(self):
        class Geometry:
            def occupied(self,pose):return set()
            def choose_turn(self,*a):raise AssertionError('small steering must remain continuous')
        p=TurnPolicy()
        for w in (.1,-.1,.08,-.08):
            command=(.08,0.,w)
            self.assertEqual(p.step(Geometry(),[0,0,0],command,.04),command)
            self.assertIsNone(p.remaining)
    def test_translation_without_rotation_unchanged(self):
        self.assertEqual(TurnPolicy().step(None,[0,0,0],(.28,0,0),0),(.28,0,0))
    def test_empty_room_no_extra_spin(self):
        self.assertEqual(TurnPolicy().step(self.guard(),[2,2,0],(.2,0,.3),.5),(.2,0,.3))
    def test_current_path_not_destination_behind_wall(self):
        self.assertAlmostEqual(heading_goal([0,0,0],[5,0,0],[(0,0),(0,.4),(5,0)]),math.pi/2)
        self.assertEqual(heading_goal([5,0,0],[5,0,1.2],[]),1.2)
    def test_recorded_14cm_failure_still_targets_position(self):
        pose=[2.611921643251657,.4697870103775873,-1.6101755240792512]
        goal=[2.70520558938825,.570000402511669,-1.5707964897155762]
        bearing=math.atan2(goal[1]-pose[1],goal[0]-pose[0])
        arrival=ArrivalHeading()
        yaw,changed=arrival.select(pose,goal,[pose,goal])
        self.assertTrue(changed);self.assertFalse(arrival.arrived)
        self.assertAlmostEqual(yaw,bearing)
        self.assertGreater(abs(wrap(yaw-goal[2])),2.)
        geometry=self.guard();geometry.selector.origin[1]=-2.
        velocity=TurnPolicy().step(geometry,pose,(0.,0.,.03),yaw)
        self.assertGreater(velocity[2],0.)
    def test_final_yaw_enters_only_after_xy_arrival(self):
        tracker=ArrivalHeading();goal=[1,0,math.pi/2]
        for distance in (.18,.14,.08,.061):
            yaw,_=tracker.select([1-distance,0,0],goal,[])
            self.assertFalse(tracker.arrived);self.assertAlmostEqual(yaw,0.)
        yaw,changed=tracker.select([.941,0,0],goal,[])
        self.assertTrue(changed);self.assertTrue(tracker.arrived);self.assertEqual(yaw,goal[2])
        # Localization noise cannot repeatedly flip an already-arrived goal.
        yaw,changed=tracker.select([.939,0,0],goal,[])
        self.assertFalse(changed);self.assertEqual(yaw,goal[2])
        # Real displacement outside the existing acceptance requires XY again.
        yaw,changed=tracker.select([.90,0,0],goal,[])
        self.assertTrue(changed);self.assertFalse(tracker.arrived);self.assertEqual(yaw,0.)
    def test_new_goal_cannot_inherit_previous_xy_arrival(self):
        tracker=ArrivalHeading();tracker.select([1,0,0],[1,0,1.2],[])
        yaw,changed=tracker.select([1,0,0],[1.14,0,-1.2],[])
        self.assertTrue(changed);self.assertFalse(tracker.arrived);self.assertEqual(yaw,0.)
    def test_heading_uses_configured_position_tolerance(self):
        self.assertEqual(heading_goal([.95,0,0],[1,0,1.2],[],xy_tolerance=.04),0.)
        self.assertEqual(heading_goal([.95,0,0],[1,0,1.2],[],xy_tolerance=.06),1.2)
    def test_turn_stops_at_selected_endpoint(self):
        p=TurnPolicy();p.remaining=.1;p.previous=[0,0,0]
        self.assertEqual(p.step(None,[0,0,.09],(0,0,-.4),0),(0.,0.,0.));self.assertIsNone(p.remaining)

class WiringTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name);self.calls=[];self.adjusts=[]
        self.rt=SimpleNamespace(sample=lambda **kw:None,state=lambda:dict(poses={'map':[2,2,0]},joints=[0]*5+[.3]),base_move=lambda *a:'unchanged')
        self.motion=SimpleNamespace(command=lambda args,*a,**kw:self.calls.append(args) or {'status':'arrived'},
            adjust=lambda *a,**kw:self.adjusts.append((a,kw)),session_cancel_event=Event())
        self.stages=SimpleNamespace(runtime=self.rt,motion=self.motion,output=self.root,check=lambda e:None,
            candidate_selector=CandidateSelector(np.ones((100,100),bool),.05,[0,0,0],.2,{}))
        self.guard=SharedTurnGuard(self.stages)
        self.guard.envelope=lambda:Path('/tmp/current-envelope.json')
    def test_all_nav_variants_and_early_room_entry_keep_arguments(self):
        for exe in ('navigate_carry_trial.py','efficient_navigation.py','room_entry_navigation.py','carry_navigation_runtime.py'):
            self.motion.command([exe,'--goal',1,2,3,'--room-region','region.json','--carry-envelope','old.json'])
            a=self.calls[-1];self.assertEqual(Path(a[0]).name,'carry_navigation_runtime.py')
            self.assertEqual(a.count('--carry-envelope'),1);self.assertNotIn('old.json',a)
            self.assertIn('--room-region',a)
    def test_finite_mixed_motion_separated_preserving_target_yaw(self):
        selected=[];self.guard.turn=lambda yaw:selected.append(yaw)
        self.motion.adjust('align',[1,2,.4],[.3,.1,-.2],[0]*5,.3)
        self.assertEqual(self.adjusts[0][0][2],[.3,.1,0.]);self.assertAlmostEqual(selected[0],.2)
    def test_empty_base_move_same_guard_and_return_contract(self):
        selected=[];self.guard.turn=lambda yaw:selected.append(yaw)
        result=self.rt.base_move([0,0,.2],[0]*5,'grasp-align')
        self.assertEqual(selected,[.2]);self.assertIn('poses',result)
    def test_no_change_to_gripper_or_arm_commands(self):
        self.motion.command(['pregrasp_raised_trial.py','--run'])
        self.assertEqual(self.calls[-1],['pregrasp_raised_trial.py','--run'])
    def test_persistent_dynamic_blockage_escapes_before_retry(self):
        output=self.root/'nav.json';calls=[];escapes=[]
        self.stages.grasp_backend=SimpleNamespace(record=lambda e:None)
        self.guard.geometry=lambda:object()
        self.guard.turner.escape=lambda g,c,excluded,**kw:escapes.append(list(excluded))
        def command(args,*a,**kw):
            calls.append(args)
            if len(calls)==1:
                output.write_text(json.dumps(dict(error='carry_replan_required:dynamic_sweep_blocked',
                    nav_reset=True,carry_footprint_restored=True)))
                raise RuntimeError('blocked')
            self.assertEqual(len(escapes),1)
            return {'status':'arrived'}
        self.guard.original_command=command
        result=self.motion.command(['navigate_carry_trial.py','--goal',3,3,0,'--output',output])
        self.assertEqual(result['status'],'arrived');self.assertEqual(escapes,[[[2,2,0]]])
    def test_directed_turn_does_not_reenter_selection(self):
        self.guard.busy=True;self.guard.turn=lambda _:self.fail('recursive selection')
        self.motion.adjust('x',[0,0,0],[0,0,.2],[0]*5,.3)
        self.assertEqual(self.adjusts[0][0][2],[0,0,.2])

class RelayTests(unittest.TestCase):
    def relay(self):
        from navigation_turn_relay import NavigationTurnRelay
        r=NavigationTurnRelay.__new__(NavigationTurnRelay)
        self.sent=[]
        r.Twist=lambda:SimpleNamespace(linear=SimpleNamespace(x=0.,y=0.),angular=SimpleNamespace(z=0.))
        r.pub=SimpleNamespace(publish=lambda m:self.sent.append((m.linear.x,m.linear.y,m.angular.z)))
        r.enabled=True;r.latest=(.2,0,-.3);r.last_time=0.;r.error=None
        r.arrival=ArrivalHeading();r.record=lambda row:None
        r.stats=dict(stale_input_stops=0,tf_wait_ticks=0,dynamic_wait_ticks=0,forwarded=0,directed_turn_ticks=0,max_angular_speed=0.)
        return r
    def test_stale_input_sends_zero(self):
        r=self.relay();r.tick();self.assertEqual(self.sent,[(0.,0.,0.)])
    def test_bypass_publisher_rejected_before_enable(self):
        r=self.relay();r.enabled=False
        r.node=SimpleNamespace(get_publishers_info_by_topic=lambda t:[SimpleNamespace(node_name='controller_server')],get_name=lambda:'guard')
        with self.assertRaisesRegex(ValueError,'bypasses_turn_guard'):r.enable()
        self.assertFalse(r.enabled)
    def test_guard_failure_never_forwards_original_angular_command(self):
        import time
        r=self.relay();r.last_time=time.monotonic();r.pose=lambda:[0,0,0];r.geometry=None;r.goal=[1,0,0];r.path=[]
        def fail(*a):raise ValueError('no_clear_carry_turn_arc')
        r.policy=SimpleNamespace(remaining=None,step=fail,can_reuse=lambda *a:False,reset=lambda:None)
        r.tick();self.assertTrue(all(v==(0.,0.,0.) for v in self.sent));self.assertTrue(r.error.startswith('carry_replan_required'))

class ScanTimingTests(unittest.TestCase):
    def scan(self,seconds):
        ns=round(seconds*1e9)
        return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=ns//10**9,nanosec=ns%10**9)))
    def test_newest_scan_ahead_of_tf_uses_exact_older_pair(self):
        from navigation_turn_relay import TransformMatchedScan
        m=TransformMatchedScan();old=self.scan(10.);new=self.scan(10.1)
        m.add(old);m.add(new)
        class ExtrapolationException(Exception):pass
        def lookup(s):
            if s is new:raise ExtrapolationException('future by 70ms')
            return 'old_exact_tf'
        self.assertEqual(m.select(10150000000,lookup),(old,'old_exact_tf'))
        self.assertEqual(m.failures,1)
        self.assertEqual(m.select(10160000000,lambda s:'new_exact_tf'),(new,'new_exact_tf'))
    def test_matched_pair_not_retimestamped_and_expires(self):
        from navigation_turn_relay import TransformMatchedScan
        m=TransformMatchedScan();s=self.scan(10.);m.add(s)
        m.select(10000000000,lambda s:'exact')
        self.assertEqual(m.select(10600000000,lambda s:self.fail('must reuse')),(s,'exact'))
        self.assertIsNone(m.select(10710000000,lambda s:self.fail('expired')))
    def test_no_pair_and_unexpected_error_are_not_ignored(self):
        from navigation_turn_relay import TransformMatchedScan
        m=TransformMatchedScan();m.add(self.scan(10))
        class ConnectivityException(Exception):pass
        def disconnected(s):raise ConnectivityException('missing')
        self.assertIsNone(m.select(10100000000,disconnected))
        def broken(s):raise RuntimeError('bad data')
        with self.assertRaisesRegex(RuntimeError,'bad data'):m.select(10100000000,broken)
    def test_reusable_pure_turn_does_not_send_intermediate_zero(self):
        import time
        fixture=RelayTests();r=fixture.relay();r.last_time=time.monotonic();r.latest=(0.,0.,.1)
        r.pose=lambda:[0,0,0];r.goal=[1,0,0];r.path=[];r.geometry=None;r.last_choice=None
        r.arrival.select([0,0,0],r.goal,[])
        r.policy=SimpleNamespace(remaining=None,last_choice=None,can_reuse=lambda *a:True,step=lambda *a:(0.,0.,.1))
        r.dynamic_clear=lambda *a:True
        r.tick();self.assertEqual(fixture.sent,[(0.,0.,.1)])

class GoalHandoffTests(unittest.TestCase):
    def relay(self,events):
        r=SimpleNamespace(enabled=True,path=['old'],latest=(.2,0,.1),last_time=123.,waiting_dynamic=True)
        r.zero=lambda:events.append(('zero',r.enabled))
        r.policy=SimpleNamespace(reset=lambda:events.append(('reset',r.enabled)))
        return r
    def test_old_action_terminal_and_stopped_before_clear_and_reenable(self):
        from navigation_turn_policy import reset_navigation_goal
        events=[];r=self.relay(events)
        def cancel():events.append(('cancel',r.enabled))
        def ready(**kw):
            self.assertEqual(kw,dict(stopped=True));events.append(('ready',r.enabled));return [.15,-1.73,-3.06]
        self.assertEqual(reset_navigation_goal(r,cancel,ready),[.15,-1.73,-3.06])
        self.assertEqual(events,[('zero',False),('cancel',False),('ready',False),('reset',False)])
        self.assertTrue(r.enabled);self.assertIsNone(r.latest);self.assertEqual(r.path,[]);self.assertEqual(r.last_time,0)
        self.assertFalse(r.waiting_dynamic)
    def test_failed_cancellation_cannot_reenable_old_velocity(self):
        from navigation_turn_policy import reset_navigation_goal
        events=[];r=self.relay(events)
        def fail():raise RuntimeError('cancel failed')
        with self.assertRaisesRegex(RuntimeError,'cancel failed'):
            reset_navigation_goal(r,fail,lambda **kw:self.fail('must not reacquire'))
        self.assertFalse(r.enabled)
    def test_failed_reacquisition_cannot_reenable(self):
        from navigation_turn_policy import reset_navigation_goal
        events=[];r=self.relay(events)
        def fail(**kw):raise RuntimeError('feedback lost')
        with self.assertRaisesRegex(RuntimeError,'feedback lost'):
            reset_navigation_goal(r,lambda:None,fail)
        self.assertFalse(r.enabled)

class NavigationTerminalRecoveryTests(unittest.TestCase):
    def terminal(self,status,kind='door_center'):
        # Execute the actual terminal-result branch without starting ROS.
        import ast,textwrap
        from navigation_turn_policy import reset_navigation_goal
        source=(Path(__file__).resolve().parents[1]/'carry_navigation_runtime.py').read_text()
        start=source.index('                handle,future=current\n')
        end=source.index('            except FeedbackPending as exc:',start)
        code='for iteration in range(1):\n'+textwrap.indent(textwrap.dedent(source[start:end]),'    ')
        compile(ast.parse(code),'<actual navigation terminal branch>','exec')
        pose=[.1498379053,-1.7258707255,-3.0625656178]
        events=[];r=GoalHandoffTests().relay(events)
        future=SimpleNamespace(done=lambda:True,result=lambda:SimpleNamespace(status=status))
        clock=[0.]
        def pump(dt):clock[0]+=max(dt,.05)
        env=dict(current=(object(),future),check=lambda:pose,math=math,wrap=wrap,
            row=dict(kind=kind,pose=[-.5900000334,-1.7641439438,math.pi]),
            policy=dict(final_acceptance=[.085,.11]),pass_assessment=lambda *a:dict(passed=False),
            report=dict(reached=[],nav2_retries=[]),index=1,turn_relay=r,
            reset_navigation_goal=reset_navigation_goal,cancel_all=lambda:events.append('cancel'),
            ready=lambda **kw:pose,front_distance=lambda:None,
            retry=SimpleNamespace(failed=lambda *a,**kw:dict(wait_s=.5,retry=1,task_retained=True)),
            event=lambda e:events.append(e),time=SimpleNamespace(monotonic=lambda:clock[0]),pump=pump,
            ProgressWatch=lambda:object())
        exec(compile(code,'<actual navigation terminal branch>','exec'),env)
        self.assertEqual(env['index'],1);self.assertIsNone(env['current'])
        self.assertEqual(env['report']['reached'],[])
        self.assertIsNone(r.latest);self.assertTrue(r.enabled)
        self.assertIn('cancel',events);self.assertGreaterEqual(clock[0],.5)
        return env['report']['nav2_retries'][0]
    def test_recorded_success_74cm_before_door_retries_same_landmark(self):
        self.assertEqual(self.terminal(4)['reason'],'landmark_not_physically_passed_reacquire_and_retry')
    def test_nav2_abort_retains_task_and_retries(self):
        self.assertEqual(self.terminal(6)['reason'],'nav2_local_recovery_exhausted')
    def test_final_goal_success_outside_tolerance_is_not_accepted(self):
        self.assertEqual(self.terminal(4,'final')['reason'],'final_pose_requires_reacquisition_or_correction')
