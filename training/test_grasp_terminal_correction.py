"""Synthetic geometry/controller wiring only; no ROS or robot connections."""
import copy
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import Mock,patch

import numpy as np
import test_apple_approach_lease as fixtures
from apple_approach_lease import CONTACT_XY,closure_position_error,validate_terminal_correction
from grasp_terminal_correction import terminal_delta,correct_terminal,completed_context
from reviewed_carry_adjustment import terminal_grasp_limit


def previous(stamp=900_000_000):
    return dict(status='endpoint_verified',command_sent=True,grip_commands_sent=False,
                single_grasp_approach=True,motion_completed_at_ns=stamp,final_odom=[0.,0.,0.],
                identity_context=dict(recording='/CURRENT-RUN',epoch='epoch',task_id='task'))


class GeometryTests(unittest.TestCase):
    def test_real_r17_failure_combines_forward_and_lateral_error(self):
        target=np.array([.7156999616220105,.0584170883261531,.5430623715286986])
        height=.5324080327487622
        delta=terminal_delta(target,height)
        np.testing.assert_allclose(delta,[.026404743122010577,-.019582911673846903,0.])
        closure_position_error(target-np.array(delta),height)

    def test_already_graspable_is_not_forced_to_exact_center(self):
        self.assertIsNone(terminal_delta([.704,.088,.539],.53))

    def test_pure_lateral_and_small_reverse_are_allowed(self):
        for point in ([CONTACT_XY[0],.10,.536],[.68,.10,.536]):
            delta=terminal_delta(point,.53)
            self.assertLessEqual(delta[0],0)
            closure_position_error(np.array(point)-delta,.53)

    def test_invalid_height_does_not_trigger_useless_horizontal_steps(self):
        with self.assertRaisesRegex(ValueError,'height_requires_arm_replanning'):
            terminal_delta([.72,.10,.56],.53)

    def test_outside_near_region_is_not_a_second_coarse_approach(self):
        with self.assertRaisesRegex(ValueError,'outside_terminal_near_region'):
            terminal_delta([1.2,.078,.536],.53)

    def test_controller_pose_still_empty_and_horizontal(self):
        arm=[.255,-1.7,0,.1292036732051034,0]
        for delta in ([0,.025,0],[-.01,.025,0]):
            self.assertEqual(terminal_grasp_limit(delta,arm,.65,'current'),.4)
        for delta,opening in [([0,.025,0],.4),([0,0,.1],.65),([0,0,0],.65)]:
            with self.assertRaises(ValueError):terminal_grasp_limit(delta,arm,opening,'current')


class SchedulingTests(unittest.TestCase):
    def run_loop(self,targets,move_status='endpoint_verified'):
        observe=Mock(side_effect=[(target,dict(frame=i)) for i,target in enumerate(targets)])
        counter=[1_000_000_000]
        def perform(delta,context,prior):
            counter[0]+=1_000_000_000
            value=previous(counter[0]);value.update(status=move_status,single_grasp_approach=False,grasp_terminal_correction=True)
            return value
        move=Mock(side_effect=perform)
        return lambda:correct_terminal(observe,move,.53,previous()),observe,move

    def test_actual_failure_gets_one_correction_then_independent_close_check(self):
        run,observe,move=self.run_loop([[.7157,.0584,.54],[.691,.079,.54]])
        result=run()
        self.assertEqual(result['corrections'],1)
        self.assertFalse(result['closure_authorized'])
        self.assertEqual(move.call_count,1)
        self.assertEqual(observe.call_args_list[0].args,(900_000_000,))
        self.assertEqual(observe.call_args_list[1].args,(2_000_000_000,))

    def test_already_ready_does_not_move(self):
        run,observe,move=self.run_loop([[.704,.078,.536]])
        self.assertEqual(run()['corrections'],0);move.assert_not_called()

    def test_second_correction_allowed_only_while_converging(self):
        run,observe,move=self.run_loop([[.73,.105,.536],[.712,.092,.536],[.692,.079,.536]])
        self.assertEqual(run()['corrections'],2)
        self.assertEqual(move.call_count,2)

    def test_nonconvergence_stops_without_replaying(self):
        run,observe,move=self.run_loop([[.72,.10,.536],[.73,.11,.536]])
        with self.assertRaisesRegex(RuntimeError,'not_converging'):run()
        self.assertEqual(move.call_count,1)

    def test_failed_correction_does_not_trigger_retry_or_close(self):
        run,observe,move=self.run_loop([[.72,.10,.536]],move_status='failed')
        with self.assertRaisesRegex(ValueError,'no_failed_action_retry'):run()
        self.assertEqual(move.call_count,1);self.assertEqual(observe.call_count,1)

    def test_budget_does_not_create_unbounded_micro_moves(self):
        run,observe,move=self.run_loop([[x,.078,.536] for x in (.76,.75,.74,.73,.72,.711)])
        with self.assertRaisesRegex(RuntimeError,'budget_exhausted'):run()
        self.assertEqual(move.call_count,5)

    def test_fifth_correction_can_succeed_without_a_sixth_movement(self):
        run,observe,move=self.run_loop([[x,.078,.536] for x in (.76,.75,.74,.73,.72,.69)])
        self.assertEqual(run()['corrections'],5)
        self.assertEqual(move.call_count,5)
        self.assertEqual(observe.call_count,6)

    def test_missing_or_failed_predecessor_never_authorizes_a_correction(self):
        for update in (dict(status='failed'),dict(motion_completed_at_ns=None),dict(command_sent=False)):
            prior=previous();prior.update(update);move=Mock();observe=Mock()
            with self.assertRaises(ValueError):correct_terminal(observe,move,.53,prior)
            move.assert_not_called();observe.assert_not_called()


class PublicationLeaseTests(unittest.TestCase):
    def setUp(self):
        fixture=fixtures.Tests();fixture.setUp()
        self.snapshot=fixture.snapshot;self.observation=fixture.observation
        self.observation['identity_tracking']['measurement']['point_odom_m']=[.7157,.0584,.54]
        self.kw=dict(now_ns=1_200_000_000,epoch='epoch',task_id='task',current_odom=[0.,0.,0.],
                     translation=[.7157-CONTACT_XY[0],.0584-CONTACT_XY[1]],palm_height=.53,
                     previous=previous(),recording='/CURRENT-RUN',expected_room='living_room')

    def check(self):return validate_terminal_correction(self.observation,self.snapshot,**self.kw)

    def test_new_measured_correction_is_not_closure_authorization(self):
        self.assertFalse(self.check()['motion_authorized'])
        self.assertTrue(self.check()['terminal_correction'])

    def test_old_frame_from_before_motion_is_rejected(self):
        self.kw['previous']['motion_completed_at_ns']=1_000_000_000
        with self.assertRaisesRegex(ValueError,'post_approach_observation_required'):self.check()

    def test_wrong_task_and_post_motion_base_shift_rejected(self):
        for change in ('task','base'):
            with self.subTest(change=change):
                prior=copy.deepcopy(self.kw['previous'])
                if change=='task':prior['identity_context']['task_id']='different'
                else:prior['final_odom']=[.03,0,0]
                with patch.dict(self.kw,previous=prior),self.assertRaises(ValueError):self.check()

    def test_target_loss_and_stale_depth_never_allow_correction(self):
        self.observation['identity_tracking']['identity_confirmed']=False
        with self.assertRaisesRegex(ValueError,'identity_not_confirmed'):self.check()
        self.observation['identity_tracking']['identity_confirmed']=True;self.kw['now_ns']=2_000_000_000
        with self.assertRaisesRegex(ValueError,'expired'):self.check()

    def test_command_that_overshoots_or_moves_away_rejected(self):
        for delta in ([.10,0],[-.02,0],[0,0]):
            with patch.dict(self.kw,translation=delta),self.assertRaises(ValueError):self.check()

    def test_small_reverse_lateral_uses_fresh_geometry(self):
        self.observation['identity_tracking']['measurement']['point_odom_m']=[.68,.1,.536]
        self.kw['translation']=[.68-CONTACT_XY[0],.1-CONTACT_XY[1]]
        self.assertTrue(self.check()['terminal_correction'])


class EntryWiringTests(unittest.TestCase):
    def exercise(self,fail_correction=False):
        import run_bedroom_apple_regrasp as entry
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);recording=root/'recording';recording.mkdir();frame=recording/'frame';frame.mkdir()
            (frame/'observation.json').write_text('{}')
            calls=[];motion_count=[0];closure=Mock()
            def latest(_):
                stamp=time.time_ns()
                snapshot=dict(streams={key:dict(stamp_ns=stamp) for key in ('rgb','depth')},tracking_transforms={key:dict(
                    translation=[0,0,0],quaternion=[0,0,0,1]) for key in ('odom_from_base_at_depth','map_from_base_at_depth')})
                (frame/'snapshot.json').write_text(json.dumps(snapshot))
                return frame/'observation.json'
            def measured(*args,**kwargs):
                target=([1.19,.013,.54] if motion_count[0]==0 else
                        [.7157,.0584,.54] if motion_count[0]==1 else [.691,.079,.54])
                return dict(target_base_m=target)
            def execute(command,**kwargs):
                calls.append(command);output=Path(command[command.index('--output')+1])
                if Path(command[1]).name=='reviewed_carry_adjustment.py':
                    motion_count[0]+=1;value=previous(time.time_ns())
                    if '--grasp-terminal-correction' in command:
                        value.update(single_grasp_approach=False,grasp_terminal_correction=True)
                        if fail_correction:value['status']='failed'
                    output.write_text(json.dumps(value))
                else:
                    output.mkdir();(output/'result.json').write_text(json.dumps(dict(effective_grasp_success_verified=True)))
            argv=['test','--run','--output',str(root/'trial'),'--model-python','unused','--registry','unused',
                  '--environment','unused','--observer-recording',str(recording),'--observer-epoch','new',
                  '--task-id','new','--efficient-transfer','--verify-preclose-rgbd-follow','--contact-start-lift','.255']
            with patch.object(entry.sys,'argv',argv),patch.dict(entry.os.environ,ROS_DOMAIN_ID='71'),\
                 patch.object(entry,'latest',side_effect=latest),patch.object(entry,'measured_target',side_effect=measured),\
                 patch.object(entry.subprocess,'run',side_effect=execute),\
                 patch('apple_approach_lease.validate_closure',closure),patch('builtins.print'):
                if fail_correction:
                    with self.assertRaises(SystemExit):entry.main()
                else:entry.main()
            return calls,closure,read_json(root/'trial/result.json')

    def test_real_entry_executes_coarse_correction_then_close_in_order(self):
        calls,closure,result=self.exercise()
        self.assertEqual(len(calls),3)
        self.assertIn('--single-grasp-approach',calls[0])
        self.assertIn('--grasp-terminal-correction',calls[1])
        self.assertIn('--correction-after',calls[1])
        self.assertNotIn('--single-grasp-approach',calls[1])
        self.assertEqual(Path(calls[2][1]).name,'run_apple_close_lift_trial.py')
        closure.assert_called_once()
        self.assertTrue(result['grasp_success_verified'])

    def test_failed_correction_never_repeats_or_closes_in_real_entry(self):
        calls,closure,result=self.exercise(fail_correction=True)
        self.assertEqual(len(calls),2);closure.assert_not_called()
        self.assertFalse(result['grasp_success_verified'])


def read_json(path):return json.loads(path.read_text())


if __name__=='__main__':unittest.main()
