"""Deleted demo/redundant gates must not remove actual identity/release evidence."""
import ast
import copy
import json
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from apple_approach_lease import validate_closure
from apple_identity_track import AppleIdentityTrack
from apple_inference_worker import measured_proposal
from apple_transfer_auto_plan import (RELEASE_XY_TOLERANCE_M, plan_grasp,
                                     plan_placement, placement_step, validate_config)
import test_apple_approach_lease as lease_fixtures
import test_apple_transfer_automation as transfer_fixtures
import test_tabletop_support as support_fixtures
import test_verify_table_placement as placement_fixtures
from verify_table_placement import verify

ROOT = Path(__file__).parent


class SupportSeparationTest(unittest.TestCase):
    def test_identity_can_seed_and_continue_with_fresh_geometry_without_contact_claim(self):
        track=AppleIdentityTrack('task','epoch')
        for index in range(4):
            stamp=1_000_000_000+index*100_000_000
            item=dict(point_odom_m=[.7,.078,.54],radius_m=.03,confidence=.95,
                fit_residual_p95_m=.006,measurement='fresh_rgbd',stamp_ns=stamp,
                class_name='apple',red_appearance=True,clipped=False,depth_valid=True,
                support_consistent=False)
            result=track.update([item],stamp_ns=stamp,now_ns=stamp+100_000_000,
                task_id='task',epoch='epoch',room_confirmed=True)
        self.assertTrue(result['identity_confirmed'])
        self.assertFalse(result['measurement']['support_consistent'])
        self.assertFalse(result['motion_authorized'])

    def test_source_plan_still_requires_support_even_with_confirmed_identity(self):
        args=transfer_fixtures.PlanningTest().grasp_fixture()
        args[0]['tracking_proposals'][0]['support']['support_consistent']=False
        with self.assertRaisesRegex(ValueError,'same_frame_measured_support_required'):
            plan_grasp(*args)

    def test_source_plan_still_rejects_wrong_table_and_invalid_height(self):
        args=transfer_fixtures.PlanningTest().grasp_fixture();args[3]['target_base_m'][1]+=2
        with self.assertRaisesRegex(ValueError,'apple_not_on_requested_source_table'):plan_grasp(*args)
        args=transfer_fixtures.PlanningTest().grasp_fixture();args[0]['tracking_proposals'][0]['support']['support_top_midpoint_z_m']=.9
        with self.assertRaisesRegex(ValueError,'measured_grasp_height'):plan_grasp(*args)

    def test_support_failure_does_not_discard_measured_object_geometry(self):
        candidate=dict(center_m=[.7,.078,.54]);detection=dict(name='apple')
        with patch('apple_inference_worker.estimate',return_value=candidate), \
                patch('apple_inference_worker.check_support',side_effect=ValueError('target_outside_support_hull')):
            result=measured_proposal(None,None,None,detection)
        self.assertEqual(result['candidate'],candidate)
        self.assertNotIn('rejected_reason',result)
        self.assertFalse(result['support']['support_consistent'])
        self.assertEqual(result['support']['reason'],'target_outside_support_hull')

    def test_failed_rgbd_geometry_is_still_rejected_not_resurrected(self):
        with patch('apple_inference_worker.estimate',side_effect=ValueError('insufficient_target_depth')):
            result=measured_proposal(None,None,None,dict(name='apple'))
        self.assertEqual(result['rejected_reason'],'insufficient_target_depth')
        self.assertNotIn('candidate',result)

    def test_close_lease_keeps_fresh_geometry_without_requiring_sphere_bottom_contact(self):
        fixture=lease_fixtures.Tests();fixture.setUp()
        m=fixture.observation['identity_tracking']['measurement']
        m.update(point_odom_m=[.703,.078,.54],support_consistent=False)
        kwargs={k:v for k,v in fixture.kw.items() if k!='translation'}
        self.assertFalse(validate_closure(fixture.observation,fixture.snapshot,**kwargs)['grasp_center_verified'])
        kwargs['now_ns']+=1_000_000_000
        with self.assertRaises(ValueError):validate_closure(fixture.observation,fixture.snapshot,**kwargs)


class DemoConstraintTest(unittest.TestCase):
    def test_staging_coordinates_are_configuration_not_exact_historical_constants(self):
        config=json.loads((ROOT/'config/apple_transfer_auto_v1.json').read_text())
        config['source_staging_goal']=[9.95,-6.65,.03]
        config['destination_staging_goal']=[2.1,2.6,math.pi/2+.03]
        validate_config(config)
        for bad in ([1,2], [1,2,3,4], [1,2,float('nan')]):
            changed=copy.deepcopy(config);changed['source_staging_goal']=bad
            with self.subTest(bad=bad),self.assertRaises(ValueError):validate_config(changed)

    def test_table_uses_current_export_and_support_not_old_global_template(self):
        points,args=support_fixtures.PlacementSelectionTest().fixture()
        offset=[1.,.5]
        points[:,:2]+=offset
        args[1]['tracking_transforms']['map_from_base_at_depth']['translation'][0]+=offset[0]
        args[1]['tracking_transforms']['map_from_base_at_depth']['translation'][1]+=offset[1]
        bounds=args[2]['axis_aligned_bounds']
        bounds['center']['x']+=offset[0];bounds['center']['y']+=offset[1]
        bounds['size']['x']=.84;bounds['size']['y']=1.40
        with patch('apple_transfer_auto_plan.depth_in_map',return_value=points):
            result=plan_placement(*args)
        self.assertTrue(result['table_edge_safety']['safe'])
        self.assertLessEqual(result['planned_approach_distance_m'],.6)

    def workspace_guard(self,**changes):
        main=next(x for x in ast.parse((ROOT/'pregrasp_raised_trial.py').read_text()).body
                  if isinstance(x,ast.FunctionDef) and x.name=='main')
        guard=next(x for x in main.body if isinstance(x,ast.If)
                   and ast.unparse(x.test)=='args.simulation_contact_trial')
        args=SimpleNamespace(simulation_contact_trial=True,segmented_high=True,retract_high=False,
            reviewed_map_pose=[20.,-20.,2.],inverse_place_stage=None,
            bedroom_side_table_grasp=False,bedroom_side_table_place=False,place_stage=None,
            tucked_camera_check=None,prepare_empty_horizontal=False,horizontal_observe=None)
        for key,value in changes.items():setattr(args,key,value)
        tree=ast.fix_missing_locations(ast.Module(body=[guard],type_ignores=[]))
        exec(compile(tree,'original_workspace_guard','exec'),dict(args=args,math=math,
            os=SimpleNamespace(environ=dict(ROS_DOMAIN_ID='71'))))

    def test_measured_grasp_and_inverse_place_have_no_absolute_rectangle(self):
        self.workspace_guard(bedroom_side_table_grasp=True)
        self.workspace_guard(inverse_place_stage='prepare')

    def test_workspace_change_does_not_remove_explicit_run_and_finite_pose_requirements(self):
        for changes in (dict(segmented_high=False),dict(reviewed_map_pose=None),
                        dict(reviewed_map_pose=[float('nan'),0,0])):
            with self.subTest(changes=changes),self.assertRaises(ValueError):
                self.workspace_guard(bedroom_side_table_grasp=True,**changes)

    def test_legacy_unmeasured_experiment_keeps_its_original_fence(self):
        with self.assertRaises(ValueError):self.workspace_guard(bedroom_side_table_place=True)


class DuplicatePlacementGateTest(unittest.TestCase):
    def test_shared_release_alignment_replaces_stricter_six_mm_veto(self):
        self.assertEqual(RELEASE_XY_TOLERANCE_M,.012)
        snapshot=dict(streams=dict(depth=dict(stamp_ns=1)),tracking_transforms=dict(
            map_from_base_at_depth=dict(stamp_ns=1,source_frame='base_link',target_frame='map',
                translation=[0,0,0],quaternion=[0,0,0,1])))
        measured=dict(center_base=[.7,.078,.5])
        self.assertTrue(placement_step(measured,snapshot,dict(release_xy_map_m=[.708,.078]),single_move=True)['aligned'])
        self.assertFalse(placement_step(measured,snapshot,dict(release_xy_map_m=[.713,.078]),single_move=True)['aligned'])

    def test_actual_table_support_replaces_aabb_and_camera_standoff(self):
        audit,prior,actions,destination,room,plan=placement_fixtures.PlacementTest().edge_fixture(3.76)
        for frame in audit['frames']:
            candidate=frame['apple_measurements'][0]
            candidate['inside_table_with_apple_margin']=False
            candidate['candidate']['center_m'][0]=.4
        result=verify(audit,prior,actions,destination,room,placement_plan=plan)
        self.assertTrue(result['placement_success_verified'])
        self.assertTrue(result['table_edge_safety_verified'])

    def test_observed_edge_still_rejects_even_if_old_aabb_is_safe(self):
        audit,prior,actions,destination,room,plan=placement_fixtures.PlacementTest().edge_fixture(3.644)
        self.assertFalse(verify(audit,prior,actions,destination,room,placement_plan=plan)['placement_success_verified'])

    def test_legacy_without_observed_support_does_not_gain_removed_gate_exemption(self):
        audit,prior,actions=placement_fixtures.PlacementTest().fixture()
        audit['frames'][0]['apple_measurements'][0]['inside_table_with_apple_margin']=False
        self.assertFalse(verify(audit,prior,actions)['placement_success_verified'])

    def test_redundant_parent_timeouts_removed_but_child_waits_remain(self):
        tree=ast.parse((ROOT/'run_bedroom_apple_regrasp.py').read_text())
        calls=[x for x in ast.walk(tree) if isinstance(x,ast.Call) and ast.unparse(x.func)=='subprocess.run']
        deadlines=[kw.value.value for call in calls for kw in call.keywords
                   if kw.arg=='timeout' and isinstance(kw.value,ast.Constant)]
        self.assertNotIn(65,deadlines);self.assertIn(35,deadlines)
        close_source=(ROOT/'run_apple_close_lift_trial.py').read_text()
        self.assertEqual(close_source.count('timeout=40'),2)


if __name__=='__main__':unittest.main()
