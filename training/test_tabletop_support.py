import copy
import json
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

from apple_transfer_auto_plan import plan_placement
from tabletop_support import (EDGE_GAP_M, EDGE_UNCERTAINTY_M, TableSupport,
                              build_support, support_from_plan)


def support_fixture(front=3.60):
    x, y = np.meshgrid(np.arange(1.64, 2.30, .0025), np.arange(front, 4.59, .0025))
    points = np.c_[x.ravel(), y.ravel(), np.full(x.size, .444)]
    model = build_support(points, [1.971, 4.024], [.36, .60], [0, 0, .444], 'table', 1)
    return points, model


def edge_plan(model, destination='table'):
    model = copy.deepcopy(model)
    model['destination_id'] = destination
    return dict(destination_id=destination, observed_table_support=model,
                source_depth_stamp_ns=model['source_stamp_ns'],
                table_edge_policy=dict(version=1, minimum_outer_gap_m=EDGE_GAP_M,
                    uncertainty_allowance_m=EDGE_UNCERTAINTY_M,
                    selection='shortest_reachable_approach_after_clearance_passes'))


class TableSupportTest(unittest.TestCase):
    def test_wider_penguin_support_keeps_the_same_actual_edge_margin(self):
        support=TableSupport(support_fixture()[1],'table')
        small=support.safe_centers(.06);wide=support.safe_centers(.071)
        self.assertGreater(len(wide),0);self.assertLess(len(wide),len(small))
        check=support.query(wide[0],.071)
        self.assertTrue(check['safe']);self.assertGreaterEqual(check['outer_clearance_lower_estimate_m'],.04)
        self.assertEqual(check['minimum_outer_gap_m'],.03)
        with self.assertRaises(ValueError):support.query(wide[0],.091)
    def setUp(self):
        self.points, self.model = support_fixture()
        self.support = TableSupport(self.model, 'table')

    def test_aabb_inset_does_not_establish_actual_edge_margin(self):
        self.assertFalse(self.support.query([1.971, 3.65], .032)['safe'])
        self.assertTrue(self.support.query([1.971, 3.75], .032)['safe'])

    def test_not_only_center_but_object_radius_and_uncertainty_matter(self):
        small = self.support.query([1.971, 3.69], .02)
        large = self.support.query([1.971, 3.69], .06)
        self.assertTrue(small['safe'])
        self.assertFalse(large['safe'])
        self.assertAlmostEqual(small['minimum_outer_gap_m'], .03)
        self.assertAlmostEqual(small['uncertainty_allowance_m'], .01)

    def test_new_four_cm_policy_is_not_old_seven_cm_policy_or_zero_margin(self):
        newly_safe=self.support.query([1.971,3.70],.032)
        self.assertTrue(newly_safe['safe'])
        self.assertGreaterEqual(newly_safe['outer_clearance_lower_estimate_m'],.04)
        self.assertLess(newly_safe['outer_clearance_lower_estimate_m'],.07)
        self.assertFalse(self.support.query([1.971,3.66],.032)['safe'])

    def test_historical_seven_cm_plan_requires_new_planning_not_silent_reinterpretation(self):
        plan=edge_plan(self.model)
        plan['table_edge_policy'].update(minimum_outer_gap_m=.05,uncertainty_allowance_m=.02)
        with self.assertRaisesRegex(ValueError,'replan_legacy_placement'):support_from_plan(plan)

    def test_unknown_outside_and_oversized_apple_rejected(self):
        self.assertFalse(self.support.query([10, 10], .03)['safe'])
        with self.assertRaises(ValueError):
            self.support.query([1.97, 3.9], .2)

    def test_occluded_hole_is_not_bridged_by_rectangle_or_convex_hull(self):
        hole = np.linalg.norm(self.points[:, :2] - [1.97, 3.95], axis=1) > .04
        model = build_support(self.points[hole], [1.971, 4.024], [.36, .6], [0, 0, .444], 'table', 1)
        support = TableSupport(model, 'table')
        self.assertFalse(support.query([1.97, 3.95], .03)['safe'])
        self.assertFalse(support.query([1.97, 3.86], .03)['safe'])

    def test_dilation_and_quantisation_cannot_overstate_straight_edge(self):
        for y in np.arange(3.59, 3.79, .0013):
            result = self.support.query([1.971, y], .03)
            self.assertLessEqual(result['center_to_unsupported_m'], max(0, y - 3.60) + 1e-9)

    def test_legacy_wrong_destination_or_lowered_policy_cannot_release(self):
        with self.assertRaises(ValueError):
            support_from_plan(dict(destination_id='table'))
        plan = edge_plan(self.model)
        plan['table_edge_policy']['minimum_outer_gap_m'] = .001
        with self.assertRaises(ValueError):
            support_from_plan(plan)
        with self.assertRaises(ValueError):
            TableSupport(self.model, 'other')

    def test_malformed_mask_is_not_geometry(self):
        for rows in ([], ['111', '10', '111'], ['xxx'] * 3):
            model = copy.deepcopy(self.model)
            model['mask_rows'] = rows
            with self.assertRaises(ValueError):
                TableSupport(model, 'table')

    def test_support_must_come_from_the_planned_depth_capture(self):
        plan = edge_plan(self.model)
        plan['source_depth_stamp_ns'] += 1
        with self.assertRaises(ValueError):
            support_from_plan(plan)

    def test_no_depth_support_is_not_free_space(self):
        with self.assertRaises(ValueError):
            build_support(np.zeros((0, 3)), [1.971, 4.024], [.36, .6], [0, 0, .444], 'table', 1)


class PlacementSelectionTest(unittest.TestCase):
    def fixture(self):
        config = json.loads((Path(__file__).parent / 'config/apple_transfer_auto_v1.json').read_text())
        points, _ = support_fixture()
        snapshot = dict(streams=dict(rgb=dict(stamp_ns=1), depth=dict(stamp_ns=1)),
                        tracking_transforms=dict(map_from_base_at_depth=dict(stamp_ns=1,
                            source_frame='base_link', target_frame='map', translation=[2.1, 2.525, 0],
                            quaternion=[0, 0, np.sin(np.pi / 4), np.cos(np.pi / 4)])))
        table = dict(id=config['destination_id'], room='living_room',
                     axis_aligned_bounds=dict(center=dict(x=1.971, y=4.024), size=dict(x=.72, y=1.20)))
        plane = dict(plane_map=[0, 0, .444], destination_id=table['id'], source_depth_stamp_ns=1)
        held = dict(center_minus_palm_m=[.08, 0, .01], bottom_below_palm_m=.02,
                    radius_m=.032, holding_opening_rad=.395)
        return points, [None, snapshot, table, plane, held, config, 'task']

    def test_select_near_safe_edge_not_maximum_inset_or_table_center(self):
        points, args = self.fixture()
        with patch('apple_transfer_auto_plan.depth_in_map', return_value=points):
            plan = plan_placement(*args)
        self.assertTrue(plan['table_edge_safety']['safe'])
        self.assertGreater(plan['release_xy_map_m'][1], 3.69)
        self.assertLess(plan['release_xy_map_m'][1], 3.74)
        self.assertLess(plan['release_xy_map_m'][1], 4.024 - .25)
        self.assertLessEqual(plan['planned_approach_distance_m'], .6)
        self.assertEqual(plan['table_edge_policy']['selection'], 'shortest_reachable_approach_after_clearance_passes')

    def test_occupied_nearest_patch_chooses_next_clear_safe_patch(self):
        points, args = self.fixture()
        with patch('apple_transfer_auto_plan.depth_in_map', return_value=points), \
                patch('apple_transfer_auto_plan.clear_patch', side_effect=[dict(clear=False), dict(clear=True)]):
            plan = plan_placement(*args)
        self.assertEqual(len(plan['checked_candidates']), 2)
        self.assertTrue(plan['table_edge_safety']['safe'])

    def test_single_planner_accepts_safe_clear_patch_more_than_sixty_cm_away(self):
        points,args=self.fixture()
        args[1]['tracking_transforms']['map_from_base_at_depth']['translation'][1]=1.75
        with patch('apple_transfer_auto_plan.depth_in_map',return_value=points):
            plan=plan_placement(*args)
        self.assertGreater(plan['planned_approach_distance_m'],1.)
        self.assertTrue(plan['patch']['clear'])
        self.assertTrue(plan['table_edge_safety']['safe'])

    def test_planning_reserves_alignment_and_small_radius_changes(self):
        points, args = self.fixture()
        with patch('apple_transfer_auto_plan.depth_in_map', return_value=points):
            plan = plan_placement(*args)
        support = support_from_plan(plan)
        for angle in np.linspace(0, 2 * np.pi, 24):
            xy = np.array(plan['release_xy_map_m']) + .012 * np.array([np.cos(angle), np.sin(angle)])
            with self.subTest(angle=angle):
                self.assertTrue(support.query(xy, args[4]['radius_m'] + .005)['safe'])

    def test_no_safe_patch_does_not_fall_back_to_aabb(self):
        points, args = self.fixture()
        points = points[points[:, 1] < 3.69]  # Still too narrow under3cm+1cm.
        with patch('apple_transfer_auto_plan.depth_in_map', return_value=points), self.assertRaises(ValueError):
            plan_placement(*args)


if __name__ == '__main__':
    unittest.main()
