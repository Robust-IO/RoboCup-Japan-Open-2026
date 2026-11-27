import copy
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from PIL import Image

from table_release_gate import validate
from test_tabletop_support import edge_plan, support_fixture


class ReleaseGateTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.capture = Path(self.temp.name)
        self.plan = dict(task_id='task', destination_id='square_low_table#living_room', release_xy_map_m=[2.022,3.604],
                         palm_height_m=.486, holding_opening_rad=.415, held_center_minus_palm_m=[.077,0,.008],
                         footprint_center=[1.971,4.024], footprint_half_size=[.36,.60], plane_map=[0,0,.444])
        _, model = support_fixture(front=3.45)
        self.plan.update(edge_plan(model, self.plan['destination_id']))
        self.joints = [.486-.2771978151685961, -1.7, 0, .1292036732051034, 0, .415]
        names = ['arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint']
        self.snapshot = dict(tracking_epoch='epoch', streams=dict(rgb=dict(stamp_ns=1_000_000_000), depth=dict(stamp_ns=1_000_000_000)),
                             joint_stamp_ns=1_000_000_000, joints=dict(zip(names,self.joints)),
                             tracking_transforms=dict(map_from_base_at_depth=dict(stamp_ns=1_000_000_000,
                             translation=[2.1,2.9,0], quaternion=[0,0,math.sin(math.pi/4),math.cos(math.pi/4)], source_frame='base_link', target_frame='map')),
                             base_from_hand=dict(translation=[.627,.078,.486],quaternion=[0,0,0,1]),
                             palm_from_hand=dict(translation=[0,0,0],quaternion=[0,0,0,1]))
        Image.new('RGB',(10,10)).save(self.capture/'rgb.png')
        np.save(self.capture/'depth_m.npy',np.ones((10,10)))
        self.measurement = dict(center_base=[.704,.078,.494],radius_m=.03)
        self.kw = dict(now_ns=1_100_000_000, epoch='epoch', task_id='task', current_map_pose=[2.1,2.9,math.pi/2], current_joints=self.joints)
    def tearDown(self):
        self.temp.cleanup()
    def check(self):
        (self.capture/'snapshot.json').write_text(json.dumps(self.snapshot))
        with patch('table_release_gate.measure',return_value=self.measurement):
            return validate(self.capture,self.plan,**self.kw)
    def test_fresh_verified_held_apple_can_release(self):
        result = self.check()
        self.assertAlmostEqual(result['estimated_bottom_gap_m'],.02)
        self.assertFalse(result['motion_authorized'])
    def test_old_image_cannot_release(self):
        self.kw['now_ns'] += 500_000_000
        with self.assertRaises(ValueError): self.check()
    def test_wrong_task_epoch_or_destination_cannot_release(self):
        for field,value in [('task_id','other'),('epoch','other')]:
            previous = self.kw[field]
            self.kw[field] = value
            with self.subTest(field=field),self.assertRaises(ValueError): self.check()
            self.kw[field] = previous
        self.plan['destination_id'] = 'wrong_table'
        with self.assertRaises(ValueError): self.check()
    def test_base_or_hand_changed_cannot_release(self):
        self.kw['current_map_pose'][0] += .03
        with self.assertRaises(ValueError): self.check()
        self.kw['current_map_pose'][0] -= .03
        self.kw['current_joints'] = self.joints[:5]+[.65]
        with self.assertRaises(ValueError): self.check()
    def test_safe_actual_table_region_need_not_match_planned_point(self):
        self.measurement['center_base'][0] += .05
        result=self.check()
        self.assertTrue(result['ready'])
        self.assertGreater(result['planned_xy_error_m'],.049)
        self.assertFalse(result['planned_xy_is_success_gate'])
    def test_release_too_high_cannot_release(self):
        self.measurement['center_base'][2] += .06
        with self.assertRaises(ValueError): self.check()
    def test_furniture_aabb_does_not_override_safe_measured_table_support(self):
        self.plan['footprint_half_size'][0] = .02
        self.assertTrue(self.check()['table_edge_safety']['safe'])
    def test_safe_inside_aabb_but_near_observed_edge_cannot_release(self):
        _, model = support_fixture(front=3.58)
        self.plan.update(edge_plan(model, self.plan['destination_id']))
        with self.assertRaisesRegex(ValueError, 'observed_table_edge'): self.check()
    def test_legacy_aabb_only_plan_requires_replanning(self):
        self.plan.pop('observed_table_support')
        with self.assertRaises(ValueError): self.check()


if __name__ == '__main__': unittest.main()
