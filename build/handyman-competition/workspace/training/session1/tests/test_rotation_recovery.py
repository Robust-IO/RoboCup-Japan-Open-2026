import math,unittest
import numpy as np
from grasp_runtime.autograsp.rotation_recovery import choose,hand_support,angle_distance

class RotationRecoveryTests(unittest.TestCase):
    def candidate(self,angle,score=2):
        return dict(id=str(angle),strategy='overhead',wrist_roll_rad=angle,score=score,
                    observed_width_m=.04,grasp_point_base_m=[0,0,1])
    def test_changes_angle_instead_of_repeat_translation(self):
        old=self.candidate(0,3);new=self.candidate(math.pi/2,2)
        d=choose(dict(selected=old,candidates=[old,new]),old)
        self.assertEqual(d['selected']['id'],new['id'])
        self.assertTrue(d['recovery_selection']['used_alternative_angle'])
    def test_no_new_candidate_is_invented(self):
        d={'selected':None,'candidates':[]}
        self.assertEqual(choose(d,self.candidate(0)),d)
    def test_parallel_jaw_half_turn_is_same_angle(self):
        self.assertAlmostEqual(angle_distance(0,math.pi),0)
    def hint(self):
        return dict(camera_world=np.zeros(3),rotation_world=np.eye(3),raster_to_frame=np.eye(3),
            k=np.array([[10,0,10],[0,10,10],[0,0,1]]),rgb=np.full((21,21,3),100,dtype=np.uint8),
            reference_rgb=np.full((20,3),100))
    def test_matching_hand_patch_is_only_advisory(self):
        self.assertEqual(hand_support([0,0,1],self.hint()),1.)
        self.assertEqual(hand_support([0,0,-1],self.hint()),0.)
        self.assertEqual(hand_support([20,0,1],self.hint()),0.)
    def test_camera_motion_transform_applied(self):
        h=self.hint();h['camera_world']=np.array([3,2,0])
        self.assertEqual(hand_support([3,2,1],h),1.)
    def test_mismatched_or_missing_rgb_does_not_block_geometry(self):
        old=self.candidate(0);new=self.candidate(.7)
        h=self.hint();h['rgb'][:]=255
        d=choose(dict(selected=old,candidates=[old,new]),old,hint=h)
        self.assertEqual(d['selected']['id'],new['id'])
        self.assertEqual(d['recovery_selection']['hand_rgb_support'],0.)
