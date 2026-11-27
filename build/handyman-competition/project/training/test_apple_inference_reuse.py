import copy
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch
import numpy as np
from apple_inference_worker import measured_proposal,confirmed_geometry,predict_full_frame
from apple_rgbd_candidate import estimate


def sphere_fixture():
    v,u=np.indices((120,120));x=(u-60)/200;y=(v-60)/200
    a=x*x+y*y+1;disc=1-4*a*(.25-.04**2);hit=disc>0
    depth=np.zeros((120,120));depth[hit]=(1-np.sqrt(disc[hit]))/(2*a[hit])
    rgb=np.zeros((120,120,3),np.uint8);rgb[hit]=[230,30,50]
    snapshot=dict(streams={'rgb':{'stamp_ns':1000000000},'depth':{'stamp_ns':1000000000}},
        rgb_k=[200,0,60,0,200,60,0,0,1],
        base_from_rgb={'translation':[0,0,0],'quaternion':[0,0,0,1]})
    return SimpleNamespace(rgb=rgb,depth=depth,s=snapshot,d=dict(name='apple',confidence=.9,box=[43,43,77,77]))


class ReuseTests(unittest.TestCase):
    def test_same_current_geometry_equals_old_second_fit_without_running_it(self):
        fixture=sphere_fixture()
        proposal=measured_proposal(fixture.rgb,fixture.depth,fixture.s,fixture.d)
        expected=estimate(fixture.rgb,fixture.depth,fixture.s,fixture.d)
        before=copy.deepcopy(proposal)
        with (patch('apple_inference_worker.estimate',side_effect=AssertionError('duplicate_fit')),
                patch('apple_inference_worker.check_support',side_effect=AssertionError('duplicate_support'))):
            candidate,support=confirmed_geometry([proposal],fixture.d)
        self.assertEqual(candidate,expected);self.assertEqual(support,proposal['support'])
        self.assertFalse(candidate['identity_confirmed']);self.assertFalse(candidate['ready_for_motion'])
        self.assertEqual(proposal,before)

    def test_shared_fit_failure_cannot_be_marked_a_candidate(self):
        detection=dict(name='apple',confidence=.95)
        with self.assertRaisesRegex(ValueError,'insufficient_target_depth'):
            confirmed_geometry([dict(detection=detection,rejected_reason='insufficient_target_depth')],detection)

    def test_geometry_must_belong_to_the_same_detection_not_another_equal_copy(self):
        detection=dict(name='apple',confidence=.95)
        with self.assertRaisesRegex(ValueError,'same_detection_geometry_required'):
            confirmed_geometry([dict(detection=copy.deepcopy(detection),candidate={},support={})],detection)

    def test_low_confidence_or_wrong_class_does_not_inherit_candidate_status(self):
        for name,confidence in (('apple',.79),('toy_car',.99)):
            detection=dict(name=name,confidence=confidence)
            with self.assertRaises(ValueError):confirmed_geometry([dict(detection=detection,candidate={},support={})],detection)

    def test_array_prediction_uses_identical_bgr_pixels_not_an_extra_file_decode(self):
        rgb=np.arange(3*12*15,dtype=np.uint8).reshape(12,15,3);before=rgb.copy()
        model=Mock();model.predict.return_value=['prediction']
        self.assertEqual(predict_full_frame(model,rgb,960,'cuda:0'),'prediction')
        arg=model.predict.call_args.args[0]
        np.testing.assert_array_equal(arg,rgb[:,:,::-1]);np.testing.assert_array_equal(rgb,before)
        self.assertTrue(arg.flags.c_contiguous);self.assertEqual(model.predict.call_args.kwargs['conf'],.1)


if __name__=='__main__':unittest.main()
