import copy
import unittest
from can_identity_selection import confirmed_cans


class CanIdentitySelectionTests(unittest.TestCase):
    def test_same_object_duplicate_keeps_original_highest_score(self):
        a=dict(name='canned_juice',confidence=.94,box=[10,20,50,90])
        b=dict(name='canned_juice',confidence=.83,box=[10.2,20.1,50.1,90.2])
        self.assertEqual(confirmed_cans([b,a]),[a])
    def test_separate_high_score_objects_remain_ambiguous(self):
        ds=[dict(name='canned_juice',confidence=.94,box=[10,20,50,90]),dict(name='canned_juice',confidence=.9,box=[90,20,130,90])]
        self.assertEqual(len(confirmed_cans(ds)),2)
    def test_low_scores_are_not_added_together_or_promoted(self):
        ds=[dict(name='canned_juice',confidence=.79,box=[10,20,50,90])]*4
        self.assertEqual(confirmed_cans(ds),[])
    def test_other_classes_and_invalid_boxes_cannot_become_can(self):
        ds=[dict(name='filled_plastic_bottle',confidence=.99,box=[10,20,50,90]),
            dict(name='canned_juice',confidence=.99,box=[50,20,10,90])]
        self.assertEqual(confirmed_cans(ds),[])
    def test_input_detector_output_is_preserved(self):
        ds=[dict(name='canned_juice',confidence=.94,box=[10,20,50,90])];before=copy.deepcopy(ds)
        confirmed_cans(ds);self.assertEqual(ds,before)


if __name__=='__main__':unittest.main()
