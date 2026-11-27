import copy,unittest
import numpy as np
from scipy.spatial.transform import Rotation
from table_localization import config,anchor_from_pose,compose,pose,decide,stationary

class TableLocalizationTest(unittest.TestCase):
    def rows(self,delta=(.3,-.35,.06)):
        rows=[]
        for i in range(3):
            odom=dict(translation=[8.5+i*.0001,-6.,0.],quaternion=Rotation.from_euler('z',-.07).as_quat().tolist())
            current=[9.8+i*.0001,-6.9,-.07];matched=(np.asarray(current)+delta).tolist()
            rows.append(dict(stamp_ns=100+i,current_pose=current,current_score=.28 if delta[0] else .002,current_inlier_fraction=.1 if delta[0] else .96,
                matched_pose=matched,matched_score=.002,matched_inlier_fraction=.96,odom_from_base=odom,anchor=anchor_from_pose(matched,odom)))
        return rows
    def test_anchor_composes_back_to_matched_base(self):
        for row in self.rows():np.testing.assert_allclose(pose(compose(row['anchor'],row['odom_from_base'])),row['matched_pose'],atol=1e-10)
    def test_three_consistent_matches_request_correction(self):self.assertTrue(decide(self.rows(),config())['apply_correction'])
    def test_aligned_pose_does_not_trigger_repeated_corrections(self):self.assertFalse(decide(self.rows((0,0,0)),config())['apply_correction'])
    def test_one_unreliable_scan_cannot_change_map_transform(self):
        rows=self.rows();rows[1]['matched_inlier_fraction']=.4
        with self.assertRaisesRegex(ValueError,'not_reliable'):decide(rows,config())
    def test_repeated_scan_does_not_count_as_three_observations(self):
        rows=self.rows();rows[2]['stamp_ns']=rows[1]['stamp_ns']
        with self.assertRaisesRegex(ValueError,'distinct'):decide(rows,config())
    def test_disagreement_cannot_change_map_transform(self):
        rows=self.rows();rows[1]['anchor']['translation'][0]+=.15
        with self.assertRaisesRegex(ValueError,'disagree'):decide(rows,config())
    def test_robot_motion_invalidates_scan_computation(self):
        original=self.rows()[0]['odom_from_base'];current=copy.deepcopy(original);self.assertTrue(stationary(original,current,config()))
        current['translation'][0]+=.02;self.assertFalse(stationary(original,current,config()))
        current=copy.deepcopy(original);current['quaternion']=Rotation.from_euler('z',.1).as_quat().tolist();self.assertFalse(stationary(original,current,config()))
    def test_both_grasp_and_place_localize_before_geometric_actions(self):
        from pathlib import Path
        grasp=Path(__file__).with_name('flat_box_grasp.py').read_text();place=Path(__file__).with_name('flat_box_place.py').read_text()
        self.assertLess(grasp.index("localize(runtime,root,'source')"),grasp.index("runtime.move_joints(empty"))
        self.assertLess(place.index("localize(runtime,root,'destination')"),place.index("plan=plan_placement"))
    def test_source_nominal_center_path_can_be_diagnostic_without_removing_motion_check(self):
        from unittest.mock import patch
        from flat_box_geometry import select_source
        from test_flat_box_place_geometry import fixture
        s,_,_,_=fixture();table=dict(axis_aligned_bounds=dict(center=dict(x=3.,y=0.),size=dict(x=1.,y=1.)))
        model=dict(center_m=[3.3,0,.5],axis_base=[1,0,0],length_m=.084,width_m=.05,height_m=.022,confidence=.9)
        with patch('flat_box_geometry.estimate',return_value=model):
            with self.assertRaisesRegex(ValueError,'base_inside_table'):select_source(None,None,s,[dict(name='cigarette')],table)
            selected,_=select_source(None,None,s,[dict(name='cigarette')],table,check_nominal_reach=False);self.assertEqual(selected['center_m'],model['center_m'])
        text=__import__('pathlib').Path(__file__).with_name('flat_box_grasp.py').read_text()
        self.assertIn("source_target_not_reachable_without_base_entering_table",text)
if __name__=='__main__':unittest.main()
