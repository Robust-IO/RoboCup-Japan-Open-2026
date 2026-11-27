import unittest,numpy as np,json
from thin_box_clearance import longitudinal_overlap,closure_region
class FingerOverlapTest(unittest.TestCase):
    def regions(self):
        return [dict(vertices_palm_m=[[x,y,z] for x in (.05,.075) for y in (side-.001,side+.001) for z in (-.0175,.0175)]) for side in (-.04,.04)]
    def test_box_need_not_match_pad_center_when_both_sides_have_contact_length(self):
        proof=longitudinal_overlap([.105,0,.51],[1,0,0],.084,.05,[0,0,.53],np.eye(3),self.regions())
        self.assertTrue(proof['ready']);self.assertGreater(min(proof['overlap_m']),.01)
        self.assertTrue(json.loads(json.dumps(proof))['ready'])
    def test_tips_short_of_box_do_not_authorize_closure(self):
        self.assertFalse(longitudinal_overlap([.14,0,.51],[1,0,0],.084,.05,[0,0,.53],np.eye(3),self.regions())['ready'])
    def test_only_one_pad_reaching_is_insufficient(self):
        regions=self.regions();regions[1]['vertices_palm_m']=[[x-.06,y,z] for x,y,z in regions[1]['vertices_palm_m']]
        self.assertFalse(longitudinal_overlap([.105,0,.51],[1,0,0],.084,.05,[0,0,.53],np.eye(3),regions)['ready'])

class ClosureRegionTest(unittest.TestCase):
    def scene(self):
        r=np.array([[0,0,1],[0,-1,0],[1,0,0]],float)
        pads=[dict(vertices_palm_m=(np.array([[x,y,z] for x in (.05,.075) for y in (side-.001,side+.001) for z in (-.0175,.0175)])@r).tolist()) for side in (-.0475,.0475)]
        return dict(model=dict(length_m=.084,width_m=.05,height_m=.022),palm=[0,0,0],orientation=r,regions=pads,axis=[1,0,0])
    def check(self,center,**kwargs):return closure_region(center,**dict(self.scene(),**kwargs))
    def test_lateral_offset_over_eight_mm_inside_open_gap_can_close(self):
        p=self.check([.07,.015,0]);self.assertTrue(p['ready'])
        self.assertGreater(min(p['open_side_clearance_m']),.001)
        self.assertTrue(json.loads(json.dumps(p))['ready'])
    def test_forward_offset_over_fifteen_mm_can_close_with_contact(self):
        self.assertTrue(self.check([.105,0,0])['ready'])
    def test_height_need_not_match_nominal_grasp_point(self):
        self.assertTrue(self.check([.07,0,-.024])['ready'])
    def test_inside_open_gap_alone_is_not_enough_if_tips_do_not_reach(self):
        self.assertFalse(self.check([.15,0,0])['ready'])
    def test_object_outside_open_gap_cannot_close(self):
        self.assertFalse(self.check([.07,.03,0])['ready'])
    def test_no_vertical_side_contact_cannot_close(self):
        self.assertFalse(self.check([.07,0,.04])['ready'])
    def test_only_one_finger_has_height_contact_cannot_close(self):
        scene=self.scene();scene['regions'][1]['vertices_palm_m']=(np.asarray(scene['regions'][1]['vertices_palm_m'])+np.array([.08,0,0])).tolist()
        self.assertFalse(closure_region([.07,0,0],**scene)['ready'])
    def test_box_yaw_uses_actual_projected_width(self):
        self.assertTrue(self.check([.07,0,0],axis=[0,1,0])['ready'])
        self.assertFalse(self.check([.07,.01,0],axis=[0,1,0])['ready'])
    def test_robot_yaw_and_translation_do_not_change_decision(self):
        s=self.scene();c=np.array([.105,.015,-.024]);original=closure_region(c,**s)
        t=.6;r=np.array([[np.cos(t),-np.sin(t),0],[np.sin(t),np.cos(t),0],[0,0,1]]);p=np.array([2.,3.,.5])
        s.update(palm=p,axis=r@np.array(s['axis']),orientation=r@s['orientation'])
        result=closure_region(r@c+p,**s);self.assertTrue(result['ready']);self.assertEqual(original['ready'],result['ready'])
        np.testing.assert_allclose(original['open_side_clearance_m'],result['open_side_clearance_m'],atol=1e-12)
    def test_missing_geometry_or_nonfinite_input_cannot_close(self):
        for s in (dict(regions=[]),dict(center=[float('nan'),0,0])):
            args=dict(self.scene(),center=[.07,0,0]);args.update(s)
            with self.assertRaises(ValueError):closure_region(**args)
    def test_runtime_uses_region_even_when_nominal_error_disagrees(self):
        # Execute the actual runtime method without importing ROS controllers.
        import ast
        from pathlib import Path
        from types import SimpleNamespace
        from hand_camera_ray import rotation
        from outcome_transfer_policy import load
        tree=ast.parse(Path(__file__).with_name('flat_box_grasp.py').read_text())
        method=next(m for c in tree.body if isinstance(c,ast.ClassDef) and c.name=='BoxRuntime' for m in c.body if isinstance(m,ast.FunctionDef) and m.name=='is_aligned')
        scope=dict(np=np,rotation=rotation,outcome_policy=load);exec(compile(ast.Module(body=[method],type_ignores=[]),'actual-is-aligned','exec'),scope)
        for ready,error in ((True,[.05,.02,.02]),(False,[0.,0.,0.])):
            with self.subTest(ready=ready):
                runtime=SimpleNamespace(contact_error=lambda *_:np.asarray(error),palm_pose=lambda _:None,box_axis_odom=[1,0,0],box_model={},record=lambda _:None,
                    box_safety=SimpleNamespace(closure_region=lambda *_args,**_kwargs:dict(ready=ready)))
                snapshot=dict(tracking_transforms=dict(odom_from_base_at_depth=dict(quaternion=[0,0,0,1])))
                self.assertEqual(scope['is_aligned'](runtime,[0,0,0],snapshot),ready)
if __name__=='__main__':unittest.main()
