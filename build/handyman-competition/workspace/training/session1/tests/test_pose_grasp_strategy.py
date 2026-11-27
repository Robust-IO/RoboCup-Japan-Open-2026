import unittest,tempfile
from pathlib import Path
import numpy as np
from grasp_runtime.autograsp.planner import Scene,Policy
from grasp_runtime.autograsp.templates import select_template
from grasp_runtime.autograsp.capture_names import unused_capture_name

class PoseStrategyTests(unittest.TestCase):
    def scene(self,width,height,yaw=0.,table=.4):
        x,y,z=np.meshgrid(np.linspace(-width/2,width/2,12),np.linspace(-.03,.03,7),np.linspace(.002,height,10))
        p=np.c_[x.ravel(),y.ravel(),z.ravel()]
        c,s=np.cos(yaw),np.sin(yaw);p[:,:2]=p[:,:2]@np.array([[c,s],[-s,c]])
        p+=np.array([.7,0,table])
        return Scene(p,np.empty((0,3)),np.array([0,0,table]),1)
    def test_upright_can_prefers_side(self):
        p,m=select_template('canned_juice',self.scene(.065,.125),Policy())
        self.assertEqual(p.preferred_strategy,'side')
    def test_lying_can_side_first_then_overhead_after_fold(self):
        for yaw in [0,.6,1.57]:
            p,m=select_template('canned_juice',self.scene(.13,.065,yaw),Policy())
            self.assertEqual(p.preferred_strategy,'side');self.assertEqual(p.allowed_strategies,('side',));self.assertTrue(m['overhead_after_fold'])
    def test_ambiguous_remains_side(self):
        p,m=select_template('canned_juice',self.scene(.08,.07),Policy())
        self.assertEqual(p.preferred_strategy,'side')
    def test_upright_again_does_not_keep_overhead_only(self):
        old,_=select_template('canned_juice',self.scene(.13,.065),Policy())
        new,_=select_template('canned_juice',self.scene(.065,.125),old)
        self.assertEqual(new.allowed_strategies,('side','overhead'));self.assertEqual(new.preferred_strategy,'side')
    def test_table_height_does_not_change_strategy(self):
        p,_=select_template('canned_juice',self.scene(.13,.065,table=.8),Policy())
        self.assertEqual(p.preferred_strategy,'side')
    def test_retry_keeps_old_capture(self):
        with tempfile.TemporaryDirectory() as root:
            directory=Path(root)/'captures';directory.mkdir()
            self.assertEqual(unused_capture_name(root,'preclose'),'preclose')
            (directory/'preclose').mkdir();(directory/'preclose-attempt-002').mkdir()
            self.assertEqual(unused_capture_name(root,'preclose'),'preclose-attempt-003')
    def test_empty_close_invalidates_old_shape(self):
        from types import SimpleNamespace
        from grasp_runtime.autograsp.ros_backend import RosBackend
        events=[]
        backend=SimpleNamespace(holding_state=lambda:'empty',retreat_steps=[],identity_confirmed=True,
            pose_locked=True,record=events.append,rt=SimpleNamespace(state=lambda:{'joints':[0,0,0,0,0,0]},move_joints=lambda *a,**k:None))
        RosBackend.recover(backend,{'action':'withdraw_empty_reobserve_replan','open_hand':True,'failure':'empty_close'})
        self.assertFalse(backend.identity_confirmed);self.assertFalse(backend.pose_locked)
        self.assertFalse(events[-1]['old_surface_template_reused'])
