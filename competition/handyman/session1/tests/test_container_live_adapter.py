import numpy as np
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from container_live_adapter import ContainerLiveAdapter

class LiveContainerTests(unittest.TestCase):
    def setUp(self):
        self.tmp=TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.calls=[]
        self.adapter=ContainerLiveAdapter.__new__(ContainerLiveAdapter)
        self.adapter.released=False
        self.adapter.opening=dict(destination_id='bin',polygon_map_m=[[-.2,-.2],[.2,-.2],[.2,.2],[-.2,.2]],rim_height_m=.5)
        self.adapter.stages=SimpleNamespace(output=Path(self.tmp.name),destination_id='bin')
        self.adapter.rt=object()
        def release(rt,check):
            check();self.calls.append('release')
        self.adapter.core=SimpleNamespace(release_open=release)
        self.adapter._snapshot=lambda:{'streams':{'depth':{'stamp_ns':123}}}
        self.adapter.object_projection_map_m=np.array([[-.01,-.01],[.01,-.01],[.01,.01],[-.01,.01]])
        self.adapter.current_gripper_and_bottom=lambda:([0,0,.7],.55)
    def test_one_release_no_visual_identity_check(self):
        self.assertEqual(self.adapter.release_once(),123)
        self.assertTrue(json.loads((Path(self.tmp.name)/'container-release.json').read_text())['release_sent'])
        with self.assertRaises(ValueError):self.adapter.release_once()
        self.assertEqual(self.calls,['release'])
    def test_current_outside_opening_no_release(self):
        self.adapter.object_projection_map_m=np.array([[.49,-.01],[.51,-.01],[.51,.01],[.49,.01]])
        self.adapter.current_gripper_and_bottom=lambda:([.5,0,.7],.55)
        with self.assertRaises(ValueError):self.adapter.release_once()
        self.assertFalse(self.calls)

    def test_large_projection_does_not_block_center_over_opening(self):
        self.adapter.object_projection_map_m=np.array([[-.5,-.5],[.5,.5]])
        self.assertEqual(self.adapter.release_once(),123)
        self.assertEqual(self.calls,['release'])
        record=json.loads((Path(self.tmp.name)/'container-release.json').read_text())
        self.assertFalse(record['held_projection_check_enabled'])
    def test_raised_release_plan_has_fifteen_cm_bottom_clearance(self):
        from container_transfer import release_pose
        plan=release_pose(self.adapter.opening,.08)
        self.assertAlmostEqual(plan['palm_height_m'],.5+.15+.08)
    def test_bottom_below_rim_still_does_not_release(self):
        self.adapter.current_gripper_and_bottom=lambda:([0,0,.6],.49)
        with self.assertRaises(ValueError):self.adapter.release_once()

    def test_failed_rim_refinement_keeps_configured_opening_without_release(self):
        from unittest.mock import Mock,patch
        a=self.adapter
        original=dict(a.opening)
        a.stages.config={'session_profile':{'session_number':4}}
        a.stages.motion=SimpleNamespace()
        snapshot={'streams':{'depth':{'stamp_ns':123}},'tracking_transforms':{
            'map_from_base_at_depth':{'quaternion':[0,0,0,1],'translation':[0,0,0]}}}
        a.rt=Mock();a.rt.save.return_value='capture'
        a.prepare_low_bin_view=Mock()
        a.rt.palm_pose.return_value={'translation':[0,0,0],'quaternion':[0,0,0,1]}
        a.map_point=lambda snapshot,p:p
        a.backend=Mock();a.backend._sample.return_value={'snapshot':snapshot}
        a.backend._cloud.return_value=(np.empty((0,3)),None,None)
        a.rotation=lambda q:np.eye(3)
        with patch('container_route_heading.face_container_route'),patch('container_live_adapter.motion_cancel',return_value=None),patch('container_live_adapter.check_cancel'),patch('container_rim_calibration.calibrate',side_effect=ValueError('current_bin_rim_edges_ambiguous')):
            self.assertEqual(a.observe_opening('bin'),original)
        self.assertEqual(a.rt.face_source.call_count,3)
        event=a.backend.record.call_args.args[0]
        self.assertEqual(event['event'],'container_rim_calibration_fallback')
        self.assertFalse(event['current_rim_verified'])
        self.assertFalse(a.released)
        self.assertEqual(a.opening,original)

    def test_low_view_uses_lower_height_without_releasing(self):
        from unittest.mock import Mock
        a=self.adapter
        a.core=Mock();a.core.offset=.3;a.core.rotation=np.eye(3)
        a.core.predict.return_value=np.array([1.,0.,.62])
        a.bottom_below_palm_m=lambda:.08
        a.map_point=lambda s,p:p
        a.rt=Mock();a.rt.state.return_value={'joints':[0,0,0,0,0,.4]}
        a.backend=Mock()
        a.prepare_low_bin_view()
        a.core.prepare_release.assert_called_once_with(a.rt,.62,.4)
        self.assertFalse(a.released)
    def test_low_view_does_not_lower_hand_over_mouth(self):
        from unittest.mock import Mock
        a=self.adapter
        a.core=Mock();a.core.offset=.3;a.core.rotation=np.eye(3)
        a.core.predict.return_value=np.array([0.,0.,.62])
        a.bottom_below_palm_m=lambda:.08
        a.map_point=lambda s,p:p
        a.backend=Mock()
        a.prepare_low_bin_view()
        a.core.prepare_release.assert_not_called()
    def test_bottle_bin_also_uses_low_view(self):
        import inspect
        # Exercise the same adapter branch with Session2 through the existing fixture.
        from unittest.mock import patch
        original=ContainerLiveAdapter.observe_opening
        def session2(adapter,destination):
            adapter.stages.config['session_profile']['session_number']=2
            return original(adapter,destination)
        with patch.object(ContainerLiveAdapter,'observe_opening',session2):
            self.test_failed_rim_refinement_keeps_configured_opening_without_release()
