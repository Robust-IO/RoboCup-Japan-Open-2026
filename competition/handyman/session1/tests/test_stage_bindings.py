import json
import tempfile
import unittest
from threading import Event
from types import SimpleNamespace
from pathlib import Path
import numpy as np
from stage_bindings import StageBindings
class BindingTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.calls=[]
        rt=SimpleNamespace(face_source=lambda *a:None,sample=lambda **kw:None,state=lambda:dict(joints=[0,0,0,0,0,.31],poses={'map':[1,1,0]}))
        motion=SimpleNamespace(room_at=lambda p:'living_room',navigate=self.navigate)
        env=dict(rooms={'living_room':{'region':[[0,0],[2,0],[2,2],[0,2]]}},destinations={
            'white_side_table':[dict(room='bedroom',x=99,y=99,yaw=0),dict(room='living_room',x=1,y=1,yaw=0)],
            'square_low_table':[dict(room='living_room',x=1.5,y=1,yaw=0)]})
        self.scene=SimpleNamespace(target=np.array([[1,1,.5],[1.01,1,.5]]),stamp_ns=10)
        self.backend=SimpleNamespace(observe_stopped=lambda target:self.scene,source_snapshot={},
            table={'id':'white_side_table#living_room','axis_aligned_bounds':{'center':{'x':1,'y':1},'size':{'x':.5,'y':.5}}})
        self.b=StageBindings(rt,motion,self.backend,{},env,self.tmp.name,map_point=lambda s,p:p)
        self.effect={'operation_id':'run/1','stage':'navigate_source'};self.cancel=Event()
    def navigate(self,*args):self.calls.append(args);return dict(verified=True,evidence='synthetic-navigation')
    def test_already_in_room_no_motion(self):
        self.assertTrue(self.b.navigate_room(self.effect,self.cancel)['arrived']);self.assertFalse(self.calls)
    def test_source_uses_current_opening_and_existing_route_backend(self):
        self.backend.table['axis_aligned_bounds']['center']['x']=5
        self.b.navigate_source(self.effect,self.cancel)
        self.assertEqual(self.b.motion.stage,'navigate_source')
        self.assertEqual(self.calls[0][2],[1.,1.,0.]);self.assertEqual(self.calls[0][3],.31)
    def test_actual_scene_target_field(self):
        self.assertTrue(self.b.observe_target(self.effect,self.cancel)['confirmed'])
        self.scene.target=np.array([[1.8,1,.5]])
        self.assertEqual(self.b.observe_target(self.effect,self.cancel)['status'],'incomplete')
    def test_cancel_before_navigation(self):
        self.cancel.set()
        with self.assertRaises(RuntimeError):self.b.navigate_source(self.effect,self.cancel)
        self.assertFalse(self.calls)
    def test_no_placeholder_transfer_stages(self):
        with self.assertRaises(ValueError):self.b.handlers({})

    def test_source_does_not_use_list_order_when_other_candidate_is_nearer(self):
        self.backend.table['axis_aligned_bounds']['center']['x']=5
        self.b.source_candidates=[[8.,8.,0.],[1.,1.,0.]]
        self.b.navigate_source(self.effect,self.cancel)
        self.assertEqual(self.calls[0][2],[1.,1.,0.])

    def test_room_entry_uses_same_path_ranking_before_first_navigation(self):
        self.b.motion.room_at=lambda p:'bedroom'
        self.b.source_candidates=[[8.,8.,0.],[2.,1.,0.]]
        ranks=[]
        self.b.candidate_selector=SimpleNamespace(rank=lambda pose,poses,fid:(ranks.append((pose,poses,fid)) or [poses[1],poses[0]],{'ranked_indices':[1,0]}))
        selected=[]
        self.b.navigate=lambda effect,cancel,goal,room:selected.append(goal) or {'arrived':True}
        self.b.navigate_room({'operation_id':'run/1','stage':'navigate_room'},self.cancel)
        self.assertEqual(selected,[[2.,1.,0.]])
        self.assertEqual(len(ranks),1)
        self.assertTrue((Path(self.tmp.name)/'room-entry-candidate-selection.json').exists())
    def test_room_entry_does_not_drive_when_no_unobstructed_candidate(self):
        self.b.motion.room_at=lambda p:'bedroom'
        self.b.candidate_selector=SimpleNamespace(rank=lambda *a:([],{'ranked_indices':[]}))
        with self.assertRaises(ValueError):self.b.navigate_room(self.effect,self.cancel)
        self.assertFalse(self.calls)

    def test_near_source_already_at_center_skips_undefined_heading(self):
        self.b.runtime.face_source=lambda *a:self.fail('first frame must use current view')
        self.assertEqual(self.b.navigate_source(self.effect,self.cancel)['basis'],'already_near_furniture')
        self.assertTrue(self.b.observe_target(self.effect,self.cancel)['confirmed'])
        self.assertEqual(self.calls,[])

    def test_destination_navigation_uses_destination_room(self):
        self.b.config['destination_room']='bedroom'
        self.b.environment['rooms']['bedroom']={'region':[[0,0],[3,0],[3,3],[0,3]]}
        calls=[]
        self.b.navigate=lambda effect,cancel,goal,room:calls.append(room) or {'status':'ok'}
        self.b.navigate_destination(self.effect,self.cancel)
        self.assertEqual(calls,['bedroom'])

    def test_observation_faces_furniture_without_changing_xy(self):
        self.backend.table['axis_aligned_bounds']['center'].update(x=1,y=2)
        heads=[];self.b.runtime.face_source=lambda goal:heads.append(goal)
        self.b.face_source_furniture(self.effect,self.cancel)
        self.assertEqual(self.calls[0][2][:2],[1,1])
        self.assertAlmostEqual(self.calls[0][2][2],np.pi/2)
        self.assertEqual(heads,[(0.,0.)])

    def test_grasp_arrival_retries_head_and_stops_on_detection(self):
        from grasp_runtime.autograsp.coordinator import AttemptFailure
        from grasp_runtime.autograsp.recovery import Failure
        heads=[];events=[];self.backend.identity_head_goal=(0.,0.)
        self.b.runtime.face_source=lambda h:heads.append(h)
        self.backend.record=events.append
        def observe(target):
            if len(heads)<3:raise AttemptFailure(Failure.OCCLUDED)
            return self.scene
        self.backend.observe_stopped=observe
        self.assertIs(self.b.reacquire_after_staging(self.cancel),self.scene)
        from observation_head_views import source_head_views
        self.assertEqual(heads,source_head_views()[:3])
        self.assertEqual(self.calls,[])
        self.assertEqual(events[-1]['event'],'grasp_staging_identity_reacquired')
    def test_grasp_arrival_does_not_repeat_angles_or_hide_sensor_faults(self):
        from grasp_runtime.autograsp.coordinator import AttemptFailure
        from grasp_runtime.autograsp.recovery import Failure
        heads=[];self.backend.identity_head_goal=(0.,-.5)
        self.b.runtime.face_source=lambda h:heads.append(h);self.backend.record=lambda r:None
        def missing(target):raise AttemptFailure(Failure.OCCLUDED)
        self.backend.observe_stopped=missing
        with self.assertRaises(AttemptFailure):self.b.reacquire_after_staging(self.cancel)
        self.assertEqual(len(heads),len(set(heads)))
        heads.clear()
        def sensor(target):raise AttemptFailure(Failure.SENSOR_UNAVAILABLE)
        self.backend.observe_stopped=sensor
        with self.assertRaises(AttemptFailure):self.b.reacquire_after_staging(self.cancel)
        self.assertEqual(len(heads),1)

    def test_table_arrival_retains_existing_mode(self):
        self.b.navigate_destination(self.effect,self.cancel)
        self.assertFalse((self.b.output/'position-only-arrival.json').exists())

    def test_repeated_facing_uses_fresh_child_output_identity(self):
        self.backend.table['axis_aligned_bounds']['center'].update(x=1,y=2)
        effects=[]
        self.b.navigate=lambda e,c,g,r:effects.append(e['operation_id'])
        self.b.face_source_furniture(self.effect,self.cancel)
        self.b.face_source_furniture(self.effect,self.cancel)
        self.assertEqual(len(set(effects)),2)

    def test_lateral_view_arrives_facing_furniture(self):
        from furniture_reobservation import lateral_views
        rows=lateral_views([0,0,1.2],[[-4,-4],[4,-4],[4,4],[-4,4]],center={'x':2,'y':0})
        import math
        for x,y,yaw in rows:self.assertAlmostEqual(yaw,math.atan2(-y,2-x))
