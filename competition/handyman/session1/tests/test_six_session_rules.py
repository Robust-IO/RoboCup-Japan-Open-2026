import json,math,tempfile,unittest
from pathlib import Path
from threading import Event
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from container_geometry import projection_inside,fit_release_xy
from container_transfer import OutsideFloorEvidence
from navigation_turn_policy import ArrivalHeading
from session3_search_policy import choose_points
from grasp_runtime.autograsp.shape_contacts import protrusion,contact_preference
from grasp_runtime.autograsp.planner import Scene
from adaptive_grasp_staging import transform_scene

class NewRulesTests(unittest.TestCase):
 def test_room_boundary_1_35(self):
  row={'main':dict(x=0,y=0,yaw=0),'backups':[dict(x=1,y=0,yaw=0)]}
  for width,n in [(1.35,1),(1.351,2)]:
   self.assertEqual(len(choose_points(row,[[0,0],[width,0],[width,1],[0,1]])['points']),n)
 def test_navigation_handoff_near_arrival_rotation_no_50degree_deadlock(self):
  a=ArrivalHeading();yaw,changed=a.select([.061,0,.9],[0,0,math.pi/2],[],controller_rotating=True)
  self.assertTrue(a.arrived);self.assertAlmostEqual(yaw,math.pi/2)
  self.assertEqual(a.select([.075,0,1.2],[0,0,math.pi/2],[])[0],math.pi/2)
  a.select([.09,0,1.2],[0,0,math.pi/2],[]);self.assertFalse(a.arrived)
 def test_no_early_turn_for_translating_far_robot(self):
  a=ArrivalHeading();a.select([.061,0,0],[0,0,1],[],controller_rotating=False);self.assertFalse(a.arrived)
  a.select([.14,0,0],[0,0,1],[],controller_rotating=True);self.assertFalse(a.arrived)
 def test_moderator_position_does_not_require_facing(self):
  from room_entry import entered
  region=dict(mode='position_only',goal=[0,0,0],polygon=[[-1,-1],[1,-1],[1,1],[-1,1]])
  self.assertTrue(entered([.04,0,math.pi],region));self.assertFalse(entered([.1,0,0],region))
 def test_gripper_inside_but_object_over_rim_is_rejected(self):
  poly=[[-.15,-.15],[.15,-.15],[.15,.15],[-.15,.15]]
  obj=np.array([[-.04,-.04],[.04,-.04],[.04,.04],[-.04,.04]])
  self.assertTrue(projection_inside(obj,poly));self.assertFalse(projection_inside(obj+[.13,0],poly))
  xy=fit_release_xy(dict(polygon_map_m=poly),obj,np.array([.13,0]))
  self.assertTrue(projection_inside(obj+xy,poly));self.assertLess(xy[0],.13)
 def test_concave_mouth_cannot_be_bridged(self):
  poly=[[-1,-1],[1,-1],[1,1],[.1,1],[.1,0],[-.1,0],[-.1,1],[-1,1]]
  self.assertFalse(projection_inside([[-.4,.5],[.4,.5],[.4,.7],[-.4,.7]],poly))
 def test_oversized_object_never_gets_fake_release_position(self):
  with self.assertRaisesRegex(ValueError,'does_not_fit'):
   fit_release_xy(dict(polygon_map_m=[[-.05,-.05],[.05,-.05],[.05,.05],[-.05,.05]]),np.array([[-.1,-.1],[.1,-.1],[.1,.1],[-.1,.1]]),np.array([0,0]))
 def test_rim_object_not_external_floor_is_not_success(self):
  proof=OutsideFloorEvidence('can','bin',1)
  def row(stamp,**kw):return dict(destination_id='bin',depth_stamp_ns=stamp,valid_frame=True,floor_visible=True,rim_visible=True,rim_observation_required=True,evidence_ref='image',floor_targets=[],**kw)
  proof.observe(row(2_000_000_000));proof.observe(row(2_200_000_000))
  r=proof.observe(row(2_400_000_000,rim_targets=[dict(target='can')]))
  self.assertEqual(r['state'],'rim_target_found');self.assertFalse(proof.clear)
  r=proof.observe(row(2_600_000_000));self.assertEqual(r['state'],'observing')
 def test_virtual_pose_keeps_support_plane_coordinates_consistent(self):
  scene=Scene(np.array([[1.,2.,.7],[1.1,2,.72]]),np.array([[1.,2.,.7]]),np.array([.2,.1,.3]),1)
  s=transform_scene(scene,[2,3,.7],[-1,2,-.4])
  residual0=scene.target[:,2]-scene.target[:,:2]@scene.plane[:2]-scene.plane[2]
  residual1=s.target[:,2]-s.target[:,:2]@s.plane[:2]-s.plane[2]
  np.testing.assert_allclose(residual0,residual1,atol=1e-10)
 def test_dolls_require_root_tip_not_just_small_width(self):
  # Flat constant-width bar has no observed widening at a protrusion root.
  p=np.array([[x,y,z] for x in np.linspace(-.03,.03,12) for y in np.linspace(-.02,.02,8) for z in np.linspace(0,.03,10)])
  proof=protrusion(p,[0,0,.025]);self.assertIsNone(proof)
 def test_clear_protrusion_has_measured_root_and_at_least_cm(self):
  body=np.array([[x,y,z] for x in np.linspace(-.04,.04,12) for y in np.linspace(-.04,.04,12) for z in np.linspace(0,.04,10)])
  tip=np.array([[x,y,z] for x in np.linspace(-.01,.01,8) for y in np.linspace(-.01,.01,8) for z in np.linspace(.04,.075,15)])
  proof=protrusion(np.vstack([body,tip]),[0,0,.06])
  self.assertIsNotNone(proof);self.assertGreaterEqual(proof['length_m'],.01)
 def test_subcentimeter_protrusion_rejected(self):
  body=np.array([[x,y,z] for x in np.linspace(-.04,.04,12) for y in np.linspace(-.04,.04,12) for z in np.linspace(0,.04,15)])
  tip=np.array([[x,y,z] for x in np.linspace(-.01,.01,8) for y in np.linspace(-.01,.01,8) for z in np.linspace(.04,.047,8)])
  self.assertIsNone(protrusion(np.vstack([body,tip]),[0,0,.045]))
 def test_source_head_misses_reach_lateral_views(self):
  from furniture_reobservation import observe_furniture
  from grasp_runtime.autograsp.coordinator import AttemptFailure
  from grasp_runtime.autograsp.recovery import Failure
  observed=[];moves=[]
  def see(*a,head=None):
   observed.append(head)
   if not moves:raise AttemptFailure(Failure.OCCLUDED,'hidden')
   return {'confirmed':True}
  stages=SimpleNamespace(state=lambda c:{'poses':{'map':[0,0,0]}},grasp_backend=SimpleNamespace(record=lambda r:None,table={'axis_aligned_bounds':{'center':{'x':1,'y':0}}}),
   environment={'rooms':{'room':{'region':[[-1,-1],[1,-1],[1,1],[-1,1]]}}},source_room='room',source_id='table',
   candidate_selector=SimpleNamespace(rank=lambda p,c,f:(c,{})),runtime=SimpleNamespace(prepare_navigation_from_current=lambda:None),
   check=lambda c:None,face_source_furniture=lambda e,c:None,navigate=lambda *a:moves.append(a[2]),_observe_target_once=see)
  from unittest.mock import patch
  with patch('furniture_reobservation.shift_observation_view',side_effect=lambda s,p,c:moves.append(p) or True):
   self.assertTrue(observe_furniture(stages,{'operation_id':'id'},Event())['confirmed'])
  self.assertEqual(len(moves),1);self.assertAlmostEqual(abs(moves[0][1]),.2);self.assertEqual(len(observed),13)
 def test_suspect_interrupts_scan_and_resume_retains_covered_bins(self):
  from session3_search_live import Session3Search
  with tempfile.TemporaryDirectory() as tmp:
   s=Session3Search.__new__(Session3Search);s.options=dict(continuous_bins=12);s.points=[dict(x=0,y=0,yaw=0)];s.serial=0;s.output=Path(tmp);s.cfg={'search_plan':{}}
   calls=[];near=[];heads=[]
   s.rt=SimpleNamespace(face_source=lambda h:heads.append(h),sample=lambda **k:None,state=lambda:{'poses':{'map':[0,0,.5]}})
   s.navigate=lambda *a:None;s.capture=lambda *a:self.fail('valid coverage must not be repeated')
   candidate=dict(position_map_m=[1,1,.6],confidence=.4)
   def row(i,**k):return dict(angle_bin=i,usable=True,matches=[],unresolved=False,candidates=[],**k)
   first=row(0);first['candidates']=[candidate];first['unresolved']=True
   results=iter([(0,[first],.5),(0,[row(i) for i in range(1,12)],2*math.pi-.53),(0,[row(i) for i in range(12)],2*math.pi)])
   def run(*a,**k):calls.append(k);return next(results)
   s.near_candidate=lambda *a:near.append(a) or dict(confirmed=False,resolved=True)
   with patch('continuous_scan.ContinuousSweep') as cls:
    cls.return_value.run.side_effect=run
    result=s.continuous_search({'task':{'target':'dog_doll'}},Event())
   self.assertTrue(result['search_exhausted']);self.assertEqual(len(near),1)
   self.assertLess(calls[1]['remaining_angle'],2*math.pi);self.assertIn(0,calls[1]['skip_bins'])
   self.assertIn((0.,-math.pi/6),heads)

class DeploymentAndRecoveryTests(unittest.TestCase):
 def test_all_six_configurations_share_rules_and_are_valid(self):
  from live_config import validate
  root=Path(__file__).resolve().parents[1]
  for n in range(1,7):
   name='session3-search-config.json' if n==3 else f'session{n}-live-config.json'
   c=json.loads((root/name).read_text());validate(c)
   self.assertEqual(c['shared_behavior_rules']['grasp']['irregular_min_protrusion_m'],.01)
   self.assertEqual(c['shared_behavior_rules']['search']['aspect_ratio_threshold'],1.35)
 def test_crossed_opening_and_wrong_destination_rejected(self):
  from import_container_opening import validate
  row=dict(destination_id='bin',polygon_map_m=[[0,0],[.3,.3],[.3,0],[0,.3]],rim_height_m=.6,source='reviewed')
  with self.assertRaises(ValueError):validate(row,'bin')
  row['polygon_map_m']=[[0,0],[.3,0],[.3,.3],[0,.3]]
  self.assertIs(validate(row,'bin'),row)
  with self.assertRaisesRegex(ValueError,'destination'):validate(row,'another-bin')
 def test_rim_recovery_uses_real_grasp_and_restores_source_binding(self):
  from container_recovery import recover
  source={'id':'source'};bin_table={'id':'bin','axis_aligned_bounds':{'center':{'x':0,'y':0,'z':.3},'size':{'x':.3,'y':.3,'z':.6}}}
  backend=SimpleNamespace(table=source,holding_state=lambda:'empty')
  stages=SimpleNamespace(target='can',motion=SimpleNamespace(session_cancel_event=Event()),grasp_backend=backend,
   config={'container_destination':bin_table,'container_opening':{'rim_height_m':.6},'destination_room':'living_room'},
   source_id='source',source_room='kitchen',context={'old':True},candidate_selector=SimpleNamespace(tables={'bin':bin_table}))
  calls=[]
  def observe(e,c):calls.append(('observe',stages.source_id));return {'confirmed':True}
  def grasp(e,c):calls.append(('grasp',stages.source_id));stages.context={'new_verified_hold':True};return {'grasp_verified':True}
  stages.observe_target=observe;stages.grasp=grasp
  result=recover(stages,[dict(target='can',location='rim',center_map_m=[0,0,.65])])
  self.assertEqual(calls,[('observe','bin'),('grasp','bin')]);self.assertTrue(result['new_verified_hold'])
  self.assertIs(backend.table,source);self.assertEqual(stages.source_room,'kitchen');self.assertIs(stages.candidate_selector.tables['bin'],bin_table)
 def test_recovery_cannot_fake_hold_when_identity_missing(self):
  from container_recovery import recover
  source={'id':'old'};table={'id':'bin','axis_aligned_bounds':{'center':{'x':0,'y':0,'z':.3},'size':{'x':.3,'y':.3,'z':.6}}}
  b=SimpleNamespace(table=source,holding_state=lambda:'empty')
  s=SimpleNamespace(target='can',motion=SimpleNamespace(session_cancel_event=Event()),grasp_backend=b,source_id='old',source_room='room',context={},candidate_selector=SimpleNamespace(tables={}),
    config={'container_destination':table,'destination_room':'room','container_opening':{'rim_height_m':.6}},observe_target=lambda *a:{'confirmed':False},grasp=lambda *a:self.fail('must not grasp unobserved target'))
  self.assertFalse(recover(s,[dict(target='can',location='rim',center_map_m=[0,0,.65])]))
  self.assertEqual(s.source_id,'old');self.assertIs(b.table,source)
