import math,tempfile,unittest
from types import SimpleNamespace
from pathlib import Path
from threading import Event
from unittest.mock import patch
import numpy as np
from grasp_staging import candidates,NAVIGATION_GAPS_M
from adaptive_grasp_staging import prepare,direct_plan
from grasp_runtime.autograsp.kinematics import forward,posture

class NavigationStandoffTests(unittest.TestCase):
 def test_all_normal_and_oblique_candidates_at_least_one_metre_outside_rotated_furniture(self):
  for yaw in (0.,math.pi/4,-.6):
   table={'oriented_bounds':{'center':{'x':0,'y':0},'size':{'x':.6,'y':.8},'yaw':yaw}}
   c,s=math.cos(yaw),math.sin(yaw);R=np.array([[c,-s],[s,c]])
   rows=candidates(table,[0,0,.6],[[-4,-4],[4,-4],[4,4],[-4,4]],angle_offsets=(0.,math.pi/12,-math.pi/12))
   self.assertTrue(rows)
   for row in rows:
    local=np.asarray(row[:2])@R
    distance=np.linalg.norm(np.maximum(abs(local)-[.3,.4],0))
    self.assertGreaterEqual(distance,1.-1e-8)
 def test_far_grasp_plan_can_use_same_observation_direction(self):
  joints=posture('side',lift=.2);palm,_=forward(joints)
  candidate=dict(arm_goal=joints,palm_base_m=(palm+[1.,0,0]).tolist())
  with patch('adaptive_grasp_staging.approach_plan',return_value={'candidates':[candidate]}):
   selected,_=direct_plan(None,None,'dog_doll')
  self.assertIs(selected,candidate);self.assertAlmostEqual(selected['base_alignment_distance_m'],1.)
 def test_navigation_standoff_hands_off_to_grasp_without_demanding_direct_reach(self):
  with tempfile.TemporaryDirectory() as tmp:
   scene=SimpleNamespace(target=np.array([[0,0,.6],[.01,0,.6]]),environment=np.empty((0,3)),plane=np.array([0,0,.5]),stamp_ns=1)
   table={'axis_aligned_bounds':{'center':{'x':0,'y':0},'size':{'x':.4,'y':.4}}}
   backend=SimpleNamespace(scene=scene,table=table,source_snapshot={},record=lambda r:None,holding_state=lambda:'empty')
   moves=[];refreshed=[]
   state={'poses':{'map':[0,-2,math.pi/2]},'joints':[0,0,0,0,0,.65]}
   selector=SimpleNamespace(radius=.26,rank=lambda p,rows,f:(rows,{}))
   stages=SimpleNamespace(grasp_backend=backend,state=lambda c:state,target='dog_doll',map_point=lambda s,p:p,
       environment={'rooms':{'room':{'region':[[-4,-4],[4,-4],[4,4],[-4,4]]}}},source_room='room',candidate_selector=selector,
       source_id='table',output=Path(tmp),check=lambda c:None,runtime=SimpleNamespace(prepare_navigation_from_current=lambda:None),
       navigate=lambda e,c,g,r:moves.append(g),reacquire_after_staging=lambda c:refreshed.append(True))
   feasible={'selected':{'id':'candidate'},'candidates':[{'id':'candidate'}],'rejected':{}}
   with patch('adaptive_grasp_staging.direct_plan',return_value=(None,feasible)) as direct,patch('adaptive_grasp_staging.approach_plan',return_value=feasible),patch('adaptive_grasp_staging.current_base_approach_clear',return_value=True),patch('adaptive_grasp_staging.current_arm_path_clear',return_value=True):
    prepare(stages,{'operation_id':'test'},Event())
   self.assertEqual(direct.call_count,1);self.assertEqual(len(moves),1);self.assertEqual(refreshed,[True])
   self.assertGreaterEqual(np.linalg.norm(np.maximum(abs(np.array(moves[0][:2]))-.2,0)),1.-1e-8)
 def test_current_usable_approach_does_not_force_another_navigation(self):
  backend=SimpleNamespace(table={'id':'white_side_table'},scene=object(),record=lambda r:None)
  s=SimpleNamespace(grasp_backend=backend,state=lambda c:{'poses':{'map':[0,0,0]}},target='apple')
  with patch('adaptive_grasp_staging.direct_plan',return_value=({'id':'direct'},{'rejected':{}})),patch('adaptive_grasp_staging.current_arm_path_clear',return_value=True):
   prepare(s,{},Event()) # No selector/navigation needed for an already usable pose.
