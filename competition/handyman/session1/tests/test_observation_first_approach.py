import unittest
from types import SimpleNamespace
from threading import Event
from unittest.mock import patch
import numpy as np
from adaptive_grasp_staging import prepare,current_arm_path_clear,current_base_approach_clear
from grasp_runtime.autograsp.kinematics import posture,forward
from candidate_selection import CandidateSelector

class ObservationFirstTests(unittest.TestCase):
 def candidate(self):
  goal=posture('side',lift=.2);palm,_=forward(goal)
  return dict(strategy='side',arm_goal=goal,palm_base_m=(palm+[1.,0,0]).tolist())
 def test_unobstructed_one_metre_approach_does_not_select_furniture_pose(self):
  candidate=self.candidate();backend=SimpleNamespace(table={'id':'white_side_table'},scene=None,record=lambda r:None)
  stages=SimpleNamespace(grasp_backend=backend,state=lambda c:{'poses':{'map':[0,0,0]}},target='dog_doll')
  report=dict(candidates=[candidate],rejected={})
  with patch('adaptive_grasp_staging.direct_plan',return_value=(candidate,report)),patch('adaptive_grasp_staging.current_arm_path_clear',return_value=True):
   prepare(stages,{},Event()) # No map selector or navigate method: must retain view.
 def test_tries_other_local_grasp_before_relocating(self):
  a=self.candidate();b=dict(a,id='second');calls=[]
  backend=SimpleNamespace(table={'id':'white_side_table'},scene=None,record=lambda r:None)
  stages=SimpleNamespace(grasp_backend=backend,state=lambda c:{'poses':{'map':[0,0,0]}},target='dog_doll')
  with patch('adaptive_grasp_staging.direct_plan',return_value=(a,dict(candidates=[a,b],rejected={}))),patch('adaptive_grasp_staging.current_arm_path_clear',side_effect=lambda backend,c:calls.append(c) or c is b):
   prepare(stages,{},Event())
  self.assertEqual(len(calls),2)
 def test_obstacle_in_middle_of_approach_not_only_final_pose_is_checked(self):
  candidate=self.candidate();env=np.array([[1.,0,.8]])
  backend=SimpleNamespace(rt=SimpleNamespace(state=lambda:{'joints':posture('side',lift=.2)}),scene=SimpleNamespace(environment=env,target=np.empty((0,3))),hand=object())
  positions=[]
  def hits(points,palm,*a,**kw):
   positions.append(palm[0]);return 4 if points is env and .95<palm[0]<1.05 else 0
  with patch('grasp_runtime.autograsp.planner._open_collision',side_effect=hits):self.assertFalse(current_arm_path_clear(backend,candidate))
  with patch('grasp_runtime.autograsp.planner._open_collision',return_value=0):self.assertTrue(current_arm_path_clear(backend,candidate))
  self.assertTrue(any(.95<x<1.05 for x in positions))
 def test_base_path_blocker_triggers_relocation_but_distance_alone_does_not(self):
  free=np.ones((100,100),bool);selector=CandidateSelector(free,.05,[-2,-2,0],.1,{})
  stages=SimpleNamespace(candidate_selector=selector);candidate=self.candidate()
  self.assertTrue(current_base_approach_clear(stages,candidate,[0,0,0]))
  selector.walk[selector.cell([.5,0])]=False
  self.assertFalse(current_base_approach_clear(stages,candidate,[0,0,0]))

 def test_executor_uses_candidate_that_passed_current_direction_check(self):
  from grasp_runtime.autograsp.cyclic_recovery import CyclicCoordinator
  from grasp_runtime.autograsp.planner import Policy
  a={'id':'blocked','strategy':'side'};b={'id':'clear','strategy':'side'};executed=[]
  backend=SimpleNamespace(holding_state=lambda:'empty',check_task_cancelled=lambda:None,
      observe_stopped=lambda target:object(),record=lambda r:None,
      filter_approach_decision=lambda d:dict(d,selected=b,candidates=[b]),
      execute_candidate=lambda c,s:executed.append(c),verify_lift=lambda:SimpleNamespace(),candidate=b,stop=lambda:None)
  with patch('grasp_runtime.autograsp.cyclic_recovery.select_template',return_value=(Policy(),{})),patch('grasp_runtime.autograsp.cyclic_recovery.plan',return_value={'selected':a,'candidates':[a,b]}),patch('grasp_runtime.autograsp.cyclic_recovery.accepted',return_value=True):
   result=CyclicCoordinator(backend,None,Policy(),lambda r:None).run('dog_doll')
  self.assertEqual(executed,[b]);self.assertEqual(result['status'],'grasp_verified')
