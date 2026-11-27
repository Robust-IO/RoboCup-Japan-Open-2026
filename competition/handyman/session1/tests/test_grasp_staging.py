import unittest,math
import numpy as np
from grasp_staging import candidates,at_goal
from grasp_runtime.autograsp.kinematics import forward,posture,CONTACT
class GraspStagingTests(unittest.TestCase):
 def test_normal_to_edges_and_gripper_aligned_to_target(self):
  p,r=forward(posture('side'));lateral=(p+r@CONTACT)[1]
  for angle in (0,.6):
   table={'oriented_bounds':{'center':{'x':1,'y':2},'size':{'x':2,'y':1},'yaw':angle}}
   target=[1.2,2.1,.8];poses=candidates(table,target,[[-5,-5],[5,-5],[5,5],[-5,5]])
   self.assertEqual(len(poses),12)
   for x,y,yaw in poses:
    delta=np.array(target[:2])-np.array([x,y]);left=np.array([-math.sin(yaw),math.cos(yaw)])
    self.assertAlmostEqual(delta@left,lateral)
    self.assertAlmostEqual(math.sin(2*(yaw-angle)),0,places=7)
    self.assertGreater(delta@np.array([math.cos(yaw),math.sin(yaw)]),0)
 def test_wrong_heading_is_not_a_completed_grasp_pose(self):
  self.assertFalse(at_goal([1,2,1],[1,2,0]))
  self.assertTrue(at_goal([1,2,0],[1,2,0]))
 def test_no_out_of_room_candidate(self):
  table={'axis_aligned_bounds':{'center':{'x':0,'y':0},'size':{'x':2,'y':1}}}
  self.assertEqual(candidates(table,[0,0,.8],[[-.1,-.1],[.1,-.1],[.1,.1],[-.1,.1]]),[])

 def test_deep_table_end_is_excluded_before_distance_ranking(self):
  table={'axis_aligned_bounds':{'center':{'x':0,'y':0},'size':{'x':1,'y':2.3}}}
  audit=[]
  poses=candidates(table,[.4,0,.8],[[-5,-5],[5,-5],[5,5],[-5,5]],base_radius=.2611,audit=audit)
  self.assertEqual(len(poses),3)
  self.assertTrue(all(abs(abs(p[2])-math.pi)<1e-6 for p in poses))
  self.assertEqual(sum(a['accepted'] for a in audit),1)
  self.assertTrue(all(a['required_reach_m']<=a['max_calibrated_reach_m'] for a in audit if a['accepted']))

 def test_pregrasp_standoff_preserves_observation_space(self):
  table={'axis_aligned_bounds':{'center':{'x':0,'y':0},'size':{'x':1,'y':2.3}}}
  poses=candidates(table,[.4,0,.8],[[-5,-5],[5,-5],[5,5],[-5,5]],base_radius=.2611)
  self.assertEqual(len(poses),3)
  gaps=sorted(p[0]-.5 for p in poses)
  np.testing.assert_allclose(gaps,[1.,1.15,1.3])
  self.assertGreaterEqual(min(gaps)-.2611,.58)

 def test_oblique_candidates_preserve_gripper_line_and_reach(self):
  table={'axis_aligned_bounds':{'center':{'x':0,'y':0},'size':{'x':1,'y':2.3}}}
  target=[.4,0,.8];audit=[]
  poses=candidates(table,target,[[-5,-5],[5,-5],[5,5],[-5,5]],base_radius=.2611,audit=audit,angle_offsets=(0.,math.radians(15),-math.radians(15)))
  self.assertGreater(len(poses),3)
  palm,r=forward(posture('side'));lateral=(palm+r@CONTACT)[1]
  for x,y,yaw in poses:
   self.assertAlmostEqual((np.array(target[:2])-np.array([x,y]))@np.array([-math.sin(yaw),math.cos(yaw)]),lateral)
  self.assertTrue(all(a['required_reach_m']<=a['max_calibrated_reach_m'] for a in audit if a['accepted']))
 def test_staging_tolerance_not_exact_edge_normal(self):
  self.assertTrue(at_goal([.1,0,math.radians(9)],[0,0,0],xy_tolerance=.12,yaw_tolerance=math.radians(10)))
  self.assertFalse(at_goal([.13,0,0],[0,0,0]))
  self.assertFalse(at_goal([0,0,math.radians(11)],[0,0,0]))
 def test_scoped_arrival_uses_staging_tolerance_not_room_entry(self):
  from room_entry import entered,completion_basis
  region=dict(mode='grasp_staging',goal=[1,1,0],polygon=[[0,0],[3,0],[3,3],[0,3]])
  self.assertTrue(entered([1.1,1,math.radians(9)],region))
  self.assertFalse(entered([2,2,0],region))
  self.assertEqual(completion_basis([1.1,1,0],region),'grasp_staging_within_12cm_10deg')

 def test_all_six_sessions_share_flexible_staging(self):
  from grasp_staging import staging_policy
  for n in range(1,7):
   p=staging_policy({'session_profile':{'session_number':n}})
   self.assertTrue(p['flexible']);self.assertEqual(len(p['angle_offsets']),5)
   self.assertAlmostEqual(max(abs(a) for a in p['angle_offsets']),math.radians(15))
 def test_default_at_goal_restores_old_session1_rule(self):
  self.assertFalse(at_goal([.1,0,0],[0,0,0]))
  self.assertFalse(at_goal([0,0,math.radians(9)],[0,0,0]))
  self.assertTrue(at_goal([.07,0,.07],[0,0,0]))
