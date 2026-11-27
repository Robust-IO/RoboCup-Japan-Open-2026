import math,unittest
from cabinet_grasp import is_cabinet,front_aligned,front_heading
from grasp_staging import candidates
class CabinetTests(unittest.TestCase):
 def table(self):return dict(id='body#kitchen',semantic_class='wooden_cupboard_body',front_yaw=-math.pi/2,axis_aligned_bounds={'center':{'x':0,'y':0},'size':{'x':1.5,'y':.62}})
 def test_side_observation_cannot_bypass_front_rule(self):
  t=self.table();self.assertTrue(is_cabinet(t));self.assertTrue(front_aligned(t,[0,-1.5,math.pi/2]))
  self.assertFalse(front_aligned(t,[1.5,0,math.pi]));self.assertFalse(front_aligned(t,[0,1.5,math.pi/2]))
  self.assertFalse(front_aligned(t,[0,-1.5,math.pi/2+.15]))
 def test_candidates_keep_only_cabinet_front(self):
  t=self.table();rows=candidates(t,[0,-.1,.7],[[-4,-4],[4,-4],[4,4],[-4,4]])
  accepted=[p for p in rows if front_aligned(t,p,1e-5)]
  self.assertTrue(accepted);self.assertTrue(all(p[1]<0 for p in accepted))
 def test_table_rule_unchanged(self):self.assertTrue(front_aligned({'id':'white_side_table'},[0,0,1]))
 def test_missing_cabinet_front_not_guessed(self):
  t=self.table();del t['front_yaw']
  with self.assertRaises(ValueError):front_heading(t)
