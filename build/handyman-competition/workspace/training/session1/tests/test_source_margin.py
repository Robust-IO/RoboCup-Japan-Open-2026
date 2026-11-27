import unittest
from grasp_runtime.autograsp.identity_policy import within_source
class SourceMarginTests(unittest.TestCase):
 def test_recorded_ketchup_edge(self):
  table={'axis_aligned_bounds':{'center':{'x':1.5579999685287476,'y':-7.251999855041504},'size':{'x':1.0406076908111572,'y':2.311880111694336}}}
  point=[2.08661996,-6.88528839,.82247171]
  self.assertFalse(within_source(point,table,0.))
  self.assertTrue(within_source(point,table))
  self.assertFalse(within_source([2.12,point[1],point[2]],table))
  self.assertFalse(within_source([float('nan'),point[1],point[2]],table))
