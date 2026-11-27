import unittest
from door_approach_heading import select_heading
class DoorHeadingTests(unittest.TestCase):
 def test_cabinet_replay(self):self.assertEqual(select_heading(2.88,[.6293,-2.2405,2.846],[.511151,-2.208804,1.57],True),1.57)
 def test_furniture_unchanged(self):self.assertEqual(select_heading(2.88,[.12,0,0],[0,0,1.57],False),2.88)
 def test_far_approach_unchanged(self):self.assertEqual(select_heading(2.88,[.5,0,0],[0,0,1.57],True),2.88)
