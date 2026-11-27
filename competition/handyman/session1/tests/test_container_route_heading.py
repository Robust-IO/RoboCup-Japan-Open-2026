import unittest,math
from types import SimpleNamespace
from container_route_heading import face_container_route
class RouteHeadingTests(unittest.TestCase):
 def fixture(self,yaw):
  calls=[];state={'poses':{'map':[0,0,yaw]},'joints':[0,0,0,-1.57,0,.4]}
  def adjust(*args):
   calls.append(args);state["poses"]["map"][2]+=args[2][2]
  stages=SimpleNamespace(runtime=SimpleNamespace(sample=lambda **k:None,state=lambda:state),
    motion=SimpleNamespace(adjust=adjust),grasp_backend=SimpleNamespace(record=lambda e:None))
  return stages,calls
 def test_route_left_of_robot_turns_before_release(self):
  s,c=self.fixture(0)
  face_container_route(s,{'polygon_map_m':[[-.1,1.9],[.1,1.9],[.1,2.1],[-.1,2.1]]})
  self.assertEqual(c[0][2][:2],[0.,0.]);self.assertAlmostEqual(c[0][2][2],math.pi/2)
 def test_aligned_route_does_not_add_rotation(self):
  s,c=self.fixture(math.pi/2)
  face_container_route(s,{'polygon_map_m':[[-.1,1.9],[.1,1.9],[.1,2.1],[-.1,2.1]]})
  self.assertEqual(c,[])

 def test_escape_translation_recomputes_bearing(self):
  s,c=self.fixture(0);state=s.runtime.state()
  def adjust(*args):
   c.append(args)
   if len(c)==1:state['poses']['map'][0]=.2
   state['poses']['map'][2]+=args[2][2]
  s.motion.adjust=adjust
  face_container_route(s,{'polygon_map_m':[[-.1,1.9],[.1,1.9],[.1,2.1],[-.1,2.1]]})
  self.assertEqual(len(c),2)
  self.assertAlmostEqual(state['poses']['map'][2],math.atan2(2,-.2))
