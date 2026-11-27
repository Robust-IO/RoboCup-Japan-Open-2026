import unittest
from startup_clock_ready import ready_window
class ClockTests(unittest.TestCase):
 def rows(self,age=.02):return [(i*.1,1000000000+i*100000000,age) for i in range(30)]
 def test_normal(self):self.assertTrue(ready_window(self.rows(),3.))
 def test_future_not_hidden(self):self.assertFalse(ready_window(self.rows(-.16),3.))
 def test_past_delay_not_hidden(self):self.assertFalse(ready_window(self.rows(.8),3.))
 def test_stopped_input(self):self.assertFalse(ready_window(self.rows(),4.))
 def test_repeated_or_backwards_stamp(self):
  r=self.rows();r[-1]=(r[-1][0],r[0][1],.02);self.assertFalse(ready_window(r,3.))
 def test_one_fresh_sample_insufficient(self):self.assertFalse(ready_window(self.rows()[-1:],3.))
