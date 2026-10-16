import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from rgbd_tf_wait import ExactStampWait

class WaitTests(unittest.TestCase):
    def test_delayed_exact_stamp(self):
        seen=[]
        def lookup(stamp):
            seen.append(stamp)
            if len(seen)==1: raise RuntimeError('future extrapolation')
            return {'ok':True}
        wait=ExactStampWait(123,10)
        self.assertEqual(wait.poll(lookup,10)[0],'waiting')
        self.assertEqual(wait.poll(lookup,10.1)[0],'ready')
        self.assertEqual(seen,[123,123])
    def test_timeout_no_latest_fallback(self):
        wait=ExactStampWait(123,10)
        self.assertEqual(wait.poll(lambda _: self.fail('lookup after deadline'),10.5)[0],'timeout')
    def test_stale_rejected_before_lookup(self):
        self.assertEqual(ExactStampWait(123,10).poll(lambda _:self.fail('stale lookup'),10.1,False)[0],'expired')
    def test_repeated_failure_bounded(self):
        def lookup(_): raise RuntimeError('missing frame')
        wait=ExactStampWait(123,10)
        for now in [10,10.1,10.4]:
            self.assertEqual(wait.poll(lookup,now)[0],'waiting')
        self.assertEqual(wait.poll(lookup,10.6),('timeout',None,'missing frame'))
    def test_configuration(self):
        for stamp,start,timeout in [(0,10,.5),(1,float('nan'),.5),(1,10,0),(1,10,3)]:
            with self.assertRaises(ValueError): ExactStampWait(stamp,start,timeout)

if __name__=='__main__': unittest.main()
