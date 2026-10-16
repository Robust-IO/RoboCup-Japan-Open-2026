import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from probe_rgbd_streams import StreamStats,closest

class ProbeTests(unittest.TestCase):
    def test_counts(self):
        s=StreamStats()
        for ns in (100,100,90,110):s.add(ns,200,{'frame':'camera'})
        self.assertEqual((s.count,s.duplicates,s.backward),(4,1,1))
        self.assertEqual((s.first,s.last),(100,110))
    def test_nearest_not_latest(self):self.assertEqual(closest([100,120,160],122),120)
    def test_missing(self):self.assertIsNone(closest([],100))
    def test_clock_ahead_visible(self):
        s=StreamStats();s.add(2_000_000_000,1_000_000_000,{})
        self.assertEqual(s.min_age,-1)

if __name__=='__main__':unittest.main()
