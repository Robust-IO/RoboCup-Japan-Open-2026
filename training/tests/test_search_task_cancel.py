from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from search_arrival_ros import matches_task_cancel,cancel_observation,RetryBindings
from search_arrival import SearchArrival
from search_scan import SearchScan
from tf_motion_gate import TfSearchObserver


class CancelTests(unittest.TestCase):
    def test_matching(self):
        self.assertTrue(matches_task_cancel('search_cancelled',dict(schema='handyman-search-request-v1',task_id='1'*32),'1'*32))
    def test_wrong_task_or_event(self):
        row=dict(schema='handyman-search-request-v1',task_id='1'*32)
        self.assertFalse(matches_task_cancel('search_cancelled',row,'2'*32))
        self.assertFalse(matches_task_cancel('search_requested',row,'1'*32))
    def test_malformed(self):
        for row in (None,[],{},'bad'):
            self.assertFalse(matches_task_cancel('search_cancelled',row,'1'*32))
    def test_discards_all_observation_evidence(self):
        scan=SearchScan('task','apple',[dict(x=0,y=0,yaw=0)],0)
        arrival=SearchArrival(scan,'1'*32);tf=TfSearchObserver(arrival,0);retry=RetryBindings(2)
        scan.phase='observe';scan.position=(1,2,3);scan.negative=2
        arrival.anchor=(0,0,0);arrival.adapter=object();retry.pending=object()
        tf.adapter.motion.samples.append((1,0,0,0,0))
        result=cancel_observation(scan,arrival,tf,retry)
        self.assertEqual(result['state'],'cancelled');self.assertFalse(result['actionable'])
        self.assertIsNone(scan.position);self.assertEqual(scan.negative,0)
        self.assertIsNone(arrival.adapter);self.assertIsNone(arrival.anchor)
        self.assertFalse(tf.adapter.motion.samples);self.assertIsNone(retry.pending)
        self.assertFalse(retry.waiting)

if __name__=='__main__':unittest.main()
