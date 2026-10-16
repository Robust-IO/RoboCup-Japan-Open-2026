import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from unreachable_policy import decide_room,decide_absence,UnreachablePolicy

class UnreachableTests(unittest.TestCase):
    def evidence(self,**kw):
        e=dict(schema='handyman-structural-reachability-v1',room='kitchen',map_sha256='a'*64,
               status='structurally_unreachable',method='static_configuration_space_connectivity',
               all_room_access_candidates_checked=True,unknown_space_affects_result=False,
               transient_obstacles_used=False,start_component_valid=True,analysis_id='test-only',robot_footprint_id='test-only')
        e.update(kw); return e
    def decide(self,e=None,requested='kitchen',enabled=True):
        return decide_room(room='kitchen',requested_room=requested,map_sha256='a'*64,
                           evidence=self.evidence() if e is None else e,policy=UnreachablePolicy(enabled))
    def test_explicit_target_policy(self):
        r=self.decide(); self.assertEqual(r['protocol_response'],'Does_not_exist'); self.assertFalse(r['actionable'])
    def test_general_search_skip(self):
        r=self.decide(requested=None); self.assertEqual(r['decision'],'skip_room'); self.assertIsNone(r['protocol_response'])
    def test_disabled(self): self.assertEqual(self.decide(enabled=False)['decision'],'task_incomplete')
    def test_fail_closed(self):
        for change in [dict(status='navigation_failed'),dict(map_sha256='b'*64),dict(room='bedroom'),
                       dict(all_room_access_candidates_checked=False),dict(unknown_space_affects_result=True),
                       dict(transient_obstacles_used=True),dict(start_component_valid=False),dict(robot_footprint_id=''),
                       dict(all_room_access_candidates_checked=1),dict(method='manual_exclusion')]:
            with self.subTest(change=change):
                self.assertIsNone(self.decide(self.evidence(**change))['protocol_response'])
    def test_exclusion_only(self): self.assertEqual(self.decide({'excluded':True})['decision'],'unresolved')

class SearchAbsenceTests(unittest.TestCase):
    def evidence(self,**kw):
        e=dict(schema='handyman-room-search-evidence-v1',task_id='task1',target='apple',room='kitchen',
               map_sha256='a'*64,room_reachable=True,coverage_verified=True,perception_valid=True,
               target_found=False,search_complete=True,required_view_ids=['v1','v2'],completed_view_ids=['v1','v2'],
               failed_view_ids=[],skipped_view_ids=[],coverage_analysis_id='synthetic-test')
        e.update(kw); return e
    def check(self,e):
        return decide_absence(task_id='task1',target='apple',room='kitchen',map_sha256='a'*64,search_evidence=e)
    def test_complete(self):
        self.assertEqual(self.check(self.evidence())['protocol_response'],'Does_not_exist')
    def test_incomplete_rejected(self):
        for kw in [dict(coverage_verified=False),dict(perception_valid=False),dict(target_found=True),
                   dict(completed_view_ids=['v1']),dict(skipped_view_ids=['v2']),dict(failed_view_ids=['v2']),
                   dict(task_id='old'),dict(target='canned_juice'),dict(map_sha256='b'*64),
                   dict(search_complete=False),dict(required_view_ids=[]),dict(room_reachable=False)]:
            with self.subTest(kw=kw): self.assertIsNone(self.check(self.evidence(**kw))['protocol_response'])
    def test_plan_exhausted_is_not_evidence(self):
        self.assertIsNone(self.check({'state':'plan_exhausted'})['protocol_response'])

if __name__=='__main__': unittest.main()
