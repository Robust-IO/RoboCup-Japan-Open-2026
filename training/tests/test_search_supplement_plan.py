import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from search_supplement_plan import supplement_views
from search_view_evidence import ViewEvidence
from search_point_sequence import SearchPointSequence


class SupplementTests(unittest.TestCase):
    def test_only_distinct_inside_room_backups(self):
        rooms={'living_room':[[0,0],[4,0],[4,4],[0,4]],'lobby':[[4,0],[8,0],[8,4],[4,4]]}
        point=lambda x,y:dict(x=x,y=y,yaw=0)
        row=dict(main=point(1,1),backups=[point(1.1,1),point(5,1),point(2.5,2.5),point(3,1)])
        views=supplement_views(row,rooms,'living_room',1)
        self.assertEqual(len(views),8)
        self.assertTrue(all(v['x']==2.5 and v['y']==2.5 for v in views))
        self.assertEqual(supplement_views(row,rooms,'living_room',0),[])
        self.assertEqual(len(supplement_views(row,rooms,'living_room',2)),16)

    def test_outside_candidate_continues_without_negative_claim(self):
        e=ViewEvidence();e.arrived(100)
        for i in range(1,4):
            e.observe(dict(depth_stamp_ns=100+i,inference=dict(view_health=dict(data_usable=True))),
                dict(state='observe',adapter_reason='target_room_outside_target_room',
                     room_membership=dict(state='outside_target_room')),i*.1)
        evidence=e.snapshot(.4)
        self.assertEqual(evidence['negative_stamps'],[])
        self.assertEqual(len(evidence['outside_room_stamps']),3)
        s=SearchPointSequence('t','can','map',['a','b'])
        result=s.complete_observation(dict(task_id='t',target='can',observer_context=dict(task_id='t',point_id='a',map_sha256='map'),
            state='incomplete',failures=['0:observation_timeout'],terminal=True,cleanup_verified=True,
            actionable=False,does_not_exist_authorized=False,view_evidence=evidence))
        self.assertEqual(result['point_id'],'b')
        self.assertFalse(result['does_not_exist_authorized'])

    def test_ambiguous_and_unhealthy_not_continuation(self):
        for reason in ['target_room_boundary_uncertain','target_room_transform_unavailable','missing_tf']:
            e=ViewEvidence();e.arrived(100)
            for i in range(1,4):e.observe(dict(depth_stamp_ns=100+i),dict(state='observe',adapter_reason=reason),i*.1)
            self.assertFalse(e.snapshot(.4)['view_data_usable'])
