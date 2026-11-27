import unittest
from session1 import Session1, INSTRUCTION

GOOD = {
 'navigate_room': dict(status='ok',room='living_room',arrived=True),
 'search': dict(status='ok',room='living_room',target='dog_doll',confirmed=True,evidence_ref='synthetic/rgbd'),
 'grasp': dict(status='ok',target='dog_doll',grasp_verified=True),
 'tuck': dict(status='ok',holding_verified=True,carry_ready=True),
 'navigate_destination': dict(status='ok',destination='square_low_table#living_room',arrived=True),
 'place': dict(status='ok',release_sent=True),
 'verify_placement': dict(status='ok',target='dog_doll',destination='square_low_table#living_room',placement_verified=True),
}


class TestSession1(unittest.TestCase):
    def ready(self, source_table=None):
        s=Session1('s1',source_table=source_table);s.preflight(ready=True)
        s.moderator('Are_you_ready?',session_id='s1')
        self.assertEqual(s.drain(),[])
        s.moderator('Environment','LayoutA',session_id='s1')
        self.assertEqual(s.drain()[0]['message'],'I_am_ready')
        s.moderator('Instruction',INSTRUCTION,session_id='s1')
        return s

    def complete(self,s,result=None):
        s.result(s.pending,GOOD[s.state] if result is None else result,
                 session_id='s1',run_id=s.run_id)

    def test_full_flow_waits_for_moderator(self):
        s=self.ready();messages=[];operations=[]
        for stage in s.STEPS:
            self.assertEqual(s.state,stage)
            for effect in s.drain():
                if effect['kind']=='operation':operations.append(effect['stage'])
                elif effect['kind']=='protocol':messages.append(effect['message'])
            self.complete(s)
        messages.extend(e['message'] for e in s.drain() if e['kind']=='protocol')
        self.assertEqual(operations,list(s.STEPS))
        self.assertEqual(messages,['Room_reached','Object_grasped','Task_finished'])
        self.assertEqual(s.state,'waiting_moderator')
        s.moderator('Task_succeeded',session_id='s1');self.assertEqual(s.state,'succeeded')

    def test_every_stage_failure_stops_without_success_publication(self):
        for failed in Session1.STEPS:
            with self.subTest(failed=failed):
                s=self.ready()
                while s.state!=failed:self.complete(s);s.drain()
                s.drain();self.complete(s,dict(status='incomplete',reason='test_failure'))
                self.assertEqual(s.state,'stopped')
                self.assertEqual([e['kind'] for e in s.drain()],['stop'])

    def test_evidence_not_optional(self):
        for stage in Session1.STEPS:
            with self.subTest(stage=stage):
                s=self.ready()
                while s.state!=stage:self.complete(s);s.drain()
                s.drain();self.complete(s,dict(status='ok'))
                self.assertEqual(s.state,'stopped')

    def test_wrong_room_target_destination_rejected(self):
        for stage,key in [('navigate_room','room'),('search','room'),('search','target'),('grasp','target'),('navigate_destination','destination'),('verify_placement','destination')]:
            with self.subTest(stage=stage,key=key):
                s=self.ready()
                while s.state!=stage:self.complete(s);s.drain()
                self.complete(s,{**GOOD[stage],key:'wrong'})
                self.assertEqual(s.state,'stopped')

    def test_duplicates_and_old_results(self):
        s=self.ready();old=s.pending;s.drain()
        s.moderator('Instruction',INSTRUCTION,session_id='s1');self.assertEqual(s.drain(),[])
        self.complete(s);s.drain()
        s.result(old,GOOD['navigate_room'],session_id='s1',run_id=s.run_id)
        self.assertEqual(s.state,'search');self.assertEqual(s.drain(),[])

    def test_cross_session_and_run_ignored(self):
        s=self.ready();s.drain()
        s.result(s.pending,GOOD[s.state],session_id='old',run_id=s.run_id)
        s.result(s.pending,GOOD[s.state],session_id='s1',run_id='old')
        s.moderator('Task_failed',session_id='old')
        self.assertEqual(s.state,'navigate_room');self.assertEqual(s.drain(),[])

    def test_cancel_discards_queued_motion(self):
        s=self.ready();old=s.pending;s.stop('cancel','cancelled')
        self.assertEqual([e['kind'] for e in s.drain()],['stop'])
        s.result(old,GOOD['navigate_room'],session_id='s1',run_id=s.run_id)
        self.assertEqual(s.state,'cancelled');self.assertEqual(s.drain(),[])

    def test_unexpected_instruction_or_layout_stops(self):
        for message,detail in [('Environment','LayoutB'),('Instruction','fetch apple'),('Corrected_instruction',INSTRUCTION)]:
            s=self.ready();s.moderator(message,detail,session_id='s1');self.assertEqual(s.state,'stopped')

    def test_moderator_failure_overrides_pending_action(self):
        s=self.ready();s.moderator('Task_failed','Time_is_up',session_id='s1')
        self.assertEqual(s.state,'failed');self.assertEqual(s.reason,'Time_is_up')
        self.assertEqual([e['kind'] for e in s.drain()],['stop'])

    def test_release_alone_never_finishes_task(self):
        s=self.ready()
        while s.state!='place':self.complete(s);s.drain()
        self.complete(s)
        self.assertEqual(s.state,'verify_placement')
        self.assertNotIn('Task_finished',[e.get('message') for e in s.drain()])

    def test_early_success_is_not_accepted(self):
        s=self.ready();s.moderator('Task_succeeded',session_id='s1')
        self.assertEqual(s.state,'stopped')

    def test_preflight_required(self):
        s=Session1('s1');s.moderator('Environment','LayoutA',session_id='s1')
        s.moderator('Are_you_ready?',session_id='s1')
        self.assertEqual(s.drain(),[])
        s.preflight(ready=True);self.assertEqual(s.drain()[0]['message'],'I_am_ready')

    def test_known_source_skips_search_but_requires_observation(self):
        source='white_side_table#living_room'
        s=self.ready(source);stages=[];messages=[]
        extra={
            'navigate_source':dict(status='ok',source_table=source,room='living_room',arrived=True),
            'observe_target':dict(status='ok',source_table=source,room='living_room',target='dog_doll',confirmed=True,evidence_ref='synthetic/new-rgbd'),
        }
        while s.state!='waiting_moderator':
            for e in s.drain():
                if e['kind']=='operation':
                    stages.append(e['stage']);self.assertEqual(e['task']['source_table'],source)
                elif e['kind']=='protocol':messages.append(e['message'])
            self.complete(s,extra.get(s.state,GOOD.get(s.state)))
        messages += [e['message'] for e in s.drain() if e['kind']=='protocol']
        self.assertEqual(stages,list(s.steps))
        self.assertNotIn('search',stages)
        self.assertLess(stages.index('observe_target'),stages.index('grasp'))
        self.assertEqual(messages,['Room_reached','Object_grasped','Task_finished'])

    def test_known_source_wrong_room_table_rejected(self):
        s=self.ready('white_side_table#living_room');self.complete(s);s.drain()
        self.complete(s,dict(status='ok',room='bedroom',source_table='white_side_table#bedroom_1',arrived=True))
        self.assertEqual(s.state,'stopped')

    def test_known_source_no_detection_does_not_grasp(self):
        s=self.ready('white_side_table#living_room');self.complete(s)
        self.complete(s,dict(status='ok',room='living_room',source_table='white_side_table#living_room',arrived=True))
        s.drain();self.complete(s,dict(status='ok',confirmed=False))
        self.assertEqual(s.state,'stopped');self.assertEqual(s.drain()[0]['kind'],'stop')


if __name__=='__main__':unittest.main()
