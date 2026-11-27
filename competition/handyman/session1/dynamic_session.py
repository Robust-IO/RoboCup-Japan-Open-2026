"""Instruction-driven state machine; absence requires completed live room search."""
from dataclasses import replace
from session1 import Session1,Task
class DynamicSession(Session1):
    STEPS=('resolve_task','navigate_room','locate_source','observe_target','grasp','tuck','navigate_destination','place','verify_placement')
    def __init__(self,session_id,*,profile,**kwargs):
        super().__init__(session_id)
        self.steps=self.STEPS;self.task=Task(layout=profile['layout']);self.instruction=None;self.revision=0;self.reported_room=None
    def moderator(self,message,detail='',*,session_id):
        if session_id!=self.session_id or self.state in self.TERMINAL:return
        if message in ('Instruction','Corrected_instruction'):
            if detail==self.instruction:return
            if self.state not in ('waiting_instruction','waiting_correction'):
                self.stop('instruction_changed_during_active_action');return
            self.instruction=detail;self.revision+=1;self.evidence={};self._request('resolve_task');return
        super().moderator(message,detail,session_id=session_id)
    def _request(self,stage):
        super()._request(stage)
        self.effects[-1].update(instruction=self.instruction,task_revision=self.revision)
    def result(self,operation_id,result,*,session_id,run_id):
        if session_id!=self.session_id or run_id!=self.run_id or operation_id!=self.pending:return
        if result.get('status')!='ok':self.stop(result.get('reason','dynamic_operation_failed'));return
        if self.state=='resolve_task':
            t=result['task'];self.task=Task(layout=self.task.layout,target=t['target'],room=t['source_room'],source_table=t.get('source_table'),destination=t['destination']['furniture_id'],destination_room=t['destination']['room'])
            self.pending=None;self.evidence['resolve_task']=result;self._request('locate_source' if self.reported_room==self.task.room else 'navigate_room');return
        if self.state=='locate_source':
            if result.get('target')!=self.task.target or result.get('room')!=self.task.room:self.stop('source_search_identity_mismatch');return
            self.pending=None;self.evidence['locate_source']=result
            if result.get('confirmed') and result.get('source_table') and result.get('evidence_ref'):
                self.task=replace(self.task,source_table=result['source_table']);self._request('observe_target');return
            if (result.get('search_exhausted') is True and result.get('valid_views')==result.get('required_views') and result.get('required_views',0)>0 and result.get('unresolved_candidates')==0 and result.get('evidence_ref')):
                self.emit('protocol',message='Does_not_exist',detail='');self.state='waiting_correction';return
            self.stop('room_search_incomplete');return
        if self.state=='place' and self.task.destination=='Moderator':
            if not (result.get('delivery_ready') and result.get('holding_verified') and result.get('release_sent') is False):self.stop('held_delivery_not_verified');return
            self.pending=None;self.evidence['place']=result;self.emit('protocol',message='Task_finished',detail='');self.state='waiting_moderator';return
        if self.state=='navigate_room' and result.get('arrived') is True and result.get('room')==self.task.room:self.reported_room=self.task.room
        super().result(operation_id,result,session_id=session_id,run_id=run_id)
