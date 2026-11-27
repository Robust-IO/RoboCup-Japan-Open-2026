"""Session3 corrected-target grasp and held delivery; official success comes from Moderator."""
from dataclasses import replace
from session1 import Session1,Task
class Session3(Session1):
    STEPS=('navigate_room','search','navigate_source','observe_target','grasp','tuck','navigate_moderator','handover')
    TERMINAL=Session1.TERMINAL
    def __init__(self,session_id,*,profile,**kw):
        super().__init__(session_id)
        self.profile=profile;self.task=Task(layout=profile.get('layout','LayoutA'),target=profile.get('target','bear_doll'),room=profile.get('corrected_source',{}).get('room','lobby'),destination='Moderator',destination_room=profile.get('destination',{}).get('room','lobby'))
        self.steps=self.STEPS;self.corrected=False
        self.instruction=profile['expected_instruction']
    def moderator(self,message,detail='',*,session_id):
        if session_id!=self.session_id or self.state in self.TERMINAL:return
        norm=lambda s:' '.join(s.lower().split()).rstrip('.')
        if message=='Corrected_instruction':
            if self.corrected and norm(detail)==norm(self.profile['corrected_instruction']):return
            if self.state!='waiting_correction' or norm(detail)!=norm(self.profile['corrected_instruction']):
                self.stop('unexpected_corrected_instruction');return
            self.corrected=True;self.task=replace(self.task,target=self.profile.get('corrected_target','dog_doll'),source_table=self.profile.get('corrected_source',{}).get('furniture_id','corner_sofa#lobby'))
            self.evidence.pop('search',None);self._request('navigate_source');return
        if message=='Instruction' and self.corrected:return
        super().moderator(message,detail,session_id=session_id)
    def result(self,operation_id,result,*,session_id,run_id):
        if (session_id!=self.session_id or run_id!=self.run_id or self.pending is None or operation_id!=self.pending):return
        if result.get('status')!='ok':self.stop(result.get('reason','search_operation_failed'));return
        stage=self.state
        if stage in ('navigate_source','observe_target','grasp'):
            return super().result(operation_id,result,session_id=session_id,run_id=run_id)
        if stage=='tuck':
            if result.get('holding_verified') is True and result.get('carry_ready') is True:
                self.pending=None;self.evidence[stage]=result;self._request('navigate_moderator')
            else:self.stop('carry_not_verified')
            return
        if stage=='navigate_moderator':
            if result.get('destination')!='Moderator' or result.get('arrived') is not True:
                self.stop('moderator_arrival_missing');return
            self.pending=None;self.evidence[stage]=result;self._request('handover');return
        if stage=='handover':
            if (result.get('target')!=self.task.target or result.get('destination')!='Moderator'
                or result.get('delivery_ready') is not True or result.get('holding_verified') is not True
                or result.get('release_sent') is not False or not result.get('evidence_ref')):
                self.stop('held_delivery_not_verified');return
            self.pending=None;self.evidence[stage]=result
            self.emit('protocol',message='Task_finished',detail='');self.state='waiting_moderator';return
        if stage=='navigate_room':
            if result.get('room')!=self.task.room or result.get('arrived') is not True:self.stop('lobby_arrival_missing');return
            self.pending=None;self.evidence[stage]=result;self.emit('protocol',message='Room_reached',detail='');self._request('search');return
        if stage!='search' or result.get('target')!=self.task.target or result.get('room')!=self.task.room or not result.get('evidence_ref'):
            self.stop('search_evidence_mismatch');return
        self.pending=None;self.evidence['search']=result
        if result.get('confirmed') is True:
            if not self.corrected:self.stop('initial_target_found_conflicts_with_fixed_session3');return
            self.stop('unexpected_corrected_full_room_search');return
        if result.get('search_exhausted') is True and result.get('valid_views')==result.get('required_views') and result.get('required_views',0)>0 and result.get('unresolved_candidates')==0:
            self.emit('protocol',message='Does_not_exist',detail='')
            self.state='waiting_moderator' if self.corrected else 'waiting_correction'
        else:self.stop('room_search_incomplete')
