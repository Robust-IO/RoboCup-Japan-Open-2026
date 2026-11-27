"""Bind parsed tasks to existing navigation, RGBD search, grasp and delivery."""
import json,math
from pathlib import Path
from copy import deepcopy
import numpy as np
import yaml
from instruction_task import parse_instruction,source_route
from execution_plan import furniture_candidates
from build_room_inventory import bind_point

class DynamicBindings:
    def __init__(self,bundle,stages):
        self.b=bundle;self.stages=stages;self.cfg=bundle['config'];self.rt=bundle['runtime'];self.backend=bundle['grasp_backend'];self.root=Path(__file__).resolve().parents[3]
        self.number=self.cfg['session_profile']['session_number'];self.transfer=None;self.search=None
        names=yaml.safe_load((self.root/'project/training/dataset.yaml').read_text())['names'];self.classes=list(names.values()) if isinstance(names,dict) else names
        path=self.root/'config/room-inventories'/f'session{self.number}.json'
        self.inventory=json.loads(path.read_text()) if path.exists() else {'complete':False,'objects':[]}
        if self.inventory.get('layout',self.cfg['layout'])!=self.cfg['layout']:raise ValueError('inventory_layout_mismatch')
    def set_source(self,fid):
        self._confirmed_source_observation=None
        s=self.stages;table=next(t for t in self.b['semantic']['destinations'] if t['id']==fid)
        from source_view_candidates import source_candidates
        s.source_id=fid;s.source_room=table['room']
        s.source_candidates=source_candidates(self.cfg,self.b['environment'],table)
        s.source_goal=s.source_candidates[0]
        self.cfg['source_table_id']=fid;self.cfg['task']['source_room']=table['room']
        self.cfg['session_profile']['source']={'room':table['room'],'furniture_id':fid}
        self.backend.table=table;self.backend.identity_confirmed=False;self.backend.pose_locked=False
    def resolve(self,effect,cancel):
        self.stages.check(cancel)
        self._confirmed_source_observation=None
        t=parse_instruction(effect['instruction'],self.classes,self.b['environment']['rooms'],self.b['semantic']['destinations'])
        dest=t['destination'];mode=dest['mode'];self.geometry=None
        if mode=='container_drop':
            catalog=json.loads((self.root/'config/container-geometries.json').read_text());key=self.cfg['layout']+'|'+dest['furniture_id']
            if key not in catalog:raise ValueError('calibrated_container_geometry_required:'+key)
            self.geometry=catalog[key]
        if mode=='avatar_handover':
            reference=json.loads((Path(__file__).parent/'session3-search-config.json').read_text())
            if reference['layout']!=self.cfg['layout']:raise ValueError('moderator_geometry_for_layout_required')
            from handover_config import validate_handover
            validate_handover(reference,check_sources=True)
            self.cfg['handover']=deepcopy(reference['handover']);dest['room']=reference['destination_room']
        self.task=t;self.route=source_route(t,self.inventory);self.transfer=None
        self.cfg.update(task={'object':t['target'],'source_room':t['source_room'],'destination':dest['furniture_id'].split('#')[0]},destination_id=dest['furniture_id'],destination_room=dest['room'])
        self.cfg['session_profile']['destination']=deepcopy(dest)
        self.cfg['session_profile']['target']=t['target']
        s=self.stages;s.target=t['target'];s.source_room=t['source_room'];s.destination_id=dest['furniture_id'];s.context=None
        s.destination_goal=None if mode in ('avatar_handover','container_drop') else furniture_candidates(self.b['environment'],dest['furniture_id'])[0]
        self.backend.identity_confirmed=False;self.backend.pose_locked=False;self.backend.target_name=t['target']
        from object_grasp_profiles import apply as apply_object_grasp_profile
        apply_object_grasp_profile(self.backend,t['target'])
        if self.geometry:self.cfg.update(container_opening=deepcopy(self.geometry['opening']),container_destination=deepcopy(self.geometry['destination']),container_geometry_pending=False)
        points=self.b['environment']['rooms'][t['source_room']]['search_points']
        if not points:raise ValueError('room_search_points_missing')
        self.cfg['search_plan']={'points':deepcopy(points[:2]),'source':'imported_room_search_points'}
        self.cfg['session_profile']['search']={'continuous_bins':12,'continuous_speed_rad_s':.4,'candidate_confidence':.15,'confirmed_confidence':.8,'frames_per_view':2}
        from session3_search_live import Session3Search
        self.search=Session3Search(self.b)
        self.search.serial=effect["task_revision"]*1000
        path=s.output/f"resolved-task-{effect['task_revision']}.json"
        path.write_text(json.dumps({'task':t,'source_route':self.route,'inventory_assumption':self.inventory.get('assumption'),'robot_commands_sent':False},indent=2))
        return dict(status='ok',task=t,evidence_ref=str(path))
    def navigate_room(self,effect,cancel):
        self.stages.motion.stage='navigate_room'
        return self.search.navigate_room(effect,cancel)
    def candidate_ids(self):
        from source_view_candidates import source_candidates
        ids=[];poses={}
        for f in self.b['semantic']['destinations']:
            if f['room']!=self.task['source_room']:continue
            try:poses[f['id']]=source_candidates(self.cfg,self.b['environment'],f)
            except ValueError:continue
            ids.append(f['id'])
        self.rt.sample(profile='state');xy=self.rt.state()['poses']['map'][:2]
        def distance(fid):return min(math.dist(xy,p[:2]) for p in poses[fid])
        return sorted(ids,key=distance)
    def confirm_furniture(self,fid,effect,cancel):
        from furniture_reobservation import _observe_cycle
        from grasp_runtime.autograsp.coordinator import AttemptFailure
        from grasp_runtime.autograsp.recovery import Failure
        self.set_source(fid);s=self.stages
        s.navigate_source(dict(effect,stage='navigate_source'),cancel)
        # Same current view, head sweep and lateral views as fixed tasks.
        # One complete cycle per furniture allows dynamic room-search fallback.
        try:
            result=_observe_cycle(s,effect,cancel)
        except AttemptFailure as exc:
            if exc.failure!=Failure.OCCLUDED:raise
            return None
        if result.get('confirmed') is not True:return None
        self._confirmed_source_observation=(self.task['target'],fid,result)
        self.backend.record(dict(event='dynamic_source_observed_with_fixed_pipeline',
                                 target=self.task['target'],source_table=fid))
        return result
    def observe_target(self,effect,cancel):
        self.stages.check(cancel)
        cached=getattr(self,'_confirmed_source_observation',None)
        self._confirmed_source_observation=None
        if (cached is not None and cached[:2]==(self.task['target'],self.stages.source_id)
                and getattr(self.backend,'scene',None) is not None
                and getattr(self.backend,'source_snapshot',None) is not None):
            # No navigation or head reset between furniture confirmation and
            # grasp preparation. The shared grasp executor still reacquires
            # current RGBD for approach/alignment/contact checks.
            self.backend.record(dict(event='dynamic_source_observation_handoff',
                                     target=cached[0],source_table=cached[1],duplicate_observation=False))
            return cached[2]
        return self.stages.observe_target(effect,cancel)
    def locate(self,effect,cancel):
        seen=set()
        candidates=list(self.route['furniture_ids'])
        if self.route['route']!='room_search':candidates+=self.candidate_ids()
        for fid in candidates:
            if fid in seen:continue
            seen.add(fid);self.stages.check(cancel)
            try:result=self.confirm_furniture(fid,effect,cancel)
            except ValueError as exc:
                if not str(exc).startswith(('no_reachable_unobstructed_candidate:', 'navigation_candidate_exhausted')):raise
                self.backend.record(dict(event='dynamic_candidate_unreachable',furniture_id=fid,reason=str(exc),absence_authorized=False))
                continue
            if result:return result
        # Static inventory never produces absence. Only completed live scans do.
        while True:
            self.stages.check(cancel);self.stages.motion.stage='search'
            result=self.search.search(effect,cancel)
            if result.get('confirmed'):
                fid=bind_point(result['position_map_m'],self.task['source_room'],self.b['semantic'])
                if fid:
                    self.set_source(fid)
                    return dict(result,source_table=fid)
                self.backend.record(dict(event='dynamic_found_target_support_unresolved',position_map_m=result['position_map_m'],absence_authorized=False))
                raise ValueError('observed_target_support_requires_binding')
            if result.get('search_exhausted'):return result
            self.backend.record(dict(event='dynamic_room_search_incomplete_retry',absence_authorized=False))
    def make_transfer(self):
        if self.transfer is not None:return self.transfer
        s=self.stages
        from withdrawal_observer import WithdrawalObserver
        from table_withdrawal import TableWithdrawal
        from transfer_bindings import TransferBindings
        from grasp_runtime.autograsp.kinematics import forward
        from support_reference import assess_bottom
        from session_placement_policy import relaxed_support_policy
        from outcome_transfer_policy import load
        import table_transfer_core as core
        import task_outcome_policy as policy
        mode=self.task['destination']['mode']
        if mode=='avatar_handover':
            from handover_live import HandoverLive
            self.transfer=HandoverLive(s);return self.transfer
        dest=next(t for t in self.b['semantic']['destinations'] if t['id']==s.destination_id)
        _,rotation=forward([.25,core.FLEX,0.,core.WRIST,0.,.3])
        transfer=TransferBindings(s,dest,rotation,core=core,policy=policy,
            withdraw_source=TableWithdrawal(self.rt,s.motion,WithdrawalObserver(self.backend,self.backend.table,lambda:s.context),label='source-withdraw'),
            withdraw_empty=TableWithdrawal(self.rt,s.motion,WithdrawalObserver(self.backend,dest,lambda:None),label='empty-withdraw'),
            bottom_evidence=lambda c:assess_bottom(c,rotation)['established'],outcome_policy=relaxed_support_policy(load()))
        if mode=='container_drop':
            from container_bindings import ContainerBindings
            from container_live_adapter import ContainerLiveAdapter
            transfer=ContainerBindings(s,transfer,ContainerLiveAdapter)
        self.transfer=transfer;return transfer
    def tuck(self,effect,cancel):
        if self.task['destination']['mode']!='avatar_handover':return self.make_transfer().tuck(effect,cancel)
        from transfer_actions import tuck
        from hold_observer import HoldObserver
        from withdrawal_observer import WithdrawalObserver
        from table_withdrawal import TableWithdrawal
        self.stages.context['require_visual_carry_checks']=False
        return tuck(self.rt,observe_hold=HoldObserver(self.backend,self.stages.context),withdraw_from_source=TableWithdrawal(self.rt,self.stages.motion,WithdrawalObserver(self.backend,self.backend.table,lambda:self.stages.context),label='dynamic-source-withdraw'))
    def navigate_destination(self,effect,cancel):
        if self.task['destination']['mode']=='avatar_handover':return self.make_transfer().navigate(effect,cancel)
        return self.stages.navigate_destination(effect,cancel)
    def place(self,effect,cancel):
        t=self.make_transfer()
        return t.present(effect,cancel) if self.task['destination']['mode']=='avatar_handover' else t.place(effect,cancel)
    def handlers(self):
        return dict(resolve_task=self.resolve,navigate_room=self.navigate_room,locate_source=self.locate,observe_target=self.observe_target,grasp=self.stages.grasp,tuck=self.tuck,navigate_destination=self.navigate_destination,place=self.place,verify_placement=lambda e,c:self.make_transfer().verify_placement(e,c))
