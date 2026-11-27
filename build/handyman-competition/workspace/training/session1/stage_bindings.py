"""Concrete navigation/observation/grasp handlers using existing live backends.

Construct inside the single owning runtime. No ROS initialization at import.
Transfer handlers must be supplied; missing stages are never simulated as success.
"""
import json
from uuid import uuid4
from task_recovery import retry_read,grasp_observation_recovery
import math
from pathlib import Path
import numpy as np
from execution_plan import furniture_candidates,grasp_result,SOURCE,DESTINATION
from perception_bridge import observation_result
from grasp_runtime.autograsp.coordinator import Coordinator

class StageBindings:
    def __init__(self,runtime,motion_backend,grasp_backend,config,environment,output,*,
                 map_point=None,coordinator_factory=Coordinator,candidate_selector=None):
        if map_point is None:
            from apple_transfer_auto_plan import map_point
        self.runtime=runtime;self.motion=motion_backend;self.grasp_backend=grasp_backend
        self.config=config;self.environment=environment;self.output=Path(output)
        self.map_point=map_point;self.coordinator_factory=coordinator_factory
        self.candidate_selector=candidate_selector
        self.target=config.get('task',{}).get('object','dog_doll')
        self.source_id=config.get('source_table_id',SOURCE)
        self.source_room=config.get('task',{}).get('source_room','living_room')
        self.destination_id=config.get('destination_id',DESTINATION)
        from source_view_candidates import source_candidates
        self.context=None
        self.source_candidates=source_candidates(config,environment,grasp_backend.table)
        self.source_goal=self.source_candidates[0]
        self.destination_goal=(None if config.get('session_profile',{}).get('destination',{}).get('mode') in ('avatar_handover','container_drop') else furniture_candidates(environment,self.destination_id)[0])

    @staticmethod
    def check(cancel):
        if cancel.is_set():raise RuntimeError('task_cancelled')

    def state(self,cancel):
        self.check(cancel);retry_read(lambda:self.runtime.sample(profile='state'),cancel=cancel)
        state=self.runtime.state();self.check(cancel);return state

    def navigate(self,effect,cancel,goal,room):
        state=self.state(cancel);source=self.motion.room_at(state['poses']['map'])
        name='session1-'+effect['operation_id'].replace('/','-')
        self.motion.stage=effect['stage']
        outcome=self.motion.navigate(source,room,goal,float(state['joints'][5]),name)
        self.check(cancel)
        after=self.state(cancel)
        if outcome.get('verified') is not True or self.motion.room_at(after['poses']['map'])!=room:
            raise ValueError('navigation_or_room_not_verified')
        return dict(status='ok',arrived=True,room=room,evidence_ref=outcome['evidence'])

    def select_goal(self,pose,candidates,furniture_id,label):
        if self.candidate_selector is None:
            # Used only by isolated adapters/tests; live assembly supplies map selection.
            return min(candidates,key=lambda p:(p[0]-pose[0])**2+(p[1]-pose[1])**2)
        ranked,audit=self.candidate_selector.rank(pose,candidates,furniture_id)
        (self.output/(label+'.json')).write_text(json.dumps(audit,indent=2))
        if not ranked:raise ValueError('no_reachable_unobstructed_candidate:'+furniture_id)
        return ranked[0]

    def navigate_room(self,effect,cancel):
        state=self.state(cancel)
        from room_entry import entered
        if self.motion.room_at(state['poses']['map'])==self.source_room and entered(state['poses']['map'],{'polygon':self.environment['rooms'][self.source_room]['region']}):
            return dict(status='ok',arrived=True,room=self.source_room,basis='current_localization')
        self.source_goal=self.select_goal(state['poses']['map'],self.source_candidates,self.source_id,'room-entry-candidate-selection')
        from room_entry import finish_on_room_entry
        with finish_on_room_entry(self.motion,self.output,self.source_room,self.environment['rooms'][self.source_room]['region']):
            return self.navigate(effect,cancel,self.source_goal,self.source_room)

    def navigate_source(self,effect,cancel):
        state=self.state(cancel);pose=state['poses']['map']
        from room_entry import entered,finish_on_room_entry
        polygon=self.environment['rooms'][self.source_room]['region']
        table=self.grasp_backend.table
        spec=dict(mode='near_furniture',polygon=polygon,bounds=table['axis_aligned_bounds'],distance_m=1.0)
        if entered(pose,spec):
            return dict(status='ok',arrived=True,room=self.source_room,source_table=self.source_id,basis='already_near_furniture')
        self.source_goal=self.select_goal(pose,self.source_candidates,self.source_id,'source-candidate-selection')
        with finish_on_room_entry(self.motion,self.output,self.source_room,polygon,furniture=table):
            result=self.navigate(effect,cancel,self.source_goal,self.source_room)
        return dict(result,source_table=self.source_id)

    def face_source_furniture(self,effect,cancel):
        pose=self.state(cancel)['poses']['map']
        center=self.grasp_backend.table['axis_aligned_bounds']['center']
        dx,dy=center['x']-pose[0],center['y']-pose[1]
        if math.hypot(dx,dy)<1e-6:return
        yaw=math.atan2(dy,dx)
        error=math.atan2(math.sin(yaw-pose[2]),math.cos(yaw-pose[2]))
        if abs(error)>.10:
            turn=dict(effect,operation_id=effect['operation_id']+'-face-source-'+uuid4().hex)
            self.navigate(turn,cancel,[pose[0],pose[1],yaw],self.source_room)
        self.check(cancel)
        self.runtime.face_source((0.,0.))
        (self.output/'source-observation-facing.json').write_text(json.dumps(dict(
            furniture=self.source_id,start_pose=pose,goal_yaw=yaw,
            base_turn_requested=abs(error)>.10,basis='face_furniture_before_identity')))

    def observe_target(self,effect,cancel):
        from furniture_reobservation import observe_furniture
        return observe_furniture(self,effect,cancel)

    def _observe_target_once(self,effect,cancel,head=None):
        self.check(cancel);backend=self.grasp_backend
        if head is not None:self.runtime.face_source(head)
        self.check(cancel)
        scene=retry_read(lambda:backend.observe_stopped(self.target),cancel=cancel,record=getattr(backend,'record',None),
            repair=lambda attempt:self.runtime.face_source((0.,(-.3,-.5,-.7,0.)[(attempt-1)%4])),retry_occluded=False);self.check(cancel)
        snapshot=backend.source_snapshot
        points=np.asarray([self.map_point(snapshot,p) for p in scene.target])
        bounds=backend.table['axis_aligned_bounds'];center=bounds['center'];size=bounds['size']
        middle=np.median(points,axis=0)
        from grasp_runtime.autograsp.identity_policy import source_association
        on_source=source_association(middle,backend.table,getattr(backend,'identity_preview',{}).get('confidence',0.))
        path=self.output/'source-target-observation.json'
        result=observation_result(target=self.target,source_table=backend.table['id'],expected_target=self.target,expected_source=self.source_id,points_map=points.tolist(),
            room_polygon=self.environment['rooms'][self.source_room]['region'],source_support_verified=on_source,
            evidence_ref=str(path),room=self.source_room)
        path.write_text(json.dumps(dict(result=result,points_map=points.tolist(),depth_stamp_ns=int(scene.stamp_ns))))
        return result

    def reacquire_after_staging(self,cancel):
        from staging_view_recovery import reacquire
        return reacquire(self,cancel)

    def prepare_grasp_staging(self,effect,cancel):
        if self.candidate_selector is None:return
        from adaptive_grasp_staging import prepare
        from staging_view_recovery import prepare_with_recovery
        return prepare_with_recovery(self,effect,cancel,prepare)

    def grasp(self,effect,cancel):
        self.check(cancel);backend=self.grasp_backend;backend.use_family_templates=True
        backend.near_grasp_pose_handoff=True
        backend.automatic_source_return=True
        backend.cyclic_rotation_recovery=True
        backend.check_task_cancelled=lambda:self.check(cancel)
        backend.session_motion=self.motion
        pump=getattr(self.runtime,'pump',None)
        if pump is not None:
            def cancellable_pump(*a,**kw):
                self.check(cancel)
                return pump(*a,**kw)
            self.runtime.pump=cancellable_pump
        try:
            self.prepare_grasp_staging(effect,cancel)
            with grasp_observation_recovery(backend,self.runtime,cancel):
                backend.prepare_grasp_staging=lambda:self.prepare_grasp_staging(effect,cancel)
                report=self.coordinator_factory(backend,backend.hand,policy=backend.policy,record=backend.record).run(self.target)
        finally:
            if pump is not None:self.runtime.pump=pump
        self.check(cancel);result=grasp_result(report,target=self.target)
        if result['status']=='ok':
            self.context=backend.held_context
            path=self.output/'held-context.json';path.write_text(json.dumps(self.context))
            result['held_context_path']=str(path)
        (self.output/'grasp-result.json').write_text(json.dumps(report))
        return result

    def navigate_destination(self,effect,cancel):
        pose=self.state(cancel)['poses']['map']
        room=self.config.get('destination_room',self.source_room)
        if self.config.get('session_profile',{}).get('destination',{}).get('mode')=='container_drop':
            from container_approach import plan_approach,container_handoff_radius
            from room_entry import finish_on_container_approach,entered
            guard=getattr(self.motion,'turn_guard',None)
            carry_radius=guard.geometry().radius if guard is not None else .9
            radius=container_handoff_radius(self.config['container_opening'],
                self.candidate_selector.tables[self.destination_id],carry_radius)
            self.destination_goal,region=plan_approach(self.candidate_selector,pose,self.config['container_opening'],
                self.environment['rooms'][room]['region'],self.destination_id,handoff_radius_m=radius)
            if entered(pose,region):
                result=dict(status='ok',arrived=True,room=room,basis='already_in_container_approach_region')
            else:
                with finish_on_container_approach(self.motion,self.output,room,region):
                    result=self.navigate(effect,cancel,self.destination_goal,room)
        else:
            from candidate_selection import destination_candidates
            candidates=destination_candidates(furniture_candidates(self.environment,self.destination_id),self.config,
                self.environment['rooms'][room]['region'])
            self.destination_goal=self.select_goal(pose,candidates,self.destination_id,'destination-candidate-selection')
            result=self.navigate(effect,cancel,self.destination_goal,room)
        return dict(result,destination=self.destination_id)

    def handlers(self,transfer_handlers):
        required={'tuck','place','verify_placement'}
        if set(transfer_handlers)!=required:raise ValueError('three_real_transfer_handlers_required')
        return dict(navigate_room=self.navigate_room,navigate_source=self.navigate_source,
                    observe_target=self.observe_target,grasp=self.grasp,
                    navigate_destination=self.navigate_destination,**transfer_handlers)
