"""Navigation and held-object presentation; this adapter never opens the hand."""
import json
from itertools import count
from task_recovery import retry_read
import numpy as np
from handover import held_center,presenting_joints,delivery_evidence
from hold_observer import HoldObserver

class HandoverLive:
    def __init__(self,stages,*,map_point=None,rotation=None):
        if map_point is None:from apple_transfer_auto_plan import map_point
        if rotation is None:from hand_camera_ray import rotation
        self.stages=stages;self.rt=stages.runtime;self.motion=stages.motion;self.backend=stages.grasp_backend
        self.cfg=stages.config['handover'];self.point=np.asarray(self.cfg['delivery_point_map_m'],float)
        self.map_point=map_point;self.rotation=rotation

    def navigate(self,effect,cancel):
        self.stages.check(cancel);self.rt.sample(profile='state');state=self.rt.state()
        selector=self.stages.candidate_selector
        from container_approach import plan_approach
        from room_entry import entered,finish_on_container_approach
        from perception_bridge import inside
        room=next((name for name,data in self.stages.environment['rooms'].items()
                   if inside(*self.point[:2],data['region'])),None)
        if room is None:raise ValueError('moderator_delivery_point_outside_rooms')
        x,y=self.point[:2]
        opening={'polygon_map_m':[[x-.025,y-.025],[x+.025,y-.025],[x+.025,y+.025],[x-.025,y+.025]]}
        bounds={'axis_aligned_bounds':{'center':{'x':x,'y':y},'size':{'x':.05,'y':.05}}}
        goal,region=plan_approach(selector,state['poses']['map'],opening,
            self.stages.environment['rooms'][room]['region'],'Moderator',target_bounds=bounds)
        if not entered(state['poses']['map'],region):
            # Shared navigation wrapper retains held-object turn protection.
            # Stop anywhere in the reachable delivery approach region, no yaw gate.
            with finish_on_container_approach(self.motion,self.stages.output,room,region):
                result=self.stages.navigate(effect,cancel,goal,room)
            if result.get('arrived') is not True:raise ValueError('moderator_navigation_incomplete')
        self.stages.check(cancel)
        return dict(status='ok',arrived=True,destination='Moderator',basis='delivery_region_any_heading')

    def observation(self):
        item=self.backend._sample();snapshot=item['snapshot'];self.rt.fresh_measurement(snapshot)
        palm=self.rt.palm_pose(snapshot)
        from handover_current_surface import locate
        points,colours,_=self.backend._cloud(item)
        center,details=locate(self.stages.context,palm,self.rotation,points,colours)
        self.last_visual=dict(visual_confirmation=True,depth_stamp_ns=snapshot['streams']['depth']['stamp_ns'],**details)
        self.backend.record(dict(event='handover_current_surface_localized',**self.last_visual))
        return snapshot,np.asarray(self.map_point(snapshot,center))

    def present(self,effect,cancel):
        self.stages.check(cancel)
        self.stages.context['require_visual_carry_checks']=False
        observe=HoldObserver(self.backend,self.stages.context);observe('before-handover-extension')
        self.rt.sample(profile='state');state=self.rt.state()
        joints=presenting_joints(self.stages.context,self.point[2],state['joints'][5])
        # Raise while folded, then extend; retain current gripper command throughout.
        raised=list(state['joints']);raised[0]=joints[0]
        self.rt.move_joints(raised,'handover-raise-held');self.stages.check(cancel)
        self.rt.move_joints(joints,'handover-extend-held');self.stages.check(cancel)
        delivery_reclosed=False
        for attempt in count():
            self.stages.check(cancel)
            snapshot,center=retry_read(self.observation,cancel=cancel,record=self.backend.record,
                repair=lambda n:self.rt.face_source((0.,(-.15,0.,-.3)[(n-1)%3])))
            self.stages.check(cancel)
            hold=observe('handover-check')
            hold=dict(hold,**self.last_visual)
            proof=delivery_evidence(center,self.point,hold,target=self.stages.target,
                                    tolerance=min(.04,self.cfg['estimated_center_tolerance_m']),require_visual=True)
            proof['center_basis']='current_rgbd_registered_held_surface_bounds_not_rigidbody_com'
            if proof['delivery_ready'] and not delivery_reclosed:
                from handover_reclose import confirm
                opening=confirm(self.rt,lambda:self.stages.check(cancel),self.backend.record)
                self.stages.context['holding_opening_rad']=opening
                self.backend.opening=opening
                delivery_reclosed=True
                # Reobserve after reclosure; the held object may have shifted.
                continue
            if proof['delivery_ready']:
                proof['fresh_delivery_reclose_verified']=delivery_reclosed
                path=self.stages.output/'handover-evidence.json';proof['evidence_ref']=str(path)
                path.write_text(json.dumps(proof,indent=2));return proof
            state=self.rt.state();delta=self.point-center
            if abs(delta[2])>.03:
                goal=list(state['joints']);goal[0]+=float(delta[2])
                if not 0<=goal[0]<=.69:raise ValueError('handover_height_outside_reach')
                self.rt.move_joints(goal,'handover-height-correction');self.stages.check(cancel)
                # Height movement may change TF timing: remeasure before base translation.
                snapshot,center=retry_read(self.observation,cancel=cancel,record=self.backend.record,
                    repair=lambda n:self.rt.face_source((0.,(-.15,0.,-.3)[(n-1)%3])))
                state=self.rt.state();delta=self.point-center
            if np.linalg.norm(delta[:2])>.02:
                r=self.rotation(snapshot['tracking_transforms']['map_from_base_at_depth']['quaternion'])
                local=r.T@np.r_[delta[:2],0.]
                self.motion.stage='approach_placement'
                self.motion.adjust(f'handover-align-{attempt}',state['poses']['map'],[float(local[0]),float(local[1]),0.],
                    state['joints'][:5],state['joints'][5],single_placement_approach=True)
                self.stages.check(cancel)
        raise ValueError('handover_alignment_incomplete')
