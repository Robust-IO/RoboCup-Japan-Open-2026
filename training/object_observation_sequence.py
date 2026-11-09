"""Class/task-bound repeatability evidence in image-time map coordinates.

    Room support and map consistency are needed for a planning handoff, not for
    displaying all30 detector classes. This never grants a motion/closure lease.
"""
import math
import numpy as np

from apple_room_evidence import room_check
from manipulation_profiles import object_profile


class ObjectObservationSequence:
    def __init__(self,name,task_id,epoch,environment,room):
        if not task_id or not epoch or room not in environment['rooms']:raise ValueError('explicit_observation_context_required')
        self.profile=object_profile(name)
        self.name,self.task_id,self.epoch,self.environment,self.room=name,task_id,epoch,environment,room
        self.history=[]

    def observe(self,inspection,snapshot,*,task_id,now_ns):
        result=dict(class_name=self.name,coarse_identity_consistent=False,motion_authorized=False,
                    grasp_center_verified=False,does_not_exist_proven=False)
        try:
            if task_id!=self.task_id or snapshot.get('tracking_epoch')!=self.epoch:
                raise ValueError('task_or_epoch_mismatch')
            if inspection.get('class_name')!=self.name or not inspection.get('unique_geometry_candidate'):
                raise ValueError('unique_target_geometry_required')
            stamps=[snapshot['streams'][k]['stamp_ns'] for k in ('rgb','depth')]
            if any(type(s) is not int or s<=0 for s in stamps) or not 0<=(now_ns-min(stamps))/1e9<=.4 or abs(stamps[0]-stamps[1])>30_000_000:
                raise ValueError('fresh_synchronised_rgbd_required')
            item=inspection['geometry_candidates'][0];detection=item['detection'];geometry=item['geometry']
            if geometry.get('adapter')!=self.profile['geometry']:raise ValueError('object_geometry_adapter_mismatch')
            if detection.get('name')!=self.name or not math.isfinite(detection['confidence']) or not self.profile['identity_confidence']<=detection['confidence']<=1:
                raise ValueError('identity_detection_not_confirmed')
            candidate=geometry.get('legacy_candidate',geometry)
            if candidate.get('stamp_ns')!=stamps[1]:raise ValueError('geometry_stamp_mismatch')
            point=np.asarray(candidate['center_m'],float)
            if point.shape!=(3,) or not np.isfinite(point).all():raise ValueError('invalid_current_geometry')
            room=room_check(candidate,snapshot,self.environment,self.room)
            if not room['room_confirmed']:raise ValueError('target_outside_requested_room')
            mapped=room['position_map_m'];stamp=stamps[1]
            if self.history:
                gap=(stamp-self.history[-1][0])/1e9
                if gap<=0:raise ValueError('duplicate_or_reversed_observation')
                if gap>1:self.history=[]
            if any(math.dist(mapped,p)>.02 for _,p in self.history):raise ValueError('inconsistent_current_geometry')
            self.history.append((stamp,list(mapped)));self.history=self.history[-3:]
            span=(stamp-self.history[0][0])/1e9
            result.update(coarse_identity_consistent=len(self.history)==3 and span>=.2,frames=len(self.history),
                          span_s=span,position_map_m=mapped,room_evidence=room,
                          limitation='Repeatability does not establish absolute accuracy, graspability, source-table ownership or contact.')
        except (KeyError,TypeError,ValueError) as exc:
            self.history=[];result['reason']=str(exc)
        return result
