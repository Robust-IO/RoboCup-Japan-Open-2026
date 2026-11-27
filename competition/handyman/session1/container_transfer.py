"""Container endpoint using measured/reviewed opening geometry and live feedback.

This intentionally implements a weak completion heuristic requested by the user:
three valid post-release views without a floor detection. It is NOT proof that
an object is inside an opaque container; only Moderator can confirm task success.
No ROS imports; live adapters own robot control and fresh observations.
"""
from dataclasses import dataclass
import math
from task_recovery import retry_read,check_cancel
import numpy as np
from perception_bridge import inside

@dataclass(frozen=True)
class ContainerPolicy:
    release_bottom_above_rim_m: float = .15
    minimum_clear_frames: int = 3
    frame_spacing_ns: int = 150_000_000
    settle_ns: int = 1_000_000_000
    maximum_retries: int = 5
    persistent_recovery: bool = False
    finish_on_release: bool = False


def release_pose(opening, bottom_below_palm_m):
    polygon=opening['polygon_map_m'];rim=float(opening['rim_height_m'])
    a=np.asarray(polygon,float)
    if a.ndim!=2 or a.shape[1]!=2 or len(a)<3 or not np.isfinite(np.r_[a.ravel(),rim,bottom_below_palm_m]).all():
        raise ValueError('finite_container_opening_required')
    center=a.mean(0)
    if not inside(*center,polygon):raise ValueError('opening_center_outside_polygon')
    return dict(destination_id=opening['destination_id'],gripper_xy_map_m=center.tolist(),
                palm_height_m=rim+ContainerPolicy().release_bottom_above_rim_m+bottom_below_palm_m,
                opening=opening,object_identity_recheck_required=False)


def release_ready(center_map,bottom_map_z,opening):
    center=np.asarray(center_map,float)
    if center.shape!=(3,) or not np.isfinite(np.r_[center,bottom_map_z]).all():return False
    # No demand for an exact center/pose or a newly visible held object.
    return bool(inside(*center[:2],opening['polygon_map_m']) and bottom_map_z>=opening['rim_height_m'])

def shorten_release_approach(plan,yaw,distance,*,allow_outside_preference=False):
    """Move the release target toward the robot along its forward axis."""
    if not math.isfinite(distance) or not 0<=distance<=.2:raise ValueError('invalid_container_approach_reduction')
    xy=np.asarray(plan['gripper_xy_map_m'])-distance*np.array([math.cos(yaw),math.sin(yaw)])
    if not allow_outside_preference and not inside(*xy,plan['opening']['polygon_map_m']):raise ValueError('shortened_release_target_outside_opening')
    return dict(plan,gripper_xy_map_m=xy.tolist(),approach_reduction_m=distance,
                original_gripper_xy_map_m=plan['gripper_xy_map_m'])


class OutsideFloorEvidence:
    def __init__(self,target,destination,release_stamp_ns,policy=ContainerPolicy()):
        self.target=target;self.destination=destination;self.release_stamp_ns=release_stamp_ns
        self.policy=policy;self.last_stamp=release_stamp_ns;self.clear=[]

    def observe(self,row):
        stamp=row.get('depth_stamp_ns',0)
        if (not row.get('valid_frame') or not row.get('evidence_ref')
                or row.get('destination_id')!=self.destination
                or stamp<=self.last_stamp or stamp<self.release_stamp_ns+self.policy.settle_ns):
            return dict(state='inconclusive',reason='new_valid_post_drop_floor_view_required')
        if self.clear and stamp-self.last_stamp<self.policy.frame_spacing_ns:
            return dict(state='inconclusive',reason='distinct_frames_required')
        self.last_stamp=stamp
        found=[d for d in row.get('floor_targets',[]) if d.get('target')==self.target]
        if found:
            self.clear=[]
            return dict(state='outside_target_found',detections=found,evidence_ref=row['evidence_ref'])
        if row.get('rim_targets'):
            self.clear=[]
            return dict(state='rim_target_found',detections=row['rim_targets'],evidence_ref=row['evidence_ref'])
        if not row.get('floor_visible'):
            return dict(state='inconclusive',reason='external_floor_view_required')
        if row.get('rim_observation_required') and not row.get('rim_visible'):
            self.clear=[]
            return dict(state='inconclusive',reason='bin_rim_view_required')
        if row.get('unlocalized_target',False):
            self.clear=[]
            return dict(state='inconclusive',reason='target_detection_depth_missing')
        self.clear.append(row['evidence_ref'])
        return dict(state='clear' if len(self.clear)>=self.policy.minimum_clear_frames else 'observing',
                    clear_frames=len(self.clear),evidence_refs=list(self.clear),
                    basis='no_target_detected_on_observed_external_floor',inside_container_proven=False)


def drop_and_check(adapter,target,destination,*,cancel,policy=ContainerPolicy()):
    """Adapter: opening/approach/release/withdraw/capture/recover, all synchronous.

    An ambiguous observation never triggers another release. Only a positive
    external-floor detection may enter automatic pickup/retry.
    """
    attempts=0
    while True:
        if cancel.is_set():raise RuntimeError('task_cancelled')
        opening=retry_read(lambda:adapter.observe_opening(destination),cancel=cancel)
        if opening['destination_id']!=destination:raise ValueError('wrong_container')
        plan=release_pose(opening,adapter.bottom_below_palm_m())
        adapter.approach(plan)
        if cancel.is_set():raise RuntimeError('task_cancelled')
        center,bottom=adapter.current_gripper_and_bottom()
        ready=getattr(adapter,'release_geometry_ready',lambda c,b:release_ready(c,b,opening))
        if not ready(center,bottom):
            if policy.persistent_recovery:
                if cancel.wait(.2):check_cancel(cancel)
                continue
            return dict(status='incomplete',reason='gripper_not_over_opening')
        stamp=adapter.release_once()
        if policy.finish_on_release:
            return dict(status='ok',target=target,destination=destination,release_sent=True,
                        release_stamp_ns=stamp,completion_actions_confirmed=True,
                        basis='gripper_release_completed',cleanup_completed=False,cleanup_skipped=True,
                        placement_verified=False,moderator_acknowledged=False)
        adapter.withdraw_empty()
        proof=OutsideFloorEvidence(target,destination,stamp,policy)
        observations=0
        while policy.persistent_recovery or observations<12:
            observations+=1
            if cancel.is_set():raise RuntimeError('task_cancelled')
            row=retry_read(adapter.capture_external_floor,cancel=cancel);result=proof.observe(row)
            if result['state']=='clear':
                return dict(status='ok',target=target,destination=destination,release_sent=True,
                            placement_verified=True,local_completion_heuristic=True,
                            moderator_acknowledged=False,retries=attempts,verification=result)
            if result['state'] in ('outside_target_found','rim_target_found'):
                if policy.persistent_recovery and not adapter.recover_floor_target(result['detections']):
                    if cancel.wait(.5):check_cancel(cancel)
                    continue
                break
            if policy.persistent_recovery and cancel.wait(.2):check_cancel(cancel)
        else:return dict(status='incomplete',release_sent=True,reason='external_floor_observation_inconclusive')
        if not policy.persistent_recovery and attempts>=policy.maximum_retries:
            return dict(status='incomplete',release_sent=True,reason='outside_target_after_retries')
        # Recovery must actually re-grasp the observed object, not fake a held flag.
        if not policy.persistent_recovery and not adapter.recover_floor_target(result['detections']):
            return dict(status='incomplete',release_sent=True,reason='outside_target_recovery_failed')
        attempts+=1
