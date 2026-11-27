"""Task-local acceptance of functional outcomes; no commands or hidden depth."""
import json,math
from functools import lru_cache
from pathlib import Path
import cv2,numpy as np

@lru_cache(maxsize=1)
def load():
    value=json.loads(Path(__file__).with_suffix('.json').read_text())
    if value['version']!=1 or value['name']!='outcome_v1':raise ValueError('unknown_outcome_policy')
    def check(node):
        if isinstance(node,dict):
            for v in node.values():check(v)
        elif isinstance(node,(int,float)) and (not math.isfinite(node) or node<0):raise ValueError('invalid_outcome_policy_number')
    check(value)
    if min(value['verification']['lift_frames'],value['verification']['placed_frames'])<2:raise ValueError('independent_outcome_evidence_required')
    return value

def snapshot(root):
    value=load();(Path(root)/'outcome-policy.json').write_text(json.dumps(value,indent=2));return value

def alignment_tolerances(open_margin):
    a=load()['alignment'];return [a['forward_m'],min(a['lateral_m'],max(.002,.5*open_margin)),a['vertical_m']]

def holding_feedback(joints,opening):
    if len(joints)!=6 or not np.isfinite([*joints,opening]).all():raise ValueError('finite_current_hold_required')
    if joints[5]>=.60 or abs(joints[5]-opening)>load()['hold']['opening_drift_rad']:raise ValueError('actual_grip_open_or_changed')
    return dict(actual_opening_rad=joints[5],opening_drift_rad=abs(joints[5]-opening))

def joint_endpoint_reached(actual,goal,*,hand=False):
    # A navigation handoff must satisfy the EXISTING .02rad Nav2 monitor.
    # Relax precision at grasp/place poses, but do not declare a navigation
    # posture ready while the unchanged downstream controller would reject it.
    tucked=abs(goal[1])<1e-7 and abs(goal[2])<1e-7 and abs(goal[3]+math.pi/2)<1e-7 and abs(goal[4])<1e-7
    angular=.015 if tucked else .04
    tolerances=[.005,angular,angular,angular,angular,.02 if hand else .03]
    return len(actual)==len(goal)==6 and all(abs(a-b)<=t for a,b,t in zip(actual,goal,tolerances))

def stationary_action_feedback(snapshot,state,now_ns,epoch,*,current_generation):
    """A stopped capture stays valid until state or command generation changes.

    Fresh input acquisition and CURRENT feedback still have their own age checks.
    Elapsed computation time alone does not invalidate an unchanged observation.
    """
    from hand_camera_ray import rotation
    from stop_observe_act import state_difference
    from observation_profiles import stamps as required_stamps,anchor_key
    if not epoch or snapshot.get('tracking_epoch')!=epoch:raise ValueError('action_epoch_mismatch')
    profile=snapshot.get('observation_profile','paired');stamps=required_stamps(snapshot)
    sync_ok=profile not in ('paired','rgbd') or abs(snapshot['streams']['rgb']['stamp_ns']-snapshot['streams']['depth']['stamp_ns'])<=30_000_000
    if any(type(s) is not int or not 0<s<=now_ns for s in stamps) or not sync_ok:
        raise ValueError('action_measurement_unsynchronised_or_clock_changed')
    marker=snapshot.get('stop_observe_act',{})
    if marker.get('policy')!='stop_observe_act_v1' or marker.get('captured_after_stop') is not True:raise ValueError('stopped_capture_proof_required')
    if marker.get('generation')!=current_generation:raise ValueError('motion_invalidated_observation_retake_required')
    if min(stamps)<max(marker['stop_verified_stamp_ns'],marker.get('capture_not_before_ns',0)):raise ValueError('new_image_after_stop_required')
    if not -20_000_000<=now_ns-state['stamp_ns']<=600_000_000:raise ValueError('fresh_current_action_feedback_required')
    names=('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint')
    captured=[float(snapshot['joints'][n]) for n in names]
    tf=snapshot['tracking_transforms']['odom_from_base_at_'+anchor_key(profile)];r=rotation(tf['quaternion'])
    old=[*tf['translation'][:2],float(np.arctan2(r[1,0],r[0,0]))]
    reference=dict(joints=captured,head=[snapshot['joints'][n] for n in ('head_pan_joint','head_tilt_joint')],poses=dict(odom=old))
    policy=dict(load()['feedback'])
    if snapshot.get('precise_publish_check'):
        from gate_recovery import CONTACT_STABILITY
        policy.update(CONTACT_STABILITY)
    difference=state_difference(reference,state,policy)
    if difference['changed']:raise ValueError('robot_changed_since_stopped_capture_retake_required')
    return dict(oldest_image_age_s=(now_ns-min(stamps))/1e9,current_joint_age_s=(now_ns-state['stamp_ns'])/1e9,
        base_translation_since_capture_m=difference['base_translation_m'],base_yaw_since_capture_rad=difference['base_yaw_rad'],
        original_snapshot_unchanged=True,validity_basis='stopped_capture_and_unchanged_command_generation_and_state',
        computation_age_is_rejection_gate=False,motion_authorized=False)

def hold_assessment(reference,current,appearance):
    """Soft shape matching. Only repeated absence is a hard loss decision."""
    def components(image):
        hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV);d=abs(hsv[:,:,0].astype(float)-appearance['hue']);d=np.minimum(d,180-d)
        mask=((d<=appearance['tolerance'])&(hsv[:,:,1]>55)&(hsv[:,:,2]>25)).astype(np.uint8)
        return cv2.connectedComponentsWithStats(cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8)),8)
    if reference is None or current is None or reference.shape!=current.shape:return dict(state='inconclusive',reason='hand_reference_unavailable')
    count,labels,stats,centers=components(reference)
    candidates=sorted(range(1,count),key=lambda i:stats[i,cv2.CC_STAT_AREA],reverse=True)
    if not candidates or stats[candidates[0],cv2.CC_STAT_AREA]<60:return dict(state='inconclusive',reason='weak_reference')
    best=candidates[0];old=labels==best;old_center=centers[best];count,labels,stats,centers=components(current)
    nearby=cv2.dilate(old.astype(np.uint8),np.ones((91,91),np.uint8)).astype(bool)
    if np.count_nonzero((labels>0)&nearby)<max(20,.05*old.sum()):return dict(state='absent',reason='no_target_colour_in_hand_region')
    options=[];policy=load()['hold']
    for i in range(1,count):
        new=labels==i;intersection=np.count_nonzero(old&new)
        if not intersection:continue
        ratio=float(new.sum()/old.sum());iou=float(intersection/np.count_nonzero(old|new));shift=float(np.linalg.norm(centers[i]-old_center))
        options.append(dict(iou=iou,area_ratio=ratio,center_shift_pixels=shift,object_depth_measured=False))
    options.sort(key=lambda x:x['iou'],reverse=True)
    if options:
        p=options[0]
        if p['iou']>=policy['iou'] and policy['area_ratio_min']<=p['area_ratio']<=policy['area_ratio_max'] and p['center_shift_pixels']<=policy['center_shift_px']:
            return dict(p,state='consistent',shape_is_not_grasp_proof=True)
        return dict(p,state='inconclusive',reason='appearance_changed_recheck_without_releasing')
    return dict(state='inconclusive',reason='foreground_near_hand_reference_but_no_overlap')

class HoldEvidence:
    def __init__(self):self.absent=[]
    def observe(self,assessment,stamp):
        if assessment['state']=='absent':
            if not self.absent or stamp-self.absent[-1]>=100_000_000:self.absent.append(stamp)
            if len(self.absent)>=load()['hold']['confirmed_absence_frames']:raise ValueError('held_object_absent_in_repeated_hand_frames')
        else:self.absent=[]
        return dict(assessment,consecutive_absence_frames=len(self.absent))

class ClosureEvidence:
    def __init__(self,opening,duration=6.):self.initial=opening;self.duration=duration;self.rows=[];self.last=None
    def observe(self,elapsed,stamp,opening,arm_error):
        if not np.isfinite([elapsed,stamp,opening,arm_error]).all() or not -.005<=opening<=.7:raise ValueError('invalid_closure_feedback')
        if self.last is not None and stamp<self.last:raise ValueError('closure_clock_reversed')
        if stamp==self.last:return None
        self.last=stamp
        if arm_error>.005:raise ValueError('arm_moved_during_close')
        if elapsed<self.duration:self.rows=[];return None
        if self.rows and (stamp-self.rows[-1][1]>600_000_000 or elapsed-self.rows[-1][0]>.6):self.rows=[]
        self.rows.append((elapsed,stamp,opening));p=load()['closure']
        if max(x[2] for x in self.rows)-min(x[2] for x in self.rows)>p['opening_range_rad']:self.rows=self.rows[-1:]
        if len(self.rows)<p['stable_samples'] or min(elapsed-self.rows[0][0],(stamp-self.rows[0][1])/1e9)<p['stable_s']:return None
        return dict(state='contact_candidate' if opening>.01 and self.initial-opening>=.015 else 'no_contact',
            observed_opening_rad=opening,stable_samples=len(self.rows),stable_duration_s=elapsed-self.rows[0][0],
            grasp_success_verified=False,requires_visual_lift_verification=True)
