"""Current held-object evidence from geometry/colour correspondence, not opening alone."""
from pathlib import Path
import json
import numpy as np
from grasp_runtime.autograsp.tracking import register_surface,UnobservedSurface

class HoldObserver:
    def __init__(self,backend,context,*,rotation=None,registration=register_surface):
        if rotation is None:
            from hand_camera_ray import rotation
        if context.get('frame')!='hand_palm_link' or context.get('surface_rgb') is None:
            raise ValueError('measured_palm_surface_colours_required')
        self.backend=backend;self.context=context;self.rotation=rotation;self.registration=registration
        self.last_stamp=context['depth_stamp_ns'];self.serial=0

    def __call__(self,label):
        item=self.backend._sample();snapshot=item['snapshot'];stamp=snapshot['streams']['depth']['stamp_ns']
        if stamp<=self.last_stamp:raise ValueError('new_hold_observation_required')
        self.last_stamp=stamp
        opening=float(snapshot['joints']['hand_motor_joint'])
        if abs(opening-self.context['holding_opening_rad'])>.03:raise ValueError('current_grip_changed')
        strict=self.context.get('require_visual_carry_checks',False)
        if strict and label=='before-tuck' and not self.context.get('hand_hold_reference'):
            from hand_hold_evidence import HandHoldEvidence
            return HandHoldEvidence(self.backend,self.context,self.rotation)(label)
        if not strict and self.context.get('lift_evidence',{}).get('basis')=='lift_and_reclose_joint_feedback':
            if opening<=.01:raise ValueError('current_grip_empty')
            self.backend.rt.fresh_measurement(snapshot)
            proof=dict(holding_verified=True,carry_state_verified=True,depth_stamp_ns=stamp,
                holding_opening_rad=opening,source='lift_reclose_plus_retained_grip',visual_confirmation=False)
            self.backend.record(dict(event='hold_joint_feedback_confirmed',label=label,**proof))
            return proof
        palm=self.backend.rt.palm_pose(snapshot)
        template=np.asarray(self.context['vertices_palm_m'])@self.rotation(palm['quaternion']).T+palm['translation']
        points,colours,_=self.backend._cloud(item)
        near=(points>=template.min(0)-.035).all(1)&(points<=template.max(0)+.035).all(1)
        try:fitted=self.registration(template,np.asarray(self.context['surface_rgb']),points[near],colours[near])
        except UnobservedSurface:
            if strict:
                from hand_hold_evidence import HandHoldEvidence
                return HandHoldEvidence(self.backend,self.context,self.rotation)(label)
            if label not in ('before-tuck','after-tuck','before-place'):raise
            folded=abs(snapshot['joints'].get('arm_flex_joint',float('inf')))<.05
            if not folded or label not in ('before-tuck','after-tuck','before-place'):raise
            self.backend.rt.fresh_measurement(snapshot)
            proof=dict(holding_verified=False,carry_state_verified=True,requires_visual_recheck_before_release=True,
                depth_stamp_ns=stamp,holding_opening_rad=opening,source='verified_grasp_plus_retained_closed_grip_in_folded_posture')
            self.backend.record(dict(event='folded_hold_not_visible',label=label,**proof))
            return proof
        if abs(opening-self.context['holding_opening_rad'])>.03:
            raise ValueError('current_grip_changed')
        self.backend.rt.fresh_measurement(snapshot)
        self.serial+=1
        path=Path(self.backend.rt.save(item,f'hold-{self.serial:03d}-{label}'))
        proof=dict(holding_verified=True,depth_stamp_ns=stamp,evidence_ref=str(path),
                   correspondence_points=len(fitted.matched_points),
                   residual_p90_m=fitted.residual_p90_m,template_coverage=fitted.template_coverage,
                   holding_opening_rad=opening,visual_confirmation=True,source='current_colour_geometry_correspondence')
        (path/'hold-evidence.json').write_text(json.dumps(proof))
        return proof
