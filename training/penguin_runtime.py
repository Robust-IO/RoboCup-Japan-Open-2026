"""Head contact targeting over the existing finite-motion runtime."""
import math
import numpy as np
from hand_camera_ray import rotation
from pepper_runtime import PepperRuntime
from canned_grasp_geometry import closure_error

def view_goal_for_pixel(u,snapshot):
    """Turn the measured camera view, leaving the base route unchanged."""
    k=np.asarray(snapshot['rgb_k'],float).reshape(3,3)
    pan=snapshot['joints']['head_pan_joint'];tilt=snapshot['joints']['head_tilt_joint']
    goal=float(np.clip(pan+math.atan2(k[0,2]-u,k[0,0]),-.6,.6))
    return [goal,tilt] if abs(goal-pan)>.12 else None

class PenguinRuntime(PepperRuntime):
    def improve_unconfirmed_source_view(self,detections,snapshot):
        candidates=[d for d in detections if d.get('name')=='toy_penguin'
            and type(d.get('confidence')) in (int,float) and .1<=d['confidence']<.8
            and len(d.get('box',[]))==4 and np.isfinite(d['box']).all()]
        attempt=getattr(self,'source_view_attempt',0)
        if attempt>=3:return False
        k=np.asarray(snapshot['rgb_k']).reshape(3,3)
        pan=snapshot['joints']['head_pan_joint'];tilt=snapshot['joints']['head_tilt_joint']
        if candidates:
            # A camera-pointing proposal need not be a confirmed identity;
            # duplicate low-score boxes must not prevent looking at the target.
            d=max(candidates,key=lambda x:x['confidence']);x1,_,x2,_=d['box']
            centered=pan+math.atan2(k[0,2]-(x1+x2)/2,k[0,0])
            self.source_candidate_view_center=centered;self.source_candidate_score=d['confidence']
        elif hasattr(self,'source_candidate_view_center'):
            centered=self.source_candidate_view_center
        else:return False
        while attempt<3:
            goal=float(np.clip(centered+(0.,-.10,.10)[attempt],-.6,.6))
            attempt+=1;self.source_view_attempt=attempt
            if abs(goal-pan)<.02:continue
            self.record(dict(event='penguin_new_view_for_unconfirmed_identity',confidence=self.source_candidate_score,
                             head_goal=[goal,tilt],identity_threshold_unchanged=.8,
                             proposal_is_identity_confirmation=False))
            self.face_source([goal,tilt]);return True
        return False

    def approach_plan(self,error):
        forward,side,_=map(float,error)
        finger_gap=.093/2-self.current_penguin_geometry['cap_radius_m']-.003
        if abs(forward)<=.03 and .004<abs(side)<=finger_gap:
            return dict(phase='small_open_head_centering',delta=[0.,side,0.])
        if abs(side)>.012:
            extent=self.current_penguin_geometry['horizontal_support_radius_m']
            clearance=.15+max(0.,extent-.03)
            if forward<clearance-.01:
                return dict(phase='clear_penguin_extent_before_lateral_alignment',
                            delta=[forward-clearance,0.,0.])
            return dict(phase='lateral_alignment',delta=[0.,side,0.])
        return super().approach_plan(error)

    def recenter_clipped_detection(self,detection,snapshot):
        x1,_,x2,_=detection['box'];width=snapshot['streams']['rgb']['width']
        if x1>2 and x2<width-2:return False
        goal=view_goal_for_pixel((x1+x2)/2,snapshot)
        if goal is None:return False
        self.record(dict(event='penguin_recenter_clipped_source_view',box=detection['box'],head_goal=goal))
        self.face_source(goal)
        return True

    def recenter_target_view(self,body,head,snapshot):
        # An existing RGBD target predicts only where to point the camera. The
        # next sample must still fit both real depth surfaces before motion.
        center=(np.asarray(body)+head)/2
        tf=snapshot['base_from_rgb'];p=rotation(tf['quaternion']).T@(center-tf['translation'])
        if p[2]<=.05:return False
        k=np.asarray(snapshot['rgb_k'],float).reshape(3,3);u=float(k[0,0]*p[0]/p[2]+k[0,2])
        width=snapshot['streams']['rgb']['width']
        if .25*width<=u<=.75*width:return False
        goal=view_goal_for_pixel(u,snapshot)
        if goal is None:return False
        self.record(dict(event='penguin_recenter_existing_target_view',predicted_pixel_x=u,head_goal=goal,
                         prediction_is_geometry_proof=False))
        self.face_source(goal)
        return True

    def accept_geometry(self,measurement,snapshot):
        if measurement.get('class_name')!='toy_penguin' or measurement['stamp_ns']!=snapshot['streams']['depth']['stamp_ns']:
            raise ValueError('current_penguin_geometry_required')
        self.current_penguin_geometry=measurement

    def contact_error(self,center,snapshot):
        m=self.current_penguin_geometry
        if m['stamp_ns']!=snapshot['streams']['depth']['stamp_ns'] or not np.allclose(center,m['center_m'],atol=1e-9,rtol=0):
            raise ValueError('penguin_head_target_must_match_current_body_frame')
        return closure_error(m['grasp_center_m'],snapshot)

    def prepare(self,lift,head_above_support):
        arm=super().prepare(lift,head_above_support)
        self.vertical_alignment_tolerance_m=.004
        return arm
