"""Measured-pose family templates. No historical target coordinates are reused."""
from dataclasses import replace
import numpy as np

GROUPS = {
 'round_body': 'canned_juice empty_ketchup empty_plastic_bottle filled_ketchup filled_plastic_bottle ground_pepper nursing_bottle pink_cup salt sauce soysauce spray_bottle sugar tumbler white_cup',
 'compact_body': 'apple cubic_clock rubiks_cube',
 'irregular_body': 'bear_doll dog_doll rabbit_doll toy_duck toy_penguin matryoshka piggy_bank',
 'thin_box': 'cigarette',
 'local_parts': 'camera game_controller toy_car',
 'waisted_body': 'hourglass',
}
FAMILIES={name:family for family,names in GROUPS.items() for name in names.split()}
REPRESENTATIVES={'round_body':'canned_juice','compact_body':'apple','irregular_body':'toy_penguin',
                 'thin_box':'cigarette','local_parts':'toy_car','waisted_body':None}


def select_template(target, scene, policy):
    """Pose inferred conservatively from observed support-relative dimensions.

    Partial RGBD geometry is evidence, not a full object mesh. Ambiguous poses
    retain clearance-checked generic candidates rather than claiming upright.
    """
    scene.validate(policy)
    target={'cannedjuice':'canned_juice'}.get(target,target)
    from object_grasp_profiles import profile_for
    object_profile=profile_for(target)
    family=object_profile['family'] if object_profile['family']!='auto' else FAMILIES.get(target,'unknown')
    p=np.asarray(scene.target);low,high=np.quantile(p,[.02,.98],axis=0)
    xy=p[:,:2]-np.median(p[:,:2],axis=0)
    _,axes=np.linalg.eigh(np.cov(xy.T))
    spans=np.diff(np.quantile(xy@axes,[.02,.98],axis=0),axis=0)[0]
    short,long=sorted(spans)
    heights=p[:,2]-p[:,:2]@scene.plane[:2]-scene.plane[2]
    height=float(np.quantile(heights,.98))
    pose='ambiguous'
    clearly_low_wide=bool(height>0 and long>0 and height/long<.65)
    if clearly_low_wide:pose='lying'
    elif height>1.25*long:pose='upright'
    elif long>1.6*max(height,short) and height>0:pose='lying'
    elif height<.045 and long>2*height:pose='flat'
    changes=dict(template_family=family,sequential_side=True,allowed_strategies=('side','overhead'),
                 lock_selected_pose=False,side_forward_insertion_m=0.)
    # Candidate fractions are priorities, never fixed metre heights.
    fractions=(.5,.4,.6)
    if family=='round_body':fractions=(.5,.4,.6)
    elif family=='irregular_body':fractions=(.4,.5,.3,.6)
    elif family=='thin_box':fractions=(.5,.7,.9)
    elif family=='waisted_body':fractions=(.5,.4,.6)
    elif family=='local_parts':fractions=(.3,.45,.6,.75,.9)
    if target=='apple':changes['template_family']='sphere'
    if target in ('empty_ketchup','filled_ketchup','empty_plastic_bottle','filled_plastic_bottle','soysauce','sauce','nursing_bottle'):
        changes['template_family']='neck_bottle';fractions=(.8,.9,.7,.6,.5)
        # High necks can be lifted 2cm and reclosed without reserving 8cm.
        changes['lift_check_m']=.02
    if family=='irregular_body':fractions=(.3,.45,.6,.75,.9)
    changes['side_height_fractions']=fractions
    if changes['template_family']=='round_body' and pose=='lying':changes['template_family']='lying_body'
    if clearly_low_wide:
        changes.update(preferred_strategy='side',allowed_strategies=('side',))
    elif family=='thin_box' and pose in ('flat','lying'):
        changes.update(preferred_strategy='side')  # Complete finger sweep raises a too-low contact.
    else:changes.update(preferred_strategy='side')
    result=replace(policy,**changes)
    return result,dict(target=target,family=result.template_family,observed_pose=pose,
        observed_dimensions_m=dict(height=height,horizontal_short=float(short),horizontal_long=float(long)),
        height_to_width_ratio=float(height/long) if long>0 else None,
        overhead_height_width_threshold=.65,
        selection_reason='side_first_overhead_after_fold' if clearly_low_wide else 'side_default_or_existing_class_rule',
        overhead_after_fold=clearly_low_wide,
        allowed_strategies=list(result.allowed_strategies),
        representative=REPRESENTATIVES.get(family),
        validation='new_template_executor_pending_live_validation',
        historical_xyz_used=False,partial_observation=True,
        approach='lateral_alignment_then_straight',tracking='current_surface',
        side_height_fractions=list(fractions),side_forward_insertion_m=0.)
