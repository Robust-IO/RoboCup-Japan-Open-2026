"""Continue a model-confirmed task target after lateral base translation."""
import numpy as np
from flat_box_geometry import measure_near
from canned_grasp_geometry import transform_point
from hand_camera_ray import rotation
from apple_transfer_auto_plan import map_point,footprint

def reacquire(rgb,depth,snapshot,reference,table):
    old=reference['snapshot'];model=reference['model'];appearance=reference['appearance']
    if model.get('class_name')!='cigarette' or not reference.get('model_detection_verified'):
        raise ValueError('prior_model_confirmed_task_target_required')
    if snapshot['streams']['depth']['stamp_ns']<=old['streams']['depth']['stamp_ns']:raise ValueError('newer_task_frame_required')
    expected=transform_point(snapshot,transform_point(old,model['center_m']),inverse=True)
    measured=measure_near(rgb,depth,snapshot,expected,model,appearance)
    current=map_point(snapshot,np.asarray(measured['center_m']));center,half=footprint(table)
    if not (abs(current[:2]-center)<=half).all():raise ValueError('tracked_target_not_on_requested_table')
    axis=rotation(snapshot['tracking_transforms']['odom_from_base_at_depth']['quaternion']).T@rotation(old['tracking_transforms']['odom_from_base_at_depth']['quaternion'])@np.asarray(model['axis_base'])
    value=dict(model,**measured);value.update(axis_base=axis.tolist(),center_base_m=measured['center_m'],support_height_m=measured['center_m'][2]-model['height_m']/2,
        identity_source='same_task_model_confirmed_target_reobserved_in_current_colour_depth',model_detection_used=False,
        confidence=None,reference_detection_confidence=model.get('confidence'))
    return value
