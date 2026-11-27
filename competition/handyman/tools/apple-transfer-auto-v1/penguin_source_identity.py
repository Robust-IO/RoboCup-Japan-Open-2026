"""Keep one high-confidence identity across head motion using NEW RGBD fits."""
import copy
import numpy as np
from canned_grasp_geometry import transform_point
from penguin_geometry import NAME,validate_reference,near_window

def source_table_matches(center,table_center,table_half,body_radius):
    """Furniture association allows a supported body's visible overhang."""
    outside=np.maximum(np.abs(np.asarray(center)[:2]-table_center)-table_half,0.)
    return bool(np.linalg.norm(outside)<=body_radius)

def make_seed(measurement,snapshot,detection,capture):
    validate_reference(measurement)
    if (detection.get('name')!=NAME or not .8<=detection.get('confidence',0)<=1
            or measurement['stamp_ns']!=snapshot['streams']['depth']['stamp_ns']):
        raise ValueError('new_high_confidence_penguin_rgbd_seed_required')
    return dict(class_name=NAME,confidence=detection['confidence'],capture=str(capture),
        stamp_ns=measurement['stamp_ns'],tracking_epoch=snapshot['tracking_epoch'],model=copy.deepcopy(measurement),
        body_odom_m=transform_point(snapshot,measurement['center_m']).tolist(),
        head_odom_m=transform_point(snapshot,measurement['cap_center_m']).tolist(),motion_authorized=False)

def proposal(seed,snapshot):
    if (seed.get('class_name')!=NAME or seed['tracking_epoch']!=snapshot.get('tracking_epoch')
            or snapshot['streams']['depth']['stamp_ns']<=seed['stamp_ns']):
        raise ValueError('same_live_penguin_seed_epoch_required')
    b=transform_point(snapshot,seed['body_odom_m'],inverse=True)
    h=transform_point(snapshot,seed['head_odom_m'],inverse=True)
    height=snapshot['streams']['rgb']['height'];width=snapshot['streams']['rgb']['width'];model=seed['model']
    _,u,v=near_window((height,width),snapshot,[b,h],
        [model['observed_horizontal_extent_m']+.015,model['cap_radius_m']+.015])
    if not u.size:return None
    box=[float(u.min()),float(v.min()),float(u.max()+1),float(v.max()+1)]
    if box[0]<=1 or box[1]<=1 or box[2]>=width-1 or box[3]>=height-1:return None
    return dict(name=NAME,confidence=seed['confidence'],box=box,fresh_model_detection=False,
                identity_source='earlier_high_confidence_seed_plus_current_rgbd',identity_seed_stamp_ns=seed['stamp_ns'])

def confirm(seed,measurement,snapshot):
    if measurement.get('partial_seed_only') is not False:raise ValueError('full_new_source_geometry_required')
    proposal(seed,snapshot);model=seed['model'];errors=[]
    for key,observed in (('body_odom_m','center_m'),('head_odom_m','cap_center_m')):
        errors.append(float(np.linalg.norm(transform_point(snapshot,measurement[observed])-seed[key])))
    if (max(errors)>.02 or abs(measurement['body_radius_m']-model['body_radius_m'])>.006
            or abs(measurement['cap_radius_m']-model['cap_radius_m'])>.006
            or abs(measurement['body_cap_distance_m']-model['body_cap_distance_m'])>.007
            or abs(measurement['height_m']-model['height_m'])>.015):
        raise ValueError('new_penguin_geometry_not_same_identity_seed')
    return dict(identity_source='earlier_high_confidence_seed_plus_current_rgbd',
                current_model_detection_used=False,seed_confidence=seed['confidence'],
                seed_stamp_ns=seed['stamp_ns'],current_depth_stamp_ns=measurement['stamp_ns'],
                body_head_odom_errors_m=errors,new_identity_created=False)
