"""Measured flat-box top and support. Separate from cylinder/cube adapters."""
import math
import cv2
import numpy as np
from object_surface_geometry import support_plane
from held_can_appearance import pixel_points
from hand_camera_ray import rotation


def box_from_points(points,surrounding):
    points=np.asarray(points,float)
    if points.ndim!=2 or points.shape[1]!=3 or len(points)<60 or not np.isfinite(points).all():
        raise ValueError('insufficient_flat_box_depth')
    plane,_,plane_error=support_plane(surrounding)
    above=points[:,2]-(points[:,:2]@plane[:2]+plane[2])
    points=points[above>max(.001,plane_error)]
    if len(points)<60:raise ValueError('flat_box_not_above_table')
    above=points[:,2]-(points[:,:2]@plane[:2]+plane[2])
    height=float(np.quantile(above,.98));top=points[abs(above-height)<.0025]
    if len(top)<40 or not np.isfinite(height) or height<=0:raise ValueError('flat_box_top_not_observed')
    rect=cv2.minAreaRect(top[:,:2].astype(np.float32));corners=cv2.boxPoints(rect)
    edges=np.roll(corners,-1,axis=0)-corners;sizes=np.linalg.norm(edges,axis=1)
    index=int(np.argmax(sizes));axis=np.r_[edges[index]/sizes[index],0.]
    if axis[1]<0:axis=-axis
    long=float(max(sizes));short=float(min(sizes))
    if not np.isfinite([short,long]).all() or short<=0 or long<=0:raise ValueError('flat_box_footprint_not_observed')
    xy=np.array(rect[0]);bottom=float(np.r_[xy,1.]@plane)
    center=np.r_[xy,bottom+height/2]
    grip_above=max(height/2,.0175+.002)
    overlap=min(height,grip_above+.0175)-max(0.,grip_above-.0175)
    if overlap<.002:raise ValueError('box_too_thin_for_observed_finger_side_overlap')
    return dict(class_name='cigarette',shape='flat_cuboid',center_m=center.tolist(),center_base_m=center.tolist(),
        axis_base=axis.tolist(),length_m=long,width_m=short,height_m=height,
        support_plane_base=plane.tolist(),support_height_m=bottom,support_plane_residual_p95_m=plane_error,
        grasp_point_base_m=[*xy,bottom+grip_above],grip_above_bottom_m=grip_above,
        estimated_finger_side_overlap_m=overlap,finger_lower_clearance_m=grip_above-.0175,
        top_points=len(top),source='current_flat_top_and_independent_table_depth',
        hidden_faces_measured=False,motion_authorized=False)


def estimate(rgb,depth,snapshot,detection):
    from rgbd_pixel_window import pixel_window
    from upright_cylinder_geometry import select_depth_surface
    if detection.get('name')!='cigarette':raise ValueError('cigarette_class_required')
    if rgb.shape!=(*depth.shape,3):raise ValueError('rgb_depth_shape_mismatch')
    stamps=[snapshot['streams'][k]['stamp_ns'] for k in ('rgb','depth')]
    if any(type(s) is not int or s<=0 for s in stamps) or abs(stamps[0]-stamps[1])>30_000_000:raise ValueError('registered_rgbd_required')
    registration=snapshot['rgb_from_depth']
    if (np.linalg.norm(registration['translation'])>1e-6 or
            not np.allclose(rotation(registration['quaternion']),np.eye(3),atol=1e-6,rtol=0)
            or not np.allclose(snapshot['rgb_k'],snapshot['depth_k'],atol=1e-6,rtol=0)):
        raise ValueError('registered_rgbd_required')
    x1,y1,x2,y2=map(float,detection['box']);h,w=depth.shape
    if not 1<x1<x2<w-1 or not 1<y1<y2<h-1:raise ValueError('box_detection_clipped')
    pad=max(25.,.7*max(x2-x1,y2-y1));window,u,v=pixel_window(depth.shape,x1-pad,y1-pad,x2+pad,y2+pad)
    z=depth[window];k=np.asarray(snapshot['rgb_k'],float).reshape(3,3)
    camera=np.stack(((u-k[0,2])*z/k[0,0],(v-k[1,2])*z/k[1,1],z),axis=-1)
    tf=snapshot['base_from_rgb'];points=camera@rotation(tf['quaternion']).T+tf['translation']
    valid=np.isfinite(z)&(z>0)&(z<3)
    ring=valid&~((u>=x1-4)&(u<x2+4)&(v>=y1-4)&(v<y2+4))
    plane,_,_=support_plane(points[ring]);above=points[:,:,2]-(points[:,:,:2]@plane[:2]+plane[2])
    inside=(u>=x1)&(u<x2)&(v>=y1)&(v<y2)
    foreground=valid&inside&(above>.001)
    seeds=(u>=x1+.2*(x2-x1))&(u<=x1+.8*(x2-x1))&(v>=y1+.2*(y2-y1))&(v<=y1+.8*(y2-y1))
    selected,segmentation=select_depth_surface(points,foreground,seeds)
    result=box_from_points(points[selected],points[ring])
    return dict(result,stamp_ns=stamps[1],confidence=detection.get('confidence'),segmentation=segmentation)

def select_source(rgb,depth,snapshot,detections,table=None,*,check_nominal_reach=True):
    """Choose a measured candidate on the requested table, not frame uniqueness."""
    from apple_transfer_auto_plan import map_point,footprint
    from apple_transfer_auto_plan import map_pose
    from table_approach_reachability import base_path
    from horizontal_can_grasp_plan import forward
    from inverse_table_place_plan import FLEX,WRIST
    candidates=[];rejected=[]
    for d in detections:
        if d.get('name')!='cigarette':continue
        try:
            model=estimate(rgb,depth,snapshot,d)
            if table is not None:
                c,h=footprint(table);mapped=map_point(snapshot,np.asarray(model['center_m']))
                if not (abs(mapped[:2]-c)<=h).all():raise ValueError('outside_requested_source_table')
                palm,r=forward([.25,FLEX,0.,WRIST,0.,.65]);contact=palm+r@np.array([.0000000685,.00005,.0621258659])
                delta=np.asarray(model['center_m'])[:2]-contact[:2]
                if check_nominal_reach and not base_path(map_pose(snapshot),[*delta,0.],table)['ready']:raise ValueError('candidate_requires_base_inside_table')
            width=abs(model['axis_base'][1])*model['length_m']+abs(model['axis_base'][0])*model['width_m']
            if (.093026-width)/2<.002:raise ValueError('candidate_too_wide_for_open_hand')
            candidates.append(model)
        except ValueError as exc:rejected.append(str(exc))
    if not candidates:raise ValueError('no_measured_cigarette_on_requested_source:'+','.join(rejected))
    candidates.sort(key=lambda m:(float(np.linalg.norm(np.asarray(m['center_m'])[:2])),-float(m.get('confidence') or 0.)))
    return candidates[0],dict(eligible_candidates=len(candidates),rejected=rejected,selection='nearest_measured_target_on_requested_table')


def appearance_from_source(rgb,depth,snapshot,model):
    p=pixel_points(depth,snapshot);c=np.asarray(model['center_m']);a=np.asarray(model['axis_base']);n=np.array([-a[1],a[0],0.])
    delta=p-c;hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    mask=(np.isfinite(depth)&(depth>0)&(abs(delta@a)<model['length_m']*.55)&(abs(delta@n)<model['width_m']*.55)
          &(abs(delta[:,:,2])<model['height_m']/2+.003)&(hsv[:,:,1]>45)&(hsv[:,:,2]>30))
    hues=hsv[:,:,0][mask]
    if len(hues)<40:raise ValueError('flat_box_coloured_surface_missing')
    mode=int(np.bincount(hues,minlength=180).argmax());d=abs(hues.astype(float)-mode);d=np.minimum(d,180-d)
    tolerance=float(max(8.,np.quantile(d,.95)+3))
    if tolerance>25:raise ValueError('flat_box_colour_not_distinct')
    return dict(hue=mode,tolerance=tolerance,source_pixels=len(hues),new_identity_detector=False)

def operator_confirmed_source(rgb,depth,snapshot,table,appearance,reference_model,*,operator_confirmed=True,expected_map=None,exclude_map=(),return_all=False):
    """Explicit human-confirmed trial identity; not a YOLO detection result."""
    from apple_transfer_auto_plan import map_point,footprint
    from can_destination_surface import measure as measure_table
    p=pixel_points(depth,snapshot);tf=snapshot['tracking_transforms']['map_from_base_at_depth']
    mapped=p@rotation(tf['quaternion']).T+tf['translation'];c,half=footprint(table)
    map_point(snapshot,[0.,0.,0.])
    measured_table=measure_table(depth,snapshot,table);coef=np.asarray(measured_table['plane_map'])
    residual=mapped[:,:,2]-(mapped[:,:,:2]@coef[:2]+coef[2])
    table_pixels=(np.isfinite(depth)&(depth>0)&(abs(mapped[:,:,:2]-c)<half-.025).all(2)&(abs(residual)<.003))
    surrounding=p[table_pixels]
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV);d=abs(hsv[:,:,0].astype(float)-appearance['hue']);d=np.minimum(d,180-d)
    mask=((depth>0)&np.isfinite(depth)&(d<=appearance['tolerance'])&(hsv[:,:,1]>45)&(hsv[:,:,2]>30)
        &(abs(mapped[:,:,:2]-c)<half).all(2)).astype(np.uint8)
    count,labels,stats,_=cv2.connectedComponentsWithStats(mask,8);candidates=[]
    for i in range(1,count):
        x,y,w,h,area=stats[i]
        if area<40 or w<5 or h<5:continue
        try:
            value=box_from_points(p[labels==i],surrounding)
            value['dimension_ratios_diagnostic']={k:value[k]/reference_model[k] for k in ('length_m','width_m','height_m')}
            mapped=map_point(snapshot,np.asarray(value['center_m']))
            if any(np.linalg.norm(mapped[:2]-np.asarray(old)[:2])<.04 for old in exclude_map):continue
            value.update(identity_source='operator_confirmed_retest_and_prior_box_colour' if operator_confirmed else 'same_verified_grasp_target_reacquired_on_destination',model_detection_used=False,operator_confirmed_class=operator_confirmed,
                stamp_ns=snapshot['streams']['depth']['stamp_ns'],confidence=None,support_source='current_whole_destination_table_plane_inliers')
            candidates.append(value)
        except ValueError:continue
    if return_all:return candidates
    if expected_map is not None and candidates:
        candidates.sort(key=lambda m:float(np.linalg.norm(map_point(snapshot,np.asarray(m['center_m']))[:2]-np.asarray(expected_map)[:2])))
        return candidates[0]
    if len(candidates)!=1:raise ValueError('unique_current_depth_box_matching_confirmed_reference_required')
    return candidates[0]


def measure_near(rgb,depth,snapshot,expected,model,appearance):
    p=pixel_points(depth,snapshot);expected=np.asarray(expected,float);hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    d=abs(hsv[:,:,0].astype(float)-appearance['hue']);d=np.minimum(d,180-d)
    radius=math.hypot(model['length_m'],model['width_m'])/2
    mask=(np.isfinite(depth)&(depth>0)&(d<=appearance['tolerance'])&(hsv[:,:,1]>45)&(hsv[:,:,2]>30)
          &(np.linalg.norm(p[:,:,:2]-expected[:2],axis=2)<radius+.045)&(abs(p[:,:,2]-expected[2])<.05))
    points=p[mask]
    if len(points)<40:raise ValueError('current_flat_box_depth_not_seen')
    top_z=float(np.quantile(points[:,2],.98));top=points[abs(points[:,2]-top_z)<.0025]
    if len(top)<30:raise ValueError('current_box_top_not_observed')
    rect=cv2.minAreaRect(top[:,:2].astype(np.float32));sides=sorted(map(float,rect[1]))
    if sides[0]<.5*model['width_m'] or sides[1]<.5*model['length_m']:
        raise ValueError('flat_box_top_too_occluded_to_center')
    center=np.r_[rect[0],top_z-model['height_m']/2]
    if np.linalg.norm(center-expected)>.065:raise ValueError('flat_box_surface_outside_same_target_region')
    return dict(class_name='cigarette',shape='flat_cuboid',center_m=center.tolist(),visible_surface_centroid_m=np.median(top,axis=0).tolist(),
        grasp_point_base_m=[*center[:2],top_z-model['height_m']+model['grip_above_bottom_m']],
        source='current_coloured_flat_top_depth',stamp_ns=snapshot['streams']['depth']['stamp_ns'],
        top_points=len(top),observed_top_z_m=top_z,height_m=model['height_m'],
        original_dimensions_depth_stamp_ns=model['stamp_ns'],hidden_faces_measured=False,motion_authorized=False)
