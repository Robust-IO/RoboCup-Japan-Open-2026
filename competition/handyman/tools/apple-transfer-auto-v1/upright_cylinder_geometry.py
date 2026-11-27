"""Upright opaque-can model proposals from observed side curvature and top.

No red mask or sphere fitting. Hidden surfaces and contact are not measured;
this module cannot authorize a grasp or certify that an object is still held.
"""
import math
import cv2
import numpy as np

from object_surface_geometry import support_plane


def select_depth_surface(points,foreground,seeds,max_jump_m=.02):
    """Separate image-adjacent but physically distant surfaces.

    Only a unique component through the detector's central region is selected;
    background size alone is never a reason to select a component.
    """
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    points=np.asarray(points,float);foreground=np.asarray(foreground,bool);seeds=np.asarray(seeds,bool)
    if (foreground.ndim!=2 or points.shape!=(*foreground.shape,3) or seeds.shape!=foreground.shape
            or not np.isfinite(points[foreground]).all() or not np.isfinite(max_jump_m) or max_jump_m<=0):
        raise ValueError('invalid_cylinder_depth_surface')
    count=int(np.count_nonzero(foreground))
    if count<80:raise ValueError('insufficient_cylinder_depth_surface')
    index=np.full(foreground.shape,-1,dtype=np.int32);index[foreground]=np.arange(count)
    edges=[]
    for a,b in ((np.s_[:,:-1],np.s_[:,1:]),(np.s_[:-1,:],np.s_[1:,:]),
                (np.s_[:-1,:-1],np.s_[1:,1:]),(np.s_[:-1,1:],np.s_[1:,:-1])):
        connected=foreground[a]&foreground[b]&(np.linalg.norm(points[a]-points[b],axis=-1)<=max_jump_m)
        edges.append((index[a][connected],index[b][connected]))
    source=np.concatenate([e[0] for e in edges]);target=np.concatenate([e[1] for e in edges])
    graph=coo_matrix((np.ones(len(source),np.uint8),(source,target)),shape=(count,count)).tocsr()
    component_count,labels=connected_components(graph,directed=False)
    sizes=np.bincount(labels,minlength=component_count)
    candidates=np.unique(labels[seeds[foreground]])
    candidates=candidates[sizes[candidates]>=80]
    if len(candidates)!=1:raise ValueError('ambiguous_or_missing_central_cylinder_surface')
    selected=np.zeros_like(foreground);selected[foreground]=labels==candidates[0]
    return selected,dict(depth_component_count=int(np.count_nonzero(sizes>=80)),
        discarded_background_points=count-int(selected.sum()),max_neighbor_jump_m=max_jump_m,
        selection='unique_depth_connected_surface_through_detector_center')


def fit_side_circle(points,radius_range):
    points=np.asarray(points,float)
    if points.ndim!=2 or points.shape[1]!=2 or len(points)<60 or not np.isfinite(points).all():
        raise ValueError('insufficient_cylinder_side_points')
    origin=points.mean(axis=0);local=points-origin
    singular=np.linalg.svd(local,compute_uv=False)
    if singular[0]<=0 or singular[-1]/singular[0]<.04:raise ValueError('cylinder_side_curvature_not_observed')
    selected=local
    for _ in range(3):
        design=np.c_[2*selected,np.ones(len(selected))]
        q,_,rank,_=np.linalg.lstsq(design,np.sum(selected*selected,axis=1),rcond=None)
        squared=q[2]+q[:2]@q[:2]
        if rank!=3 or not np.isfinite(squared) or squared<=0:raise ValueError('invalid_cylinder_circle_fit')
        radius=float(np.sqrt(squared))
        if not radius_range[0]<=radius<=radius_range[1]:raise ValueError('cylinder_radius_outside_observation_envelope')
        residual=np.abs(np.linalg.norm(local-q[:2],axis=1)-radius)
        selected=local[residual<=.004]
        if len(selected)<60:raise ValueError('noisy_cylinder_side')
    error=float(np.quantile(residual,.95))
    if error>.004:raise ValueError('cylinder_side_fit_residual')
    delta=selected-q[:2]
    angles=np.sort(np.arctan2(delta[:,1],delta[:,0]))
    coverage=float(2*math.pi-np.max(np.diff(np.r_[angles,angles[0]+2*math.pi])))
    if coverage<.5:raise ValueError('cylinder_side_arc_too_narrow')
    return q[:2]+origin,radius,error,coverage


def cylinder_from_points(points,surrounding,profile):
    points=np.asarray(points,float)
    if points.ndim!=2 or points.shape[1]!=3 or len(points)<100 or not np.isfinite(points).all():
        raise ValueError('insufficient_cylinder_surface')
    coef,plane_points,plane_residual=support_plane(surrounding)
    heights=points[:,2]-np.c_[points[:,:2],np.ones(len(points))]@coef
    points=points[(heights>.004)&(heights<profile['height_range_m'][1]+.02)]
    if len(points)<100:raise ValueError('insufficient_above_support_cylinder')
    heights=points[:,2]-np.c_[points[:,:2],np.ones(len(points))]@coef
    height=float(np.quantile(heights,.99))
    if not profile['height_range_m'][0]<=height<=profile['height_range_m'][1]:
        raise ValueError('cylinder_height_outside_observation_envelope')
    top=points[np.abs(heights-height)<.005]
    side=points[(heights>.15*height)&(heights<.8*height)]
    if len(top)<30:raise ValueError('cylinder_top_not_observed')
    if len(side)<60 or np.ptp(side[:,2])<.25*height:raise ValueError('cylinder_side_height_not_observed')
    center_xy,radius,error,coverage=fit_side_circle(side[:,:2],profile['radius_range_m'])
    distances=np.linalg.norm(top[:,:2]-center_xy,axis=1)
    if np.quantile(distances,.95)>radius+.005:raise ValueError('cylinder_top_and_side_disagree')
    if cv2.pointPolygonTest(cv2.convexHull(plane_points[:,:2].astype(np.float32)),tuple(map(float,center_xy)),False)<0:
        raise ValueError('cylinder_outside_observed_support')
    bottom=float(np.r_[center_xy,1.]@coef)
    return dict(shape='upright_cylinder',center_m=[*center_xy,bottom+height/2],height_m=height,
        footprint_size_m=[2*radius,2*radius],footprint_radius_m=radius,estimated_grasp_width_m=2*radius,
        footprint_yaw_rad=None,support_plane_base=coef.tolist(),support_height_m=bottom,
        support_plane_residual_p95_m=plane_residual,cylinder_side_residual_p95_m=error,
        observed_side_arc_rad=coverage,surface_points=len(points),top_points=len(top),side_points=len(side),
        center_semantics='support_constrained_upright_cylinder_model_fit',
        hidden_surfaces_measured=False,dimension_envelope_is_provisional=True,
        geometry_fit_is_not_total_uncertainty=True,motion_authorized=False,
        identity_confirmed=False,grasp_center_verified=False,
        limitation='Upright opaque-can model proposal. Not measured hidden back surface, contact proof, grasp center calibration or a successful grasp.')
