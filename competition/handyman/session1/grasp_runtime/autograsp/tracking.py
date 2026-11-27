"""Observed RGBD registration, with no class-specific width or hue threshold.

Inputs are already in a common frame. Robot surfaces must be removed by the
caller. A predicted transform only restricts correspondence search; it is never
returned as observed object motion when correspondence evidence is absent.
"""
from dataclasses import dataclass
import numpy as np
from scipy.spatial import cKDTree


class UnobservedSurface(ValueError):
    pass


@dataclass(frozen=True)
class Registration:
    translation: np.ndarray
    matched_points: np.ndarray
    residual_p90_m: float
    template_coverage: float
    distinct_template_points: int


def register_surface(template, template_rgb, observed, observed_rgb,
                     *, max_shift=.035, minimum_points=30, require_surface_extent=True):
    template=np.asarray(template,float); observed=np.asarray(observed,float)
    tc=np.asarray(template_rgb,float)/255.; oc=np.asarray(observed_rgb,float)/255.
    if (template.ndim!=2 or template.shape[1]!=3 or observed.ndim!=2 or observed.shape[1]!=3
            or tc.shape!=template.shape or oc.shape!=observed.shape
            or not all(np.isfinite(a).all() for a in (template,observed,tc,oc))):
        raise ValueError('invalid_registration_arrays')
    if min(len(template),len(observed))<minimum_points:
        raise UnobservedSurface('too_few_observed_surface_points')
    tree=cKDTree(template); shift=np.zeros(3)
    # Colour is local to each observed surface sample, including grey/black.
    def correspond(delta, distance):
        ds,ix=tree.query(observed-delta,k=min(8,len(template)))
        colour=np.linalg.norm(oc[:,None,:]-tc[ix],axis=2)
        valid=(ds<distance)&(colour<.28)
        cost=np.where(valid,ds+.015*colour,np.inf)
        best=cost.argmin(axis=1); rows=np.arange(len(observed))
        good=np.isfinite(cost[rows,best])
        return good,ix[rows,best],ds[rows,best]
    for _ in range(15):
        valid,ix,ds=correspond(shift,.020)
        if valid.sum()<minimum_points:
            raise UnobservedSurface('insufficient_geometry_colour_correspondences')
        step=np.median(observed[valid]-template[ix[valid]]-shift,axis=0)
        shift+=step
        if max_shift is not None and np.linalg.norm(shift)>max_shift:
            raise UnobservedSurface('surface_moved_beyond_registration_window')
        if np.linalg.norm(step)<.00015:break
    valid,ix,ds=correspond(shift,.008)
    actual=observed[valid]; unique=np.unique(ix[valid])
    if len(actual)<minimum_points or len(unique)<max(15,minimum_points//2):
        raise UnobservedSurface('insufficient_distinct_surface_correspondences')
    # Coverage is measured in object-relative principal directions, rather than
    # requiring 7.5 cm of world-Y width (which excluded small and rotated objects).
    _,_,axes=np.linalg.svd(template-template.mean(0),full_matrices=False)
    extent=np.ptp(template@axes.T,axis=0)
    matched_extent=np.ptp(template[unique]@axes.T,axis=0)
    significant=extent>.006
    if significant.sum()<2:
        raise UnobservedSurface('reference_surface_is_degenerate')
    ratios=matched_extent[significant]/extent[significant]
    if require_surface_extent and np.count_nonzero(ratios>=.35)<2:
        raise UnobservedSurface('only_tiny_surface_fragment_visible')
    return Registration(shift,actual,float(np.quantile(ds[valid],.9)),
                        len(unique)/len(template),len(unique))


def outside_boxes(points, vertices, margin=.004):
    """Remove measured points inside actual oriented robot collider boxes."""
    p=np.asarray(points); keep=np.ones(len(p),bool)
    for v in vertices:
        v=np.asarray(v); center=v.mean(0)
        _,_,axes=np.linalg.svd(v-center,full_matrices=False)
        half=np.max(abs((v-center)@axes.T),axis=0)
        keep &= ~(abs((p-center)@axes.T)<=half+margin).all(axis=1)
    return keep


def visible_reference(points,camera_origin,boxes,margin=.002):
    """Predict reference points hidden behind robot boxes, not their positions.

    A ray must intersect a box BEFORE the reference point. Never count this
    visibility prediction itself as a new observed surface measurement.
    """
    p=np.asarray(points,float);origin=np.asarray(camera_origin,float)
    visible=np.ones(len(p),bool)
    for vertices in boxes:
        v=np.asarray(vertices);center=v.mean(0)
        _,_,axes=np.linalg.svd(v-center,full_matrices=False)
        half=np.max(abs((v-center)@axes.T),axis=0)+margin
        start=(origin-center)@axes.T
        if (abs(start)<=half).all():continue  # optical origin inside housing
        direction=(p-origin)@axes.T
        parallel=abs(direction)<1e-12
        safe=np.where(parallel,1.,direction)
        aa=(-half-start)/safe;bb=(half-start)/safe
        lower=np.where(parallel,-np.inf,np.minimum(aa,bb))
        upper=np.where(parallel,np.inf,np.maximum(aa,bb))
        miss=(parallel&(abs(start)>half)).any(1)
        entry=np.maximum(lower.max(1),0.);leave=upper.min(1)
        visible &= ~(~miss&(entry<=leave)&(leave>0)&(entry<.995))
    return visible


def dominant_component(points, radius=.010, minimum=30):
    """Split a detection crop by measured 3D connectivity; retain one component.

    Similar-size disconnected components are ambiguous, not silently merged.
    """
    from scipy.sparse.csgraph import connected_components
    p=np.asarray(points)
    if len(p)<minimum:raise UnobservedSurface('insufficient_instance_surface')
    pairs=cKDTree(p).sparse_distance_matrix(cKDTree(p),radius,output_type='coo_matrix')
    _,labels=connected_components(pairs,directed=False)
    sizes=np.bincount(labels); rank=np.argsort(sizes)[::-1]
    if sizes[rank[0]]<minimum:raise UnobservedSurface('fragmented_instance_surface')
    if len(rank)>1 and sizes[rank[1]]>sizes[rank[0]]*.60:
        raise UnobservedSurface('ambiguous_disconnected_instance_surfaces')
    return labels==rank[0]
