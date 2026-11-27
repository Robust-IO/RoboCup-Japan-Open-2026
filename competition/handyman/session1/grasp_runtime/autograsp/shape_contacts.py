"""Observed contact-region preferences; protrusions need a measured root and tip.

Only visible points support these measurements. Object names choose a family,
never supply historical grasp coordinates or missing geometry.
"""
import numpy as np

def axes(points):
    p=np.asarray(points,float);center=np.median(p,axis=0)
    _,v=np.linalg.eigh(np.cov((p-center).T))
    return center,v[:,::-1]

from functools import lru_cache

@lru_cache(maxsize=16)
def _regions(data,shape):
    p=np.frombuffer(data,dtype=np.float64).reshape(shape);center,basis=axes(p)
    directions=[*basis.T,*np.eye(3)]
    # Additional diagonals cover oblique arms/ears without repeating the full
    # cross-section analysis separately for every grasp candidate.
    for a in ((1,1,0),(1,-1,0),(1,0,1),(1,0,-1),(0,1,1),(0,1,-1)):
        d=np.asarray(a,float);directions.append(d/np.linalg.norm(d))
    regions=[]
    for axis in directions:
        for sign in (-1,1):
            direction=axis*sign;u=(p-center)@direction;tip=float(np.quantile(u,.98))
            transverse=(p-center)-u[:,None]*direction;previous=[]
            for distance in np.arange(.005,min(.18,float(np.ptp(u))),.005):
                q=transverse[(u>=tip-distance-.005)&(u<tip-distance)]
                if len(q)<4:previous=[];continue
                offset=np.median(q,axis=0)
                width=float(2*np.quantile(np.linalg.norm(q-offset,axis=1),.9))
                if previous:
                    shaft=float(np.median([r[0] for r in previous[-3:]]))
                    if width>=max(shaft*1.5,shaft+.01):
                        shaft_offset=np.median([r[1] for r in previous[-3:]],axis=0)
                        regions.append((center,direction,tip,tip-distance,float(distance-.005),shaft,shaft_offset))
                        break
                previous.append((width,offset))
    return regions

def protrusion(points,contact,minimum=.01):
    p=np.ascontiguousarray(points,dtype=np.float64);contact=np.asarray(contact,float);best=None
    for center,direction,tip,root,length,shaft,offset in _regions(p.tobytes(),p.shape):
        if length+1e-9<minimum:continue
        u=float((contact-center)@direction);lateral=contact-center-u*direction
        if root-.003<=u<=tip+.003 and np.linalg.norm(lateral-offset)<=max(shaft/2,.01)+.005:
            proof=dict(length_m=length,root_base_m=(center+direction*root+offset).tolist(),
                       tip_base_m=(center+direction*tip+offset).tolist(),basis='observed_connected_narrow_section_to_wider_root')
            if best is None or proof['length_m']>best['length_m']:best=proof
    return best

def contact_preference(points,contact,family,minimum=.01):
    p=np.asarray(points,float);c=np.asarray(contact,float);center,basis=axes(p)
    scale=max(float(np.linalg.norm(np.ptp(p,axis=0))),.01)
    if family=='irregular_body':
        proof=protrusion(p,c,minimum)
        return (None,'no_observed_protrusion_at_least_1cm') if proof is None else (dict(score=1.,protrusion=proof),None)
    if family=='sphere':
        center=(np.quantile(p,.02,axis=0)+np.quantile(p,.98,axis=0))/2
        return dict(score=-5*float(np.linalg.norm(c[1:]-center[1:]))/scale,region='diameter'),None
    direction=basis[:,0];u=(p-center)@direction;at=float((c-center)@direction)
    lo,hi=np.quantile(u,[.02,.98]);length=max(hi-lo,.01)
    if family=='neck_bottle':
        from .neck_sections import sections
        origin,axis,half,necks=sections(p)
        if not necks:return None,'neck_requires_new_observation'
        at=float((c-origin)@axis)
        a,w,_=min(necks,key=lambda x:abs(x[0]-at))
        if abs(a-at)>half:return None,'contact_outside_observed_neck'
        return dict(score=2.-abs(a-at)/half,region='observed_narrow_neck',neck_width_m=w),None
    if family in ('round_body','lying_body'):
        return dict(score=1.-3*abs(at-(lo+hi)/2)/length,region='cylinder_body'),None
    return dict(score=0.,region='local_material'),None
