"""Bind physical-envelope planning to a current palm pose and measured table."""
import json,hashlib
from pathlib import Path
import numpy as np
from gripper_physical_envelope import PhysicalHand
from thin_object_table_safety import plan,require_plan
from hand_camera_ray import rotation
from apple_transfer_auto_plan import map_point

DEFAULT_GEOMETRY=Path('/__HANDYMAN_PACKAGE__/runtime/hsr-gripper-colliders-20261101-171937.json')
PREFAB=Path('/__HANDYMAN_PACKAGE__/unity-reference/Assets/SIGVerse/Models/Robot/HSR/Prefabs/HSR.prefab')

def longitudinal_overlap(center,axis,length,width,palm,orientation,regions,minimum=.008):
    """Both real open-pad forward ranges overlap the box by >=8mm.

    Lateral centering and physical tabletop clearance remain separate checks.
    Uses the fully open pads, rather than assuming the box centre must equal
    one arbitrary trigger centre. Hidden object faces retain measured sizes.
    """
    c=np.asarray(center,float);a=np.asarray(axis,float);a=a/np.linalg.norm(a);n=np.array([-a[1],a[0],0.])
    half=(abs(a[0])*length+abs(n[0])*width)/2;lo,hi=c[0]-half,c[0]+half
    overlaps=[];ranges=[]
    for region in regions:
        v=np.asarray(region['vertices_palm_m'])@np.asarray(orientation).T+np.asarray(palm)
        low,high=float(v[:,0].min()),float(v[:,0].max());ranges.append([low,high]);overlaps.append(max(0.,min(hi,high)-max(lo,low)))
    if len(overlaps)!=2:raise ValueError('two_actual_finger_regions_required')
    return dict(ready=bool(min(overlaps)>=minimum),overlap_m=[float(v) for v in overlaps],minimum_required_overlap_m=float(minimum),
        box_forward_interval_m=[float(lo),float(hi)],open_pad_forward_intervals_m=ranges,
        criterion='actual_pad_and_box_side_overlap_not_exact_center')

def closure_region(center,axis,model,palm,orientation,regions,*,minimum_forward=.008,minimum_vertical=.003,side_clearance=.001):
    """A measured box fits between open fingers and overlaps both closing pads.

    Contact can be off-centre. Forward and vertical overlap use the actual
    pad ranges, not distance to a nominal grasp point. Table clearance is
    still checked separately over the complete physical closing sweep.
    """
    c=np.asarray(center,float);a=np.asarray(axis,float);r=np.asarray(orientation,float);p=np.asarray(palm,float)
    dimensions=np.array([model['length_m'],model['width_m'],model['height_m']],float)
    if (c.shape!=(3,) or a.shape!=(3,) or r.shape!=(3,3) or p.shape!=(3,)
        or not np.isfinite(np.r_[c,a,r.ravel(),p,dimensions]).all() or min(dimensions)<=0
        or np.linalg.norm(a[:2])<1e-6 or not np.allclose(r.T@r,np.eye(3),atol=1e-5)):
        raise ValueError('finite_measured_box_and_palm_required')
    # This is the established horizontal pinch posture, not a general tilted
    # grasp planner. Horizontal yaw changes are handled by these projections.
    forward=r[:,2].copy()
    if abs(forward[2])>.05:raise ValueError('horizontal_box_pinch_posture_required')
    forward[2]=0.;forward/=np.linalg.norm(forward);side=np.array([-forward[1],forward[0],0.])
    a=np.r_[a[:2]/np.linalg.norm(a[:2]),0.];n=np.array([-a[1],a[0],0.])
    frame=np.stack([forward,side,[0,0,1]],axis=1)
    corners=np.array([c+x*dimensions[0]/2*a+y*dimensions[1]/2*n+np.array([0,0,z*dimensions[2]/2])
                      for x in (-1,1) for y in (-1,1) for z in (-1,1)])@frame
    low,high=corners.min(0),corners.max(0);bounds=[]
    for region in regions:
        v=np.asarray(region['vertices_palm_m'],float)
        if v.shape!=(8,3) or not np.isfinite(v).all():raise ValueError('finite_actual_finger_regions_required')
        v=(v@r.T+p)@frame;bounds.append((v.min(0),v.max(0)))
    if len(bounds)!=2:raise ValueError('two_actual_finger_regions_required')
    bounds.sort(key=lambda row:float(row[0][1]))
    gap_low,gap_high=float(bounds[0][1][1]),float(bounds[1][0][1])
    forward_overlap=[float(max(0.,min(high[0],hi[0])-max(low[0],lo[0]))) for lo,hi in bounds]
    vertical_overlap=[float(max(0.,min(high[2],hi[2])-max(low[2],lo[2]))) for lo,hi in bounds]
    margins=[float(low[1]-gap_low),float(gap_high-high[1])]
    ready=min(margins)>=side_clearance and min(forward_overlap)>=minimum_forward and min(vertical_overlap)>=minimum_vertical
    return dict(ready=bool(ready),criterion='box_inside_open_gap_with_two_pad_contact_not_exact_center',
        forward_overlap_m=forward_overlap,vertical_overlap_m=vertical_overlap,open_side_clearance_m=margins,
        minimum_forward_overlap_m=minimum_forward,minimum_vertical_overlap_m=minimum_vertical,minimum_side_clearance_m=side_clearance,
        box_interval_in_grasp_frame_m=[low.tolist(),high.tolist()],open_gap_in_grasp_frame_m=[gap_low,gap_high])

def plane_in_map(snapshot,base_plane):
    tf=snapshot['tracking_transforms']['map_from_base_at_depth'];r=rotation(tf['quaternion']);a,b,c=base_plane
    n=r@np.array([-a,-b,1.]);d=c+n@np.asarray(tf['translation'])
    return np.array([-n[0]/n[2],-n[1]/n[2],d/n[2]])

class ThinBoxClearance:
    def __init__(self,geometry=DEFAULT_GEOMETRY):
        geometry=Path(geometry)
        self.hand=PhysicalHand(json.loads(geometry.read_text()),attached_object='cigarette#1')
        self.envelope=self.hand.envelope(PREFAB,contact_opening=self.hand.reference_opening)
        self.open_contacts=self.hand.contact_regions(PREFAB,.65)
        self.sha256=hashlib.sha256(geometry.read_bytes()).hexdigest()

    def proposal(self,snapshot,center_base,height,palm_base,rotation_base,plane_map):
        tf=snapshot['tracking_transforms']['map_from_base_at_depth'];r=rotation(tf['quaternion'])
        palm=map_point(snapshot,np.asarray(palm_base));center=map_point(snapshot,np.asarray(center_base))
        value=plan(dict(center_m=center.tolist(),height_m=height),plane_map,palm[:2],r@rotation_base,self.envelope)
        return dict(value,geometry_sha256=self.sha256,physical_colliders=len(self.hand.parts),
                    uncertainty_includes_5mm_arm_drift=True)

    def check(self,snapshot,palm,center,height,plane_map):
        value=self.proposal(snapshot,center,height,palm['translation'],rotation(palm['quaternion']),plane_map)
        actual=map_point(snapshot,np.asarray(palm['translation']))
        require_plan(value,float(actual[2]))
        value.update(actual_palm_height_map_m=float(actual[2]),height_above_hard_floor_m=float(actual[2]-value['minimum_allowed_palm_height_m']))
        return value

    def pinch_overlap(self,palm,center,axis,model):
        return longitudinal_overlap(center,axis,model['length_m'],model['width_m'],palm['translation'],rotation(palm['quaternion']),self.open_contacts)

    def closure_region(self,palm,center,axis,model,**kwargs):
        return closure_region(center,axis,model,palm['translation'],rotation(palm['quaternion']),self.open_contacts,**kwargs)
