"""Name-blind observed-surface candidate generation and physical hand checks.

Object labels never enter this module. Unknown/hidden surfaces are not claimed
measured. A plan is a proposal; execution requires a fresh observation and lift
verification. Local spans need actual contact-band samples, not a bbox centre.
"""
from dataclasses import dataclass,asdict
from collections import Counter
import math,hashlib,json
import numpy as np
from .kinematics import forward,posture,CONTACT

@dataclass(frozen=True)
class Policy:
 jaw_inner_width_m:float=.093026
 side_attempt_max_width_m:float=.095
 side_margin_m:float=.004
 side_forward_insertion_m:float=.02
 table_margin_m:float=.005
 min_contact_overlap_m:float=.003
 observation_voxel_m:float=.003
 max_lift_m:float=.69
 min_points:int=60
 max_candidates:int=12
 feasible_budget:int=0
 rotation_search:bool=False
 neutral_wrist_only:bool=False
 max_corrections:int=5
 lift_check_m:float=.08
 preferred_strategy:str='side'
 allowed_strategies:tuple=('side','overhead')
 lock_selected_pose:bool=True
 template_family:str='generic'
 sequential_side:bool=False
 min_protrusion_length_m:float=.01
 side_height_fractions:tuple=(.3,.45,.6,.75,.9)

@dataclass
class Scene:
 target:np.ndarray
 environment:np.ndarray
 plane:np.ndarray
 stamp_ns:int
 frame:str='base_link'
 def validate(self,policy):
  for points in (self.target,self.environment):
   if points.ndim!=2 or points.shape[1]!=3 or not np.isfinite(points).all():raise ValueError('invalid_cloud')
  if len(self.target)<policy.min_points:raise ValueError('insufficient_observed_target')
  if np.asarray(self.plane).shape!=(3,) or not np.isfinite(self.plane).all():raise ValueError('invalid_support_plane')
  if self.stamp_ns<=0:raise ValueError('missing_observation_stamp')

class Hand:
 def __init__(self,data):
  self.sweep=np.asarray(data['sweep_vertices'],float)
  self.offset=np.asarray(data['sweep_offsets'],float)
  self.open=np.asarray(data['open_boxes'],float)
  self.contacts=np.asarray(data['open_contacts'],float)
  self.uncertainty=float(data['uncertainty_m'])
  self.environment_offset=float(data['environment_offset_m'])
  self.verified=bool(data['complete_physical_sweep_verified'])
  if not self.verified or self.sweep.ndim!=3 or self.sweep.shape[1:]!=(8,3) or self.offset.shape!=(len(self.sweep),):raise ValueError('complete_verified_physical_hand_required')
  if self.contacts.shape!=(2,8,3):raise ValueError('two_measured_finger_pads_required')
  if not all(np.isfinite(x).all() for x in (self.sweep,self.offset,self.open,self.contacts)):raise ValueError('invalid_hand_geometry')
  # Exact oriented boxes from the exported eight vertices; reused per candidate.
  self.boxes=[]
  for v in self.open:
   c=v.mean(0);_,_,vh=np.linalg.svd(v-c,full_matrices=False);h=np.max(abs((v-c)@vh.T),axis=0)
   self.boxes.append((c,vh.T,h))


def voxel(points,size):
 if not len(points):return points.copy()
 _,ix=np.unique(np.floor(points/size).astype(np.int64),axis=0,return_index=True)
 return points[np.sort(ix)]

def clusters(values,gap=.009):
 s=np.sort(values);cuts=np.r_[0,np.where(np.diff(s)>gap)[0]+1,len(s)]
 return [s[a:b] for a,b in zip(cuts[:-1],cuts[1:]) if b-a>=4]

def _open_collision(points,palm,R,hand,allowance=.0005,allow_target_contact=False):
 if not len(points):return 0
 local=(points-palm)@R
 count=np.zeros(len(points),bool)
 for c,axes,half in hand.boxes:
  count|=(abs((local-c)@axes)<=half+allowance).all(1)
 if allow_target_contact:
  from .target_contact import permitted_target_contact
  count &= ~permitted_target_contact(points,points,palm,R,hand.contacts)
 return int(count.sum())

def _minimum_palm_height(xy,R,plane,hand,policy):
 vertices=hand.sweep@R.T
 floors=(vertices[:,:,:2]+xy)@plane[:2]+plane[2]-vertices[:,:,2]
 return float(np.max(floors.max(1)+hand.offset)+hand.uncertainty+hand.environment_offset+policy.table_margin_m)

def _key(strategy,center,angle,object_center):
 # Translation-relative identity only; rotation changes require new association.
 rel=np.round((center-object_center)/.015).astype(int).tolist()
 return hashlib.sha256(json.dumps([strategy,rel,round(angle,2)]).encode()).hexdigest()[:16]

def generate(scene,policy):
 p=voxel(scene.target,policy.observation_voxel_m);low,high=np.quantile(p,[.02,.98],axis=0)
 bottom=float(np.r_[np.median(p[:,:2],axis=0),1.]@scene.plane);height=float(high[2]-bottom)
 if not .006<height<.5:return []
 rows=[]
 if policy.template_family=='neck_bottle':
  from .neck_sections import sections
  _,_,_,necks=sections(p)
  for _,width,center in necks:
   rows.append(dict(strategy='side',angle=0.,center=center.copy(),width=width))
 # Horizontal pinch: enumerate heights and separate real local material spans.
 for frac in policy.side_height_fractions:
  z=bottom+frac*height
  band=p[abs(p[:,2]-z)<.0175]
  if len(band)<8:continue
  for x in np.quantile(band[:,0],[.15,.35,.5,.65,.8]):
   local=band[abs(band[:,0]-x)<.0125]
   for span in clusters(local[:,1]):
    lo,hi=np.quantile(span,[.03,.97]);width=hi-lo
    if .008<width<policy.jaw_inner_width_m-2*policy.side_margin_m:
     rows.append(dict(strategy='side',angle=0.,center=np.array([x+policy.side_forward_insertion_m,(lo+hi)/2,z]),width=float(width)))
 # Top-down pinch: rotate around the vertical axis; split disconnected spans
 # (e.g. two handles) instead of proposing the empty gap between them.
 cov=np.cov(p[:,:2].T);e,v=np.linalg.eigh(cov);major=v[:,np.argmax(e)];theta=math.atan2(major[1],major[0])
 angles=sorted(set(round(x,4) for x in (0.,math.pi/2,theta,theta+math.pi/2)))
 if policy.rotation_search:
  angles=sorted(set(angles+[round(theta+a,4) for a in np.arange(0,math.pi,math.pi/6)]))
 if policy.neutral_wrist_only:angles=[0.]
 for alpha in angles:
  # R[:,1] is the closing direction; derive it from calibrated robot kinematics.
  roll=(alpha+math.pi/2)%math.pi-math.pi/2
  _,R=forward(posture('overhead',roll));closing=R[:2,1];along=R[:2,0]
  lateral=p[:,:2]@closing;longitudinal=p[:,:2]@along
  for value in np.quantile(longitudinal,[.1,.25,.4,.55,.7,.85,.95]):
   idx=abs(longitudinal-value)<.0175
   for span in clusters(lateral[idx]):
    lo,hi=np.quantile(span,[.03,.97]);width=hi-lo
    if not .008<width<policy.jaw_inner_width_m-2*policy.side_margin_m:continue
    local=p[idx&(lateral>=lo)&(lateral<=hi)]
    if len(local)<8:continue
    xy=closing*(lo+hi)/2+along*value;top=float(np.quantile(local[:,2],.95))
    rows.append(dict(strategy='overhead',angle=float(roll),center=np.r_[xy,top-.012],width=float(width)))
 return rows

def evaluate(row,scene,hand,policy):
 center=row['center'].copy();zero,R=forward(posture(row['strategy'],row['angle']));palm=center-R@CONTACT
 minimum=_minimum_palm_height(palm[:2],R,scene.plane,hand,policy)
 palm[2]=max(palm[2],minimum);center=palm+R@CONTACT
 lift=float(palm[2]-zero[2])
 if not 0<=lift<=policy.max_lift_m-policy.lift_check_m:return None,'joint_range_or_lift_headroom'
 # Map actual visible target points into the proposed palm frame. Pads close
 # along palm Y; require material in the forward/height band of both pads.
 local=(scene.target-palm)@R
 bounds=np.stack([hand.contacts.min(1),hand.contacts.max(1)],axis=1)
 lo=bounds[:,0].max(0);hi=bounds[:,1].min(0)
 band=(local[:,0]>=lo[0])&(local[:,0]<=hi[0])&(local[:,2]>=lo[2])&(local[:,2]<=hi[2])
 q=local[band & (abs(local[:,1])<policy.jaw_inner_width_m/2+.01)]
 if len(q)<8:return None,'no_observed_material_between_actual_pads'
 yy=np.quantile(q[:,1],[.03,.97]);opening_lo=float(bounds[:,1,1].min());opening_hi=float(bounds[:,0,1].max())
 if yy[0]<opening_lo+policy.side_margin_m or yy[1]>opening_hi-policy.side_margin_m:return None,'material_does_not_fit_open_gap'
 width=float(yy[1]-yy[0])
 if width<.008:return None,'insufficient_observed_pinch_width'
 if row['strategy']=='side' and width>policy.side_attempt_max_width_m+1e-9:
  return None,'local_side_width_exceeds_side_attempt_limit'
 # Avoid centring on a void: both halves and the middle must contain material.
 bins=np.histogram(q[:,1],bins=6,range=(yy[0],yy[1]))[0]
 if np.count_nonzero(bins)<4 or not bins[2:4].any():return None,'unobserved_or_empty_contact_region'
 spans=np.ptp(q[:,[0,2]],axis=0)
 span=float(spans.min())
 if span<policy.min_contact_overlap_m:return None,'contact_overlap_too_small'
 # Check the whole open hand on the approach, not merely the chosen contact
 # point. Target collision is only intentional after the explicit close action.
 approach=R[:,2]
 for d in (0.,.025,.07,.14):
  pp=palm-approach*d
  if _open_collision(scene.target,pp,R,hand,allow_target_contact=d<=.025)>2:return None,'open_hand_hits_target_before_closure'
  if _open_collision(scene.environment,pp,R,hand)>2:return None,'approach_hits_surroundings'
 object_center=np.median(scene.target,axis=0);dist=float(np.linalg.norm(center[:2]-object_center[:2]))
 contact_score=min(len(q)/40,1.);width_score=max(0.,1-abs(width-.045)/.045)
 score=2*contact_score+width_score-3*dist
 from .shape_contacts import contact_preference
 shape,shape_error=contact_preference(scene.target,center,policy.template_family,policy.min_protrusion_length_m)
 if shape_error:return None,shape_error
 score+=shape['score']
 if policy.template_family!='generic' and row['strategy']=='side':
  top=float(np.quantile(scene.target[:,2],.98));bottom=float(np.r_[center[:2],1.]@scene.plane)
  preferred=policy.side_height_fractions[0]
  score-=2*abs((center[2]-bottom)/max(top-bottom,.006)-preferred)
 return dict(id=_key(row['strategy'],center,row['angle'],object_center),strategy=row['strategy'],wrist_roll_rad=row['angle'],
  grasp_point_base_m=center.tolist(),palm_base_m=palm.tolist(),arm_goal=posture(row['strategy'],row['angle'],lift),
  observed_width_m=width,observed_contact_points=len(q),contact_overlap_m=span,
  contact_extent_pad_xz_m=spans.tolist(),score=float(score),shape_contact=shape,
  minimum_palm_height_m=minimum,table_clearance_policy_m=policy.table_margin_m,
  object_relative_grasp_m=(center-object_center).tolist(),observation_stamp_ns=scene.stamp_ns,
  checks=dict(complete_physical_closing_sweep_table=True,open_approach_target=True,open_approach_environment=True,observed_contact_material=True),
  geometry_is_partial_observation=True,motion_authorized=False),None

def minimum_horizontal_width(points):
 # Minimum support-line separation of the measured horizontal convex hull.
 from scipy.spatial import ConvexHull,QhullError
 xy=np.unique(np.asarray(points)[:,:2],axis=0)
 if len(xy)<3:return 0.
 try:hull=xy[ConvexHull(xy).vertices]
 except QhullError:return 0.
 edges=np.roll(hull,-1,axis=0)-hull
 normals=np.c_[-edges[:,1],edges[:,0]]
 normals/=np.linalg.norm(normals,axis=1)[:,None]
 return float(np.min(np.ptp(xy@normals.T,axis=0)))

def plan(scene,hand,policy=Policy(),excluded=()):
 if not 0<=policy.side_forward_insertion_m<=.04:raise ValueError('invalid_side_insertion')
 if policy.preferred_strategy not in ('overhead','side','score'):raise ValueError('invalid_strategy_preference')
 scene.validate(policy)
 if not policy.allowed_strategies or set(policy.allowed_strategies)-{'side','overhead'}:raise ValueError('invalid_allowed_strategies')
 min_width=minimum_horizontal_width(scene.target)
 if not math.isfinite(policy.side_attempt_max_width_m) or policy.side_attempt_max_width_m<=0:raise ValueError('invalid_side_attempt_width')
 # Whole-object width is diagnostic only; every local side span is evaluated.
 width_checks=[]
 rejected=Counter();by_strategy={};candidates=[];excluded=set(excluded)
 raw_rows=generate(scene,policy)
 if policy.feasible_budget:
  raw_rows=sorted(raw_rows,key=lambda r:0 if r['strategy']==policy.preferred_strategy else 1)
 evaluated=0;early=False
 for raw in raw_rows:
  reason=('strategy_disabled_by_user_policy' if raw['strategy'] not in policy.allowed_strategies else
          'invalid_local_side_width' if raw['strategy']=='side' and (not math.isfinite(raw['width']) or raw['width']<=0) else
          'local_side_width_exceeds_side_attempt_limit' if raw['strategy']=='side' and raw['width']>policy.side_attempt_max_width_m+1e-9 else None)
  candidate=None
  if reason is None:
   evaluated+=1;candidate,reason=evaluate(raw,scene,hand,policy)
  if raw['strategy']=='side':width_checks.append(dict(local_span_width_m=float(raw['width']),
   evaluated_contact_width_m=None if candidate is None else candidate.get('observed_width_m'),
   accepted=reason is None,reason=reason))
  if reason:
   rejected[reason]+=1;by_strategy.setdefault(raw['strategy'],Counter())[reason]+=1;continue
  if candidate['id'] in excluded:rejected['previous_failed_candidate']+=1;continue
  if any(candidate['id']==c['id'] or candidate['strategy']==c['strategy'] and abs(candidate['wrist_roll_rad']-c['wrist_roll_rad'])<.1 and np.linalg.norm(np.array(candidate['grasp_point_base_m'])-c['grasp_point_base_m'])<.012 for c in candidates):continue
  candidates.append(candidate)
  if policy.feasible_budget and len(candidates)>=policy.feasible_budget:
   early=True;break
 candidates.sort(key=lambda c:(0 if policy.preferred_strategy=='score' or c['strategy']==policy.preferred_strategy else 1,-c['score']))
 return dict(status='candidate_selected' if candidates else 'additional_observation_required',selected=candidates[0] if candidates else None,
  candidates=candidates[:policy.max_candidates],rejected=dict(rejected),policy=asdict(policy),
  evaluated_candidates=evaluated,early_exit=early,
  rejected_by_strategy={k:dict(v) for k,v in by_strategy.items()},
  feasible_by_strategy=dict(Counter(c['strategy'] for c in candidates)),
  observed_minimum_horizontal_width_m=min_width,side_excluded_by_width=False,
  whole_object_width_gate_enabled=False,side_candidate_width_checks=width_checks,
  horizontal_width_basis='whole_object_diagnostic_only_local_side_spans_authorize_attempts',
  preferred_strategy=policy.preferred_strategy,
  preference_fallback=bool(candidates and policy.preferred_strategy!='score' and candidates[0]['strategy']!=policy.preferred_strategy),
  selection_basis='observed_geometry_and_complete_hand_model',object_name_used_for_strategy=False,
  manual_grasp_point_used=False,observation_stamp_ns=scene.stamp_ns,motion_authorized=False)
